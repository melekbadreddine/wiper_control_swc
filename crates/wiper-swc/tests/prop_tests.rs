//! Property tests for the wiper SWC invariants.
//!
//! Machine-checkable requirement tags: these names are referenced by
//! `docs/requirements.md` and enforced by `cargo xtask traceability`.

use proptest::prelude::*;
use wiper_swc::{params, Dtc, Ignition, Inputs, MotorSpeed, StalkMode, WiperFsm, WiperSwc};

/// Minimal deterministic plant mock (same model as the unit-test Sim: 2 deg/tick,
/// out then back, jam blocks motion; park switch mirrors the angle).
struct Plant {
    swc: WiperSwc,
    t: u32,
    angle: u16,
    park: bool,
    outbound: bool,
}

impl Plant {
    fn new() -> Self {
        Self {
            swc: WiperSwc::new(),
            t: 0,
            angle: 0,
            park: true,
            outbound: true,
        }
    }

    fn inputs(&self, ignition: Ignition, rain: u8, v01: u16, current: u16) -> Inputs {
        Inputs {
            ignition,
            rain_pct: rain,
            rain_valid: true,
            speed_kph: 0,
            speed_valid: true,
            park_switch: self.park,
            motor_current_dA: current,
            battery_v0p1: v01,
            arm_angle_deg: self.angle,
            arm_speed_dps: if current > wiper_swc::params::STALL_CURRENT_DA {
                0
            } else {
                200
            },
        }
    }

    fn tick(&mut self, ignition: Ignition, rain: u8, v01: u16, current: u16) -> wiper_swc::Outputs {
        self.t += 10;
        let inp = self.inputs(ignition, rain, v01, current);
        let out = self.swc.Wiper_Run10ms(self.t, &inp);
        let moving = ignition == Ignition::On
            && out.motor.speed != MotorSpeed::Off
            && current <= wiper_swc::params::STALL_CURRENT_DA;
        if moving && !self.outbound && self.angle <= params::PARK_ANGLE_DEG {
            self.outbound = true;
        }
        if moving {
            if self.outbound {
                self.angle = (self.angle + 2).min(110);
                if self.angle >= params::SWEEP_TOP_DEG {
                    self.outbound = false;
                }
            } else {
                self.angle = self.angle.saturating_sub(2);
            }
        }
        self.park = self.angle <= params::PARK_ANGLE_DEG;
        out
    }
}

fn stalk_strategy() -> impl Strategy<Value = StalkMode> {
    prop_oneof![
        30 => Just(StalkMode::Off),
        20 => Just(StalkMode::Int),
        25 => Just(StalkMode::Low),
        25 => Just(StalkMode::High),
    ]
}

// REQ-WIP-008 invariant: when the stalk goes to OFF with a valid frame, the arm
// must always stop *in park*, and the motor must be off. Also: no stall latch
// when the actuator is healthy (current is low).
proptest! {
    // verifies: REQ-WIP-008
    #[test]
    fn proptest_park_only_stop(
        seq in prop::collection::vec(stalk_strategy(), 0..8),
    ) {
        let mut p = Plant::new();
        p.swc.Wiper_OnStalkChange(StalkMode::Low, 3, false, false, true);
        for mode in seq {
            p.swc.Wiper_OnStalkChange(mode, 3, false, false, true);
            for _ in 0..40u32 {
                let _ = p.tick(Ignition::On, 0, 135, 25);
                prop_assert!(p.angle <= 110);
            }
        }
        // Switch off and wait for the guaranteed park completion.
        p.swc.Wiper_OnStalkChange(StalkMode::Off, 3, false, false, true);
        for _ in 0..300u32 {
            let _ = p.tick(Ignition::On, 0, 135, 25);
            prop_assert!(p.angle <= 110);
        }
        prop_assert_eq!(p.swc.fsm_state(), WiperFsm::Parked, "stops parked");
        prop_assert!(p.angle <= params::PARK_ANGLE_DEG, "stops only in park, angle={}", p.angle);
        prop_assert_eq!(p.swc.is_active(Dtc::MotorStall), false, "no false stall");
    }

    // verifies: REQ-WIP-014
    #[test]
    fn proptest_voltage_gate(
        voltages in prop::collection::vec(0u16..=65535, 0..6),
    ) {
        let mut p = Plant::new();
        p.swc.Wiper_OnStalkChange(StalkMode::Low, 4, false, false, true);
        for _ in 0..10u32 {
            let _ = p.tick(Ignition::On, 0, 135, 25);
        }
        for v in voltages {
            let out = p.tick(Ignition::On, 0, v, 25);
            if p.swc.voltage_inhibited() {
                prop_assert_eq!(out.motor.speed, MotorSpeed::Off, "inhibited motor");
                prop_assert!(!out.pump_on, "inhibited pump");
            }
            // The engine never commands duty outside 0..=100 (however computed).
            prop_assert!(out.motor.duty_pct <= 100);
        }
    }

    // verifies: REQ-WIP-019
    #[test]
    fn proptest_determinism(
        seq in prop::collection::vec((
            prop::option::of(stalk_strategy()),
            prop::option::of(0u8..=100),
            prop::option::of(0u16..=250),
        ), 0..40),
    ) {
        let mut a = Plant::new();
        let mut b = Plant::new();
        for (mode, rain, _speed) in &seq {
            if let Some(mode) = mode {
                a.swc.Wiper_OnStalkChange(*mode, 2, true, false, true);
                b.swc.Wiper_OnStalkChange(*mode, 2, true, false, true);
            }
            let rain = rain.unwrap_or(20);
            for _ in 0..12u32 {
                let oa = a.tick(Ignition::On, rain, 135, 25);
                let ob = b.tick(Ignition::On, rain, 135, 25);
                prop_assert_eq!(oa, ob, "identical histories");
            }
        }
    }

    // Arbitrary inputs never panic and outputs remain sane.
    // verifies: REQ-WIP-019
    #[test]
    fn proptest_never_panics(
        rain in 0u8..=255,
        current in 0u16..=65535,
        voltage in 0u16..=65535,
        angle in 0u16..=65535,
        speed in 0i16..=32767,
        ignition in prop_oneof![Just(Ignition::Off), Just(Ignition::Acc), Just(Ignition::On)],
        stalk in stalk_strategy(),
        sens in 0u8..=255,
    ) {
        let mut p = Plant::new();
        p.swc.Wiper_OnStalkChange(stalk, sens, true, rng_bool(sens), true);
        let mut inp = p.inputs(ignition, rain, voltage, current);
        inp.arm_angle_deg = angle;
        let arm_speed_dps = (i32::from(speed) * 2)
            .saturating_sub(32768)
            .clamp(-32768, i32::from(i16::MAX));
        inp.arm_speed_dps = i16::try_from(arm_speed_dps).unwrap_or(i16::MIN);
        let out = p.swc.Wiper_Run10ms(10, &inp);
        prop_assert!(out.motor.duty_pct <= 100);
        let _ = p.swc.Wiper_Run100ms();
    }
}

fn rng_bool(x: u8) -> bool {
    x % 2 == 0
}
