//! The wiper control software component core (`Wiper_*` runnables).
//!
//! # Runnable set (AUTOSAR-style, REQ-WIP-021)
//!
//! | Runnable | Trigger | Purpose |
//! |---|---|---|
//! | `Wiper_Init` | init event | reset internal state |
//! | `Wiper_OnStalkChange` | data-received on stalk port | validate + cache stalk frame (mode, sensitivity, auto, wash button), REQ-WIP-015 fallback |
//! | `Wiper_Run10ms` | 10 ms cyclic (REQ-WIP-020) | FSM, wash sequencing, intermittent timing, diagnostics monitors, motor/pump outputs |
//! | `Wiper_Run100ms` | 100 ms cyclic | diagnostics snapshot (REQ-WIP-017) |
//!
//! # Determinism
//! All state is integral; time enters only via `t_ms`. No allocation, no floats,
//! no panic paths (enforced by lints and property tests).
//!
//! # Design notes
//! * The stalk frame travels through `Wiper_OnStalkChange` (data-received event).
//!   `valid=false` (COM-side alive supervision trip) activates the LOW fallback
//!   (REQ-WIP-015).
//! * One FSM cycle = physical transitions (angle feedback), then command handling.
//!   A wipe is counted exactly once, at the moment the back-sweep reaches park.
//! * After a wash release the FSM resumes the stalk mode present *at the end* of
//!   the 3-wipe sequence (documented interpretation of "3 wipes after release").
//! * Voltage inhibition freezes the FSM but keeps diagnostics running
//!   (REQ-WIP-014); arm-angle park detection backstops a stuck park switch
//!   (REQ-WIP-008/013).

#![allow(non_snake_case)] // AUTOSAR RTE API naming for runnables

use crate::params;
use crate::types::{
    DiagStatus, Dtc, Ignition, MotorCommand, MotorSpeed, Outputs, StalkMode, WashPhase, WiperFsm,
};

/// Internal sweep phase (finer than the reported `WiperFsm`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// At park, motor off; commands start a wipe from here.
    Parked,
    /// Moving from park to sweep top.
    Out,
    /// Moving back from top towards park.
    Back,
    /// At park, motor off, INT pause running.
    IntPause,
    /// Driving to park after OFF / fault and stopping there.
    ForcedPark,
}

/// Cached stalk frame (validated by `Wiper_OnStalkChange`).
#[derive(Debug, Clone, Copy)]
struct StalkCache {
    mode: StalkMode,
    sens: u8,
    auto: bool,
    wash_btn: bool,
    valid: bool,
}

/// Wiper control software component. The many `bool`s are monitor latches of a
/// control unit — grouped in one struct deliberately (AUTOSAR SWC instance style),
/// so `clippy::struct_excessive_bools` is allowed here.
#[allow(clippy::struct_excessive_bools)]
#[derive(Debug)]
pub struct WiperSwc {
    phase: Phase,
    wash: WashPhase,
    pump_active: bool,
    pump_started_ms: u32,
    pump_stop_ms: u32,
    pump_stop_reason_overrun: bool,
    pause_remaining_ms: u32,
    auto_hold: bool,
    stalk: StalkCache,
    /// Stall monitoring (REQ-WIP-012): 3 cooldown retries, then latch.
    stall_accum_ms: u32,
    stall_attempts: u8,
    stall_latched: bool,
    stall_firing: bool,
    stall_cooldown_until_ms: u32,
    /// Park plausibility (REQ-WIP-013).
    park_accum_ms: u32,
    park_implausible: bool,
    /// Voltage supervision (REQ-WIP-014).
    voltage_inhibit: bool,
    volt_recover_accum_ms: u32,
    volt_low: bool,
    /// Telemetry: completed wipes.
    wipe_count: u32,
    last_ignition: Ignition,
    /// Diagnostic bits.
    dtc_active: u16,
    dtc_latched: u16,
}

impl Default for WiperSwc {
    fn default() -> Self {
        Self::new()
    }
}

impl WiperSwc {
    /// Create the SWC in reset state.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            phase: Phase::Parked,
            wash: WashPhase::Idle,
            pump_active: false,
            pump_started_ms: 0,
            pump_stop_ms: 0,
            pump_stop_reason_overrun: false,
            pause_remaining_ms: 0,
            auto_hold: false,
            stalk: StalkCache {
                mode: StalkMode::Off,
                sens: 0,
                auto: false,
                wash_btn: false,
                valid: true,
            },
            stall_accum_ms: 0,
            stall_attempts: 0,
            stall_latched: false,
            stall_firing: false,
            stall_cooldown_until_ms: 0,
            park_accum_ms: 0,
            park_implausible: false,
            voltage_inhibit: false,
            volt_recover_accum_ms: 0,
            volt_low: false,
            wipe_count: 0,
            last_ignition: Ignition::On,
            dtc_active: 0,
            dtc_latched: 0,
        }
    }

    // ---------------------------------------------------------------- helpers

    #[must_use]
    fn dtc_bit(dtc: Dtc) -> u16 {
        for (i, d) in Dtc::ALL.iter().enumerate() {
            if *d == dtc {
                return 1u16 << i;
            }
        }
        0
    }

    fn set_dtc(&mut self, dtc: Dtc, active: bool) {
        let bit = Self::dtc_bit(dtc);
        if active {
            self.dtc_active |= bit;
            self.dtc_latched |= bit;
        } else {
            self.dtc_active &= !bit;
        }
    }

    /// Trusted park detection: park switch unless implausible, then angle decides.
    fn trusted_park(&self, angle_deg: u16, park_switch: bool) -> bool {
        if self.park_implausible {
            angle_deg <= params::PARK_ANGLE_DEG
        } else {
            park_switch || angle_deg <= params::PARK_ANGLE_DEG
        }
    }

    fn ignition_on(in_: &crate::types::Inputs) -> bool {
        in_.ignition == Ignition::On
    }

    // --------------------------------------------------------------- runnables

    /// `Wiper_Init`: soft reset of the volatile state.
    pub fn Wiper_Init(&mut self) {
        *self = Self::new();
    }

    /// `Wiper_OnStalkChange` (data-received): cache the validated stalk frame.
    pub fn Wiper_OnStalkChange(
        &mut self,
        mode: StalkMode,
        sensitivity: u8,
        auto: bool,
        wash_btn: bool,
        valid: bool,
    ) {
        self.stalk = StalkCache {
            mode,
            sens: sensitivity,
            auto,
            wash_btn,
            valid,
        };
        self.set_dtc(Dtc::InputTimeout, !valid);
    }

    /// `Wiper_Run100ms` diagnostics snapshot (REQ-WIP-017).
    #[must_use]
    pub fn Wiper_Run100ms(&self) -> DiagStatus {
        DiagStatus {
            dtc_active_bits: self.dtc_active,
            dtc_latched_bits: self.dtc_latched,
            fault_active: self.dtc_active != 0,
        }
    }

    /// Completed wipes (full cycles) — telemetry and verification.
    #[must_use]
    pub fn wipe_count(&self) -> u32 {
        self.wipe_count
    }

    /// Reported FSM state (for State Transitions panels/tests).
    #[must_use]
    pub fn fsm_state(&self) -> WiperFsm {
        match self.phase {
            Phase::Parked => WiperFsm::Parked,
            Phase::Out | Phase::Back => WiperFsm::Sweeping,
            Phase::IntPause => WiperFsm::IntPause,
            Phase::ForcedPark => WiperFsm::ReturningToPark,
        }
    }

    /// Current wash phase (for telemetry/tests).
    #[must_use]
    pub fn wash_phase(&self) -> WashPhase {
        self.wash
    }

    /// Stall latch state (for tests/simulators).
    #[must_use]
    pub fn stall_latched(&self) -> bool {
        self.stall_latched
    }

    /// Voltage inhibition state (for tests/simulators).
    #[must_use]
    pub fn voltage_inhibited(&self) -> bool {
        self.voltage_inhibit
    }

    /// Park implausibility state (for tests/simulators).
    #[must_use]
    pub fn park_implausible(&self) -> bool {
        self.park_implausible
    }

    /// Is `dtc` active right now (helper for tests)?
    #[must_use]
    pub fn is_active(&self, dtc: Dtc) -> bool {
        (self.dtc_active & Self::dtc_bit(dtc)) != 0
    }

    /// DTC history bitset (for tests).
    #[must_use]
    pub fn latched_bits(&self) -> u16 {
        self.dtc_latched
    }

    // -------------------------------------------------- 10 ms run + internals

    /// `Wiper_Run10ms`: the main control runnable.
    pub fn Wiper_Run10ms(&mut self, t_ms: u32, in_: &crate::types::Inputs) -> Outputs {
        let ignition = in_.ignition;
        let ignition_on = Self::ignition_on(in_);

        // Ignition edge clears the stall latch (power-up recovery, REQ-WIP-012).
        if in_.ignition != self.last_ignition {
            if ignition_on {
                self.stall_latched = false;
                self.stall_attempts = 0;
                self.stall_firing = false;
            }
            self.last_ignition = in_.ignition;
        }

        // Voltage supervision first: it gates everything below.
        self.voltage_update(in_.battery_v0p1);

        // Park switch plausibility is independent of voltage.
        self.park_plausibility_update(in_.arm_angle_deg, in_.park_switch);

        // Wash sequencing (pump cap / off-guard / release wipes).
        self.wash_update(t_ms, in_.ignition);

        // Auto-mode hold uses fresh rain + stalk validity.
        self.auto_hold_update(in_);

        // Stall monitor only while the motor is supposed to run.
        let motor_scheduled = matches!(self.phase, Phase::Out | Phase::Back | Phase::ForcedPark)
            && !self.voltage_inhibit
            && !self.stall_latched
            && !self.stall_firing;
        self.stall_monitor(t_ms, in_, motor_scheduled);

        if !self.voltage_inhibit {
            self.fsm_step(in_);
        }

        // ---------------- output mapping ----------------
        let motor_drive = matches!(self.phase, Phase::Out | Phase::Back | Phase::ForcedPark)
            && ignition_on
            && !self.voltage_inhibit
            && !self.stall_firing
            && !self.stall_latched;

        let speed = if motor_drive && self.stalk.valid && self.stalk.mode == StalkMode::High {
            MotorSpeed::High
        } else {
            MotorSpeed::Low
        };
        let motor = if motor_drive {
            MotorCommand {
                speed,
                duty_pct: if speed == MotorSpeed::High { 100 } else { 70 },
            }
        } else {
            MotorCommand::default()
        };

        Outputs {
            motor,
            pump_on: self.pump_active && ignition != Ignition::Off && !self.voltage_inhibit,
        }
    }

    fn voltage_update(&mut self, v: u16) {
        let out_of_range = !(params::UNDER_VOLTAGE_V0P1..=params::OVER_VOLTAGE_V0P1).contains(&v);
        let in_recovery = (params::RECOVERY_UNDER_V0P1..=params::RECOVERY_OVER_V0P1).contains(&v);
        if out_of_range {
            self.voltage_inhibit = true;
            self.volt_recover_accum_ms = 0;
            self.volt_low = v < params::UNDER_VOLTAGE_V0P1;
        } else if self.voltage_inhibit && in_recovery {
            // 1 s debounce before returning to actuation (REQ-WIP-014).
            self.volt_recover_accum_ms = self.volt_recover_accum_ms.saturating_add(10);
            if self.volt_recover_accum_ms >= 1000 {
                self.voltage_inhibit = false;
            }
        }
        self.set_dtc(Dtc::Undervoltage, self.voltage_inhibit && self.volt_low);
        self.set_dtc(Dtc::Overvoltage, self.voltage_inhibit && !self.volt_low);
    }

    fn park_plausibility_update(&mut self, angle_deg: u16, park_switch: bool) {
        if park_switch && angle_deg > params::OUT_OF_PARK_ANGLE_DEG {
            self.park_accum_ms = self.park_accum_ms.saturating_add(10);
            if self.park_accum_ms >= params::PARK_IMPLAUSIBLE_MS {
                self.park_implausible = true;
            }
        } else {
            self.park_accum_ms = 0;
            self.park_implausible = false;
        }
        self.set_dtc(Dtc::ParkSwitchImplausible, self.park_implausible);
    }

    fn wash_update(&mut self, t_ms: u32, ignition: Ignition) {
        if ignition == Ignition::Off {
            // REQ-WIP-001: no pump; abort the wash sequence.
            self.wash = WashPhase::Idle;
            self.pump_active = false;
            return;
        }
        match self.wash {
            // New holds (Idle) and re-press during release wipes share this path.
            WashPhase::Idle | WashPhase::AfterRelease { .. } => {
                let guarded =
                    t_ms.saturating_sub(self.pump_stop_ms) >= params::WASH_PUMP_OFFGUARD_MS;
                if self.stalk.wash_btn && guarded {
                    self.wash = WashPhase::Hold;
                    self.pump_started_ms = t_ms;
                    self.pump_active = true;
                }
            }
            WashPhase::Hold => {
                if t_ms.saturating_sub(self.pump_started_ms) >= params::WASH_PUMP_CAP_MS {
                    // Pump cap (REQ-WIP-010): pump off while still held.
                    self.pump_active = false;
                    self.pump_stop_ms = t_ms;
                    self.pump_stop_reason_overrun = true;
                }
                if !self.stalk.wash_btn {
                    self.pump_active = false;
                    self.pump_stop_ms = t_ms;
                    self.wash = WashPhase::AfterRelease {
                        wipes_left: params::WASH_RELEASE_WIPES,
                    };
                }
            }
        }
        // Release-wipe sequence ends when the count is exhausted.
        if let WashPhase::AfterRelease { wipes_left } = self.wash {
            if wipes_left == 0 {
                self.wash = WashPhase::Idle;
            }
        }
        // PumpOverrun DTC window: 300 ms blink around the cap event (REQ-WIP-010).
        self.set_dtc(
            Dtc::PumpOverrun,
            self.pump_stop_reason_overrun && t_ms.saturating_sub(self.pump_stop_ms) < 300,
        );
    }

    fn auto_hold_update(&mut self, in_: &crate::types::Inputs) {
        // REQ-WIP-015: stalk loss or rain-sensor loss → manual timing fallback.
        if !self.stalk.valid || !in_.rain_valid || !self.stalk.auto {
            self.auto_hold = false;
            return;
        }
        let threshold = params::auto_threshold_pct(self.stalk.sens);
        if in_.rain_pct >= threshold {
            self.auto_hold = true;
        } else if in_.rain_pct < threshold.saturating_sub(params::AUTO_HYSTERESIS_PCT) {
            self.auto_hold = false;
        }
    }

    fn stall_monitor(&mut self, t_ms: u32, in_: &crate::types::Inputs, motor_scheduled: bool) {
        if self.stall_latched {
            self.set_dtc(Dtc::MotorStall, true);
            return;
        }
        if self.stall_firing {
            // Cooldown window: motor forbidden; on expiry re-arm detection.
            if t_ms >= self.stall_cooldown_until_ms {
                self.stall_firing = false;
                self.stall_accum_ms = 0;
            }
            self.set_dtc(Dtc::MotorStall, true);
            return;
        }
        let stalled = motor_scheduled
            && in_.ignition == Ignition::On
            && in_.motor_current_dA > params::STALL_CURRENT_DA
            && i32::from(in_.arm_speed_dps).abs() < i32::from(params::STALL_ARM_SPEED_LIMIT_DPS);
        if stalled {
            self.stall_accum_ms = self.stall_accum_ms.saturating_add(10);
            if self.stall_accum_ms >= params::STALL_DETECT_MS {
                self.stall_accum_ms = 0;
                self.stall_attempts = self.stall_attempts.saturating_add(1);
                if self.stall_attempts > params::STALL_MAX_RETRIES {
                    // 3 retries exhausted → latch until ignition cycle (REQ-WIP-012).
                    self.stall_latched = true;
                } else {
                    self.stall_firing = true;
                    self.stall_cooldown_until_ms = t_ms.saturating_add(params::STALL_COOLDOWN_MS);
                }
                self.set_dtc(Dtc::MotorStall, true);
            }
        } else {
            self.stall_accum_ms = 0;
            self.set_dtc(Dtc::MotorStall, false);
        }
    }

    /// One FSM step: physical transitions first, then command handling.
    fn fsm_step(&mut self, in_: &crate::types::Inputs) {
        match self.phase {
            Phase::Out => {
                if in_.arm_angle_deg >= params::SWEEP_TOP_DEG {
                    self.phase = Phase::Back;
                }
            }
            Phase::Back => {
                if self.trusted_park(in_.arm_angle_deg, in_.park_switch) {
                    self.finalize_back(in_);
                }
            }
            Phase::ForcedPark => {
                if self.trusted_park(in_.arm_angle_deg, in_.park_switch) {
                    self.wipe_complete(in_);
                    self.phase = Phase::Parked;
                    self.pause_remaining_ms = 0;
                }
            }
            Phase::IntPause => {
                if self.pause_remaining_ms > 0 {
                    self.pause_remaining_ms = self.pause_remaining_ms.saturating_sub(10);
                }
                if self.auto_hold || self.pause_remaining_ms == 0 {
                    self.start_wipe();
                }
            }
            Phase::Parked => {}
        }
        self.command_step(in_);
    }

    fn start_wipe(&mut self) {
        self.phase = Phase::Out;
        self.pause_remaining_ms = 0;
    }

    /// The back-sweep reached park: count the wipe, then continue/pause/stop by mode.
    fn finalize_back(&mut self, in_: &crate::types::Inputs) {
        self.wipe_complete(in_);
        let ignition_on = Self::ignition_on(in_);
        let speed = if in_.speed_valid { in_.speed_kph } else { 0 };
        if self.stalk.valid && ignition_on {
            match self.stalk.mode {
                StalkMode::Int if self.auto_hold => {
                    // Continuous wiping (REQ-WIP-007): no IntPause detour at all.
                    self.start_wipe();
                }
                StalkMode::Int => {
                    self.pause_remaining_ms = params::int_pause_ms(self.stalk.sens, speed);
                    self.phase = Phase::IntPause;
                }
                StalkMode::Low | StalkMode::High => self.start_wipe(),
                StalkMode::Off => self.phase = Phase::Parked,
            }
        } else if ignition_on {
            // Stalk invalid → LOW fallback keeps wiping (REQ-WIP-015).
            self.start_wipe();
        } else {
            self.phase = Phase::Parked;
            self.pause_remaining_ms = 0;
        }
    }

    fn wipe_complete(&mut self, _in_: &crate::types::Inputs) {
        self.wipe_count = self.wipe_count.wrapping_add(1);
        // Reaching park proves the stall incident resolved (REQ-WIP-012).
        if !self.stall_firing {
            self.stall_attempts = 0;
        }
        if let WashPhase::AfterRelease { wipes_left } = self.wash {
            self.wash = WashPhase::AfterRelease {
                wipes_left: wipes_left.saturating_sub(1),
            };
        }
    }

    /// Mode/wash commands dominate after the physical step (start/stop wipes).
    fn command_step(&mut self, in_: &crate::types::Inputs) {
        let ignition_on = Self::ignition_on(in_);

        // Wash owns the FSM while Hold/AfterRelease is active (REQ-WIP-009).
        let wash_pending = match self.wash {
            WashPhase::Idle => false,
            WashPhase::Hold => true,
            WashPhase::AfterRelease { wipes_left } => wipes_left > 0,
        };
        if wash_pending {
            if matches!(self.phase, Phase::Parked | Phase::IntPause) {
                self.start_wipe();
            }
            return;
        }
        if !self.stalk.valid {
            // REQ-WIP-015 fallback: continuous LOW wiping.
            if ignition_on && matches!(self.phase, Phase::Parked | Phase::IntPause) {
                self.start_wipe();
            }
            return;
        }
        match self.stalk.mode {
            StalkMode::Low | StalkMode::High => {
                if ignition_on && matches!(self.phase, Phase::Parked | Phase::IntPause) {
                    self.start_wipe();
                }
            }
            StalkMode::Int => {
                if ignition_on && self.phase == Phase::Parked {
                    // Entering INT starts with an immediate wipe (industry practice).
                    self.pause_remaining_ms = 0;
                    self.phase = Phase::Out;
                }
            }
            StalkMode::Off => {
                // Complete the cycle, return to park, stop (REQ-WIP-008).
                match self.phase {
                    Phase::Out | Phase::Back => self.phase = Phase::ForcedPark,
                    Phase::IntPause => self.phase = Phase::Parked,
                    Phase::Parked | Phase::ForcedPark => {}
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::*;

    /// Deterministic plant mock: 2 deg per 10 ms tick while the motor runs,
    /// out then back; no motion when off or jammed. Park switch mirrors the angle.
    /// Monitor-latch struct too — allowed the bool count lint.
    #[allow(clippy::struct_excessive_bools)]
    struct Sim {
        swc: WiperSwc,
        t: u32,
        angle: u16,
        park_switch: bool,
        outbound: bool,
        jam: bool,
        current_dA: u16,
        arm_speed: i16,
        rain: u8,
        rain_valid: bool,
        v01: u16,
        speed_kph: u16,
        saw_int_pause: bool,
    }

    impl Sim {
        fn new() -> Self {
            Self {
                swc: WiperSwc::new(),
                t: 0,
                angle: 0,
                park_switch: true,
                outbound: true,
                jam: false,
                current_dA: 25,
                arm_speed: 200,
                rain: 0,
                rain_valid: true,
                v01: 135,
                speed_kph: 0,
                saw_int_pause: false,
            }
        }

        fn fresh(mode: StalkMode) -> Self {
            let mut s = Self::new();
            s.stalk(mode, 4, false, false, true);
            s.tick();
            s
        }

        fn stalk(&mut self, mode: StalkMode, sens: u8, auto: bool, wash_btn: bool, valid: bool) {
            self.swc
                .Wiper_OnStalkChange(mode, sens, auto, wash_btn, valid);
        }

        fn inputs(&self, ignition: Ignition) -> Inputs {
            Inputs {
                ignition,
                rain_pct: self.rain,
                rain_valid: self.rain_valid,
                speed_kph: self.speed_kph,
                speed_valid: true,
                park_switch: self.park_switch,
                motor_current_dA: self.current_dA,
                battery_v0p1: self.v01,
                arm_angle_deg: self.angle,
                arm_speed_dps: self.arm_speed,
            }
        }

        /// One 10 ms tick; plant mock integrates angle from the motor command.
        fn tick(&mut self) -> Outputs {
            self.tick_with(Ignition::On)
        }

        fn tick_with(&mut self, ignition: Ignition) -> Outputs {
            self.t += 10;
            let inp = self.inputs(ignition);
            let out = self.swc.Wiper_Run10ms(self.t, &inp);
            if self.swc.fsm_state() == WiperFsm::IntPause {
                self.saw_int_pause = true;
            }
            let running = out.motor.speed != MotorSpeed::Off;
            if self.jam {
                self.arm_speed = 0;
                return out; // arm blocked: no motion
            }
            let moving = running; // inertia is instantaneous in this mock; `jam` blocks
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
            self.park_switch = self.angle <= params::PARK_ANGLE_DEG;
            self.arm_speed = if moving { 200 } else { 0 };
            out
        }

        /// Run ticks until the reported FSM reaches the target (bounded).
        fn until(mut self, target: WiperFsm, max_ticks: u32) -> Self {
            for _ in 0..max_ticks {
                let _ = self.tick();
                if self.swc.fsm_state() == target {
                    return self;
                }
            }
            self
        }
    }

    // ------------------------------------------------------------ REQ-WIP-001/011

    #[test]
    // verifies: REQ-WIP-001, REQ-WIP-011
    fn fsm_ignition_gate() {
        // ACC: washer allowed, wiper not.
        let mut s = Sim::fresh(StalkMode::Off);
        s.stalk(StalkMode::Low, 4, false, true, true);
        let out = s.tick_with(Ignition::Off);
        assert_eq!(out.motor.speed, MotorSpeed::Off);
        assert!(!out.pump_on, "ignition off: no pump");

        // Use direct calls for ACC (tick helpers assume ON ignition).
        let mut swc = WiperSwc::new();
        swc.Wiper_OnStalkChange(StalkMode::Low, 4, false, true, true);
        let inp = Inputs::default();
        let mut acc = inp;
        acc.ignition = Ignition::Acc;
        let out = swc.Wiper_Run10ms(4000, &acc);
        assert!(out.pump_on, "washer allowed in ACC (REQ-WIP-011)");
        assert_eq!(out.motor.speed, MotorSpeed::Off, "wiper forbidden in ACC");
        let mut on = inp;
        on.ignition = Ignition::On;
        let out = swc.Wiper_Run10ms(4010, &on);
        assert_eq!(
            out.motor.speed,
            MotorSpeed::Low,
            "wiper allowed with ignition ON"
        );
        assert!(out.pump_on);
    }

    // ------------------------------------------------------------ REQ-WIP-002

    #[test]
    // verifies: REQ-WIP-002
    fn fsm_mode_switching() {
        let mut s = Sim::new();
        s.stalk(StalkMode::Off, 4, false, false, true);
        let out = s.tick();
        assert_eq!(out.motor.speed, MotorSpeed::Off, "OFF parks");

        // Mode change takes effect within exactly one tick.
        s.stalk(StalkMode::Low, 4, false, false, true);
        let out = s.tick();
        assert_eq!(
            out.motor.speed,
            MotorSpeed::Low,
            "LOW starts within one tick"
        );

        // Out→park return when switched off mid-sweep.
        s.stalk(StalkMode::Off, 4, false, false, true);
        let _ = s.tick();
        assert_eq!(s.swc.fsm_state(), WiperFsm::ReturningToPark);
    }

    // ------------------------------------------------------------ REQ-WIP-003/004

    #[test]
    // verifies: REQ-WIP-003, REQ-WIP-004
    fn fsm_low_high_continuous() {
        // LOW: wipe after wipe, no pause phase in between.
        let mut s = Sim::fresh(StalkMode::Low);
        for _ in 0..800 {
            let _ = s.tick();
        }
        assert!(s.swc.wipe_count() >= 5, "LOW wipes continuously");
        assert!(!s.saw_int_pause, "LOW never pauses");

        // HIGH uses the high profile at 100 % duty (REQ-WIP-018 mapping).
        let mut s = Sim::fresh(StalkMode::High);
        let out = s.tick();
        assert_eq!(out.motor.speed, MotorSpeed::High);
        assert_eq!(out.motor.duty_pct, 100);
    }

    // ------------------------------------------------------------ REQ-WIP-005

    #[test]
    // verifies: REQ-WIP-005
    fn intermittent_base_pause() {
        let mut s = Sim::new();
        s.stalk(StalkMode::Int, 1, false, false, true);
        let out = s.tick();
        assert_ne!(
            out.motor.speed,
            MotorSpeed::Off,
            "INT starts with an immediate wipe"
        );
        for _ in 0..200 {
            let _ = s.tick();
            if s.swc.fsm_state() == WiperFsm::IntPause {
                break;
            }
        }
        assert_eq!(s.swc.fsm_state(), WiperFsm::IntPause);
        // S1 base pause 2000 ms = 199 more ticks in pause, then a new sweep.
        for _ in 0..199 {
            let out = s.tick();
            assert_eq!(s.swc.fsm_state(), WiperFsm::IntPause, "pause not yet over");
            assert_eq!(out.motor.speed, MotorSpeed::Off);
        }
        let _ = s.tick();
        assert_eq!(
            s.swc.fsm_state(),
            WiperFsm::Sweeping,
            "new wipe after the pause"
        );
    }

    // ------------------------------------------------------------ REQ-WIP-006

    #[test]
    // verifies: REQ-WIP-006
    fn intermittent_speed_dependence() {
        // S2 at 80 km/h: base 5000 ms × 0.7 = 3500 ms pause.
        let mut s = Sim::new();
        s.stalk(StalkMode::Int, 2, false, false, true);
        s.speed_kph = 80;
        let _ = s.tick();
        for _ in 0..200 {
            let _ = s.tick();
            if s.swc.fsm_state() == WiperFsm::IntPause {
                break;
            }
        }
        assert_eq!(s.swc.fsm_state(), WiperFsm::IntPause);
        // 349 ticks in pause; the 350th tick expires it.
        for _ in 0..349 {
            let _ = s.tick();
            assert_eq!(
                s.swc.fsm_state(),
                WiperFsm::IntPause,
                "3500 ms pause counting"
            );
        }
        let _ = s.tick();
        assert_eq!(
            s.swc.fsm_state(),
            WiperFsm::Sweeping,
            "pause expires after 3500 ms"
        );
    }

    // ------------------------------------------------------------ REQ-WIP-007

    #[test]
    // verifies: REQ-WIP-007
    fn interop_auto_mode() {
        // Auto INT S3 (threshold 25 %). Rain 45 % ⇒ continuous wiping.
        let mut s = Sim::new();
        s.stalk(StalkMode::Int, 3, true, false, true);
        s.rain = 45;
        let _ = s.tick();
        for _ in 0..1500 {
            let _ = s.tick();
        }
        assert!(
            s.swc.wipe_count() >= 10,
            "rain above threshold wipes continuously"
        );
        assert!(
            !s.saw_int_pause,
            "no pauses while the rain is above threshold"
        );

        // Rain drop below threshold-5 (25-5=20): pauses come back.
        s.rain = 15;
        for _ in 0..200 {
            let _ = s.tick();
        }
        assert_eq!(
            s.swc.fsm_state(),
            WiperFsm::IntPause,
            "manual pause after rain drop"
        );
        // S3 base pause is 7 s: 699 further ticks remain paused.
        for _ in 0..400 {
            let _ = s.tick();
            assert_eq!(s.swc.fsm_state(), WiperFsm::IntPause);
        }
    }

    // ------------------------------------------------------------ REQ-WIP-008

    #[test]
    // verifies: REQ-WIP-008
    fn fsm_park_completion() {
        let mut s = Sim::fresh(StalkMode::High);
        // Sweep out to 60 deg, then switch Off mid-sweep.
        for _ in 0..30 {
            let _ = s.tick();
        }
        assert!(s.angle >= 50, "mid-sweep when leaving HIGH");
        s.stalk(StalkMode::Off, 4, false, false, true);
        let _ = s.tick();
        assert_eq!(s.swc.fsm_state(), WiperFsm::ReturningToPark);
        let mut s = s.until(WiperFsm::Parked, 200);
        assert!(
            s.angle <= params::PARK_ANGLE_DEG,
            "arm reached the park window"
        );
        // Nothing re-erupts with stalk off.
        for _ in 0..50 {
            let _ = s.tick();
        }
        assert_eq!(s.swc.fsm_state(), WiperFsm::Parked);
    }

    // ------------------------------------------------------------ REQ-WIP-009

    #[test]
    // verifies: REQ-WIP-009
    fn fsm_wash_sequence() {
        let mut s = Sim::new();
        s.stalk(StalkMode::Low, 4, false, false, true);
        let mut t = 4000;
        // Hold wash: pump runs.
        s.stalk(StalkMode::Low, 4, false, true, true);
        let out = s.swc.Wiper_Run10ms(t, &s.inputs(Ignition::On));
        assert!(out.pump_on, "pump on while the button is held");
        assert_eq!(s.swc.wash_phase(), WashPhase::Hold);

        // Release: 3 more wipes required (AfterRelease{3}).
        s.stalk(StalkMode::Low, 4, false, false, true);
        t += 10;
        s.t = t;
        let _ = s.tick();
        assert_eq!(
            s.swc.wash_phase(),
            WashPhase::AfterRelease { wipes_left: 3 }
        );

        // Drive the release wipes; exactly 3 wipes later the sequence ends.
        for _ in 0..900 {
            if s.swc.wash_phase() == WashPhase::Idle {
                break;
            }
            let _ = s.tick();
        }
        assert_eq!(
            s.swc.wash_phase(),
            WashPhase::Idle,
            "exactly 3 release wipes"
        );
        // Continuous mode resumes (LOW keeps wiping).
        let out = s.tick();
        assert_ne!(
            out.motor.speed,
            MotorSpeed::Off,
            "LOW resumes after the wash"
        );
    }

    // ------------------------------------------------------------ REQ-WIP-010/016

    #[test]
    // verifies: REQ-WIP-010, REQ-WIP-016
    fn fsm_wash_pump_cap() {
        let mut s = Sim::new();
        s.stalk(StalkMode::Off, 4, false, false, true);
        let mut t = 6000;
        s.stalk(StalkMode::Off, 4, false, true, true);
        let out = s.swc.Wiper_Run10ms(t, &s.inputs(Ignition::On));
        assert!(out.pump_on, "pump while held");
        // Hold continuously: pump stops after the 10 s cap.
        let mut pump_stopped_at = 0;
        for i in 0..1200u32 {
            t += 10;
            let out = s.swc.Wiper_Run10ms(t, &s.inputs(Ignition::On));
            if !out.pump_on && pump_stopped_at == 0 {
                pump_stopped_at = i * 10;
                break;
            }
        }
        assert_ne!(pump_stopped_at, 0, "pump cap stops it");
        assert!(pump_stopped_at <= 10_000, "cap at 10 s, not earlier");
        // Release; quick re-press inside 3 s must not pump (off-guard).
        s.stalk(StalkMode::Off, 4, false, false, true);
        t += 10;
        let _ = s.swc.Wiper_Run10ms(t, &s.inputs(Ignition::On));
        t += 100;
        s.stalk(StalkMode::Off, 4, false, true, true);
        let out = s.swc.Wiper_Run10ms(t, &s.inputs(Ignition::On));
        assert!(!out.pump_on, "3 s off-guard blocks retrigger");
        // After the off-guard a new hold pumps again.
        t += 3000;
        let out = s.swc.Wiper_Run10ms(t, &s.inputs(Ignition::On));
        assert!(out.pump_on, "new hold after off-guard pumps");
    }

    // ------------------------------------------------------------ REQ-WIP-012

    #[test]
    // verifies: REQ-WIP-012
    fn diag_stall_retry_latch() {
        let mut s = Sim::fresh(StalkMode::Low);
        // Jam the arm: current above threshold, no movement.
        s.jam = true;
        s.current_dA = 100;
        s.arm_speed = 0;
        let mut fired = false;
        for _ in 0..60 {
            let _ = s.tick();
            if s.swc.is_active(Dtc::MotorStall) {
                fired = true;
                break;
            }
        }
        assert!(fired, "stall fired after 500 ms of blockage");
        // Cooldown 2 s: motor off while cooling down.
        s.t += 10;
        let out = s.swc.Wiper_Run10ms(s.t, &s.inputs(Ignition::On));
        assert_eq!(out.motor.speed, MotorSpeed::Off, "motor off in cooldown");
        // After cooldown the retry re-commands the motor.
        for _ in 0..199 {
            s.t += 10;
            let _ = s.swc.Wiper_Run10ms(s.t, &s.inputs(Ignition::On));
        }
        s.t += 10;
        s.jam = false;
        s.outbound = true;
        s.angle = 40;
        let out = s.tick();
        assert_ne!(
            out.motor.speed,
            MotorSpeed::Off,
            "retry re-commands after cooldown"
        );

        // The third retry is also allowed; the *fourth* attempt must latch.
        // Keep the arm jammed the whole time (a recovered arm completing a wipe
        // would reset the attempt counter — that is the REQ-WIP-012 recovery path).
        s.jam = true;
        s.current_dA = 100;
        s.arm_speed = 0;
        for _ in 0..1200 {
            let _ = s.tick();
            if s.swc.stall_latched() {
                break;
            }
        }
        assert!(s.swc.stall_latched(), "4th attempt latches the failure");
        // Ignition cycle clears the latch (power-up recovery).
        for _ in 0..10 {
            s.t += 10;
            let _ = s.swc.Wiper_Run10ms(s.t, &s.inputs(Ignition::Off));
        }
        s.stalk(StalkMode::Off, 4, false, false, true);
        s.t += 10;
        let _ = s.swc.Wiper_Run10ms(s.t, &s.inputs(Ignition::On));
        assert!(!s.swc.stall_latched(), "ignition cycle clears the latch");
    }

    // ------------------------------------------------------------ REQ-WIP-013

    #[test]
    // verifies: REQ-WIP-013
    fn diag_park_implausible() {
        let mut s = Sim::fresh(StalkMode::Low);
        // Park switch stuck active while the arm is clearly out of park.
        s.jam = true;
        s.angle = 40;
        s.park_switch = true;
        for _ in 0..21 {
            let _ = s.tick();
        }
        assert!(s.swc.park_implausible(), "implausibility after 200 ms");
        assert!(s.swc.is_active(Dtc::ParkSwitchImplausible));
        // Switch released (unsticks): monitor clears.
        s.park_switch = false;
        let _ = s.tick();
        assert!(!s.swc.park_implausible());
    }

    // ------------------------------------------------------------ REQ-WIP-014

    #[test]
    // verifies: REQ-WIP-014
    fn diag_voltage_window() {
        let mut s = Sim::fresh(StalkMode::Low);
        // 8 V: under the 9 V window.
        s.v01 = 80;
        s.jam = false;
        let out = s.tick();
        assert!(s.swc.voltage_inhibited());
        assert_eq!(
            out.motor.speed,
            MotorSpeed::Off,
            "inhibited outputs are zero"
        );
        assert!(s.swc.is_active(Dtc::Undervoltage));
        // Back to 10 V: 1 s debounce before resume.
        s.v01 = 100;
        for _ in 0..99 {
            let _ = s.tick();
        }
        assert!(s.swc.voltage_inhibited(), "debounce active");
        let _ = s.tick();
        assert!(!s.swc.voltage_inhibited(), "resume after debounce");
        // Over-voltage separately inhibits with its own DTC.
        s.v01 = 170;
        let _ = s.tick();
        assert!(s.swc.voltage_inhibited());
        assert!(s.swc.is_active(Dtc::Overvoltage));
        assert!(!s.swc.is_active(Dtc::Undervoltage));
    }

    // ------------------------------------------------------------ REQ-WIP-015

    #[test]
    // verifies: REQ-WIP-015
    fn fsm_stalk_timeout_fallback() {
        let mut s = Sim::new();
        // Stalk frame lost: invalid → InputTimeout DTC + LOW fallback wiping.
        s.stalk(StalkMode::Off, 0, false, false, false);
        let out = s.tick();
        assert!(s.swc.is_active(Dtc::InputTimeout));
        assert_eq!(out.motor.speed, MotorSpeed::Low, "fallback wipes at LOW");
        assert_eq!(out.motor.duty_pct, 70);

        // Auto mode with rain-sensor loss: manual INT timing instead of
        // continuous wiping, and never parked-off.
        let mut s = Sim::new();
        s.stalk(StalkMode::Int, 2, true, false, true);
        s.rain = 60;
        s.rain_valid = false;
        let out = s.tick();
        assert_ne!(out.motor.speed, MotorSpeed::Off, "INT keeps wiping");
        for _ in 0..200 {
            let _ = s.tick();
            if s.swc.fsm_state() == WiperFsm::IntPause {
                break;
            }
        }
        assert_eq!(
            s.swc.fsm_state(),
            WiperFsm::IntPause,
            "manual timing after rain loss"
        );
    }

    // ------------------------------------------------------------ REQ-WIP-017

    #[test]
    // verifies: REQ-WIP-017
    fn diag_snapshot_cycle() {
        let mut s = Sim::fresh(StalkMode::Off);
        let snap = s.swc.Wiper_Run100ms();
        assert_eq!(snap.dtc_active_bits, 0);
        assert!(!snap.fault_active);
        // Voltage out of range is visible in the 100 ms snapshot.
        s.v01 = 80;
        let _ = s.tick();
        let snap = s.swc.Wiper_Run100ms();
        assert_eq!(
            snap.dtc_active_bits & (1 << 2),
            1 << 2,
            "undervoltage bit set"
        );
        assert!(snap.fault_active);
        // Latched history stays after recovery.
        s.v01 = 100;
        for _ in 0..110 {
            let _ = s.tick();
        }
        let snap = s.swc.Wiper_Run100ms();
        assert_eq!(snap.dtc_active_bits & (1 << 2), 0, "active bit cleared");
        assert_eq!(snap.dtc_latched_bits & (1 << 2), 1 << 2, "history kept");
    }
}
