//! End-to-end closed-loop tests: sensor nodes → CAN bus → COM/RTE →
//! `WiperSwc` → `IoHwAb` → plant → diagnostics/status. All time is the ECU's
//! virtual time; one `Ecu::tick()` is one 10 ms base tick.

use bsw_sim::com::{decode_status, STALK_RX_TIMEOUT_MS, STATUS_FRAME_ID};
use bsw_sim::ecu::Ecu;
use wiper_swc::{Dtc, Ignition, MotorSpeed, StalkMode, WiperFsm};

/// Drive `ecu` for `ms` of virtual time.
fn run(ecu: &mut Ecu, ms: u32) {
    for _ in 0..ms.div_ceil(10) {
        ecu.tick();
    }
}

/// Count wipe completions during a `Window_ms` window (drives `first` first,
/// which must be pre-paid warmup when starting from a fully parked ECU).
fn wipe_rate_cpm(ecu: &mut Ecu, warmup_ms: u32, window_ms: u32) -> u32 {
    run(ecu, warmup_ms);
    let before = ecu.swc().wipe_count();
    run(ecu, window_ms);
    let after = ecu.swc().wipe_count();
    (after - before) * 60_000 / window_ms
}

fn ignition_on_low(ecu: &mut Ecu) {
    ecu.set_ignition(Ignition::On);
    ecu.sensors_mut().stalk.mode = StalkMode::Low;
}

/// Wipes take ≈1.3 s per full cycle; waits until at least one completed wipe.
fn wait_for_first_wipe(ecu: &mut Ecu) {
    for _ in 0..(4_000 / 10) {
        ecu.tick();
        if ecu.swc().wipe_count() > 0 {
            return;
        }
    }
}

/// verifies: REQ-WIP-003 — `LOW` wipes ≈45/min end to end through the whole
/// ECU stack (CAN decode, scheduler, `IoHwAb` PWM, plant).
#[test]
fn low_profile_reaches_45cpm_through_the_whole_ecu() {
    let mut ecu = Ecu::new();
    ignition_on_low(&mut ecu);
    let cpm = wipe_rate_cpm(&mut ecu, 2_000, 20_000);
    assert!(
        (38..=50).contains(&cpm),
        "LOW wipes/min {cpm} outside 40.5–49.5 ±measurement tolerance"
    );
}

/// verifies: REQ-WIP-004 — `HIGH` wipes ≈65/min end to end.
#[test]
fn high_profile_reaches_65cpm_through_the_whole_ecu() {
    let mut ecu = Ecu::new();
    ecu.set_ignition(Ignition::On);
    ecu.sensors_mut().stalk.mode = StalkMode::High;
    let cpm = wipe_rate_cpm(&mut ecu, 2_000, 20_000);
    assert!(
        (55..=75).contains(&cpm),
        "HIGH wipes/min {cpm} outside 58.5–71.5 ±measurement tolerance"
    );
}

/// verifies: REQ-WIP-015 — stalk frame loss on the bus (> 1 s without a fresh
/// alternating counter) degrades the function to visible LOW wiping and
/// raises the `InputTimeout` DTC.
#[test]
fn stalk_bus_loss_degrades_to_low_with_dtc() {
    let mut ecu = Ecu::new();
    ignition_on_low(&mut ecu);
    run(&mut ecu, 500);
    assert!(ecu.read_stalk().value.is_some_and(|s| s.valid));
    let sweeps_before = ecu.swc().wipe_count();

    // BodyCtrl goes offline: no more frame 672.
    ecu.sensors_mut().stalk_period_ms = 0;
    run(&mut ecu, STALK_RX_TIMEOUT_MS + 500);

    assert!(
        ecu.read_stalk().value.is_some_and(|s| !s.valid),
        "COM must publish the invalid fallback push"
    );
    assert!(ecu.dem().is_active(Dtc::InputTimeout));
    assert!(ecu.dem().is_latched(Dtc::InputTimeout));
    assert!(
        ecu.swc().wipe_count() > sweeps_before,
        "fallback keeps wiping (visible LOW wiping)"
    );
    assert_eq!(ecu.det().violations(), 1, "exactly one alive-window trip");
    // Degraded profile is LOW (70 % duty), never HIGH.
    assert_ne!(ecu.last_outputs().motor.speed, MotorSpeed::High);
}

/// verifies: REQ-WIP-015 — rain sensor loss falls back to the manual INT
/// timing of the active sensitivity level (level 2 = 5 s pause).
#[test]
fn rain_bus_loss_falls_back_to_manual_int_timing() {
    let mut ecu = Ecu::new();
    ecu.set_ignition(Ignition::On);
    let sensors = ecu.sensors_mut();
    sensors.stalk.mode = StalkMode::Int;
    sensors.stalk.sensitivity = 2;
    sensors.stalk.auto = true;
    sensors.rain.pct = 80;

    // Auto mode: rain 80 % ≥ threshold 40 % → continuous wiping.
    run(&mut ecu, 3_000);
    let mut last_wipe = ecu.swc().wipe_count();
    let mut last_t = ecu.t_ms();
    let mut auto_gaps = Vec::new();
    for _ in 0..600 {
        ecu.tick();
        if ecu.swc().wipe_count() > last_wipe {
            auto_gaps.push(ecu.t_ms() - last_t);
            last_wipe = ecu.swc().wipe_count();
            last_t = ecu.t_ms();
        }
    }
    assert!(
        auto_gaps.last().is_some_and(|g| *g < 2_500),
        "auto mode wipes continuously (≈1.3 s cycle), got {auto_gaps:?}"
    );

    // RainSensor goes offline; RTE receiver timeout makes rain invalid.
    ecu.sensors_mut().rain_period_ms = 0;
    run(&mut ecu, 1_300);
    let rain = ecu.read_rain();
    // The sensor's own validity flag is carried as sent...
    assert!(
        rain.value.is_some_and(|r| r.valid),
        "sensor valid flag persists"
    );
    // ...but the transport supervision (RTE alive timeout) declares staleness.
    assert_eq!(rain.status, autosar_rte::port::ReadStatus::Timeout);

    // Manual timing: sweep end → 5 s pause (S2) → next wipe.
    let mut manual_gaps: Vec<u32> = Vec::new();
    last_wipe = ecu.swc().wipe_count();
    last_t = ecu.t_ms();
    for _ in 0..1_400 {
        ecu.tick();
        if ecu.swc().wipe_count() > last_wipe {
            manual_gaps.push(ecu.t_ms() - last_t);
            last_wipe = ecu.swc().wipe_count();
            last_t = ecu.t_ms();
        }
    }
    assert!(
        manual_gaps.iter().any(|g| (5_500..=7_000).contains(g)),
        "next manual-INT wipe is ≈5 s (S2) + one sweep after the loss, got {manual_gaps:?}"
    );
}

/// verifies: REQ-WIP-009 — wash request over CAN frame 672: pump during hold,
/// exactly 3 post-release wipes, then back to the stalk mode.
#[test]
fn wash_button_over_can_runs_release_sequence() {
    let mut ecu = Ecu::new();
    ignition_on_low(&mut ecu);
    // Convert to INT S1 (2 s pause) by moving the frame content.
    run(&mut ecu, 500);
    ecu.sensors_mut().stalk.mode = StalkMode::Int;
    ecu.sensors_mut().stalk.sensitivity = 1;
    run(&mut ecu, 3_000);

    // Press WASH over CAN.
    let wipes_at_press = ecu.swc().wipe_count();
    ecu.sensors_mut().stalk.wash_btn = true;
    let mut saw_pump = false;
    for _ in 0..80 {
        ecu.tick();
        if ecu.iohwab().pump_on() {
            saw_pump = true;
            break;
        }
    }
    assert!(saw_pump, "pump output must turn on while WASH is held");
    assert_eq!(ecu.swc().wash_phase(), wiper_swc::WashPhase::Hold);

    // Hold ~600 ms, then release.
    run(&mut ecu, 600);
    assert!(
        ecu.iohwab().pump_on(),
        "pump still active at release moment"
    );
    ecu.sensors_mut().stalk.wash_btn = false;
    run(&mut ecu, 100);
    assert!(!ecu.iohwab().pump_on(), "pump off directly after release");

    // Exactly 3 more wipes, then the sequence ends (back to stalk INT cycle).
    let mut released_wipes = 0;
    for _ in 0..(30_000 / 10) {
        ecu.tick();
        if ecu.swc().wipe_count() > wipes_at_press + released_wipes {
            released_wipes = ecu.swc().wipe_count() - wipes_at_press;
            if released_wipes >= 3 {
                break;
            }
        }
    }
    assert_eq!(released_wipes, 3, "exactly 3 wipes after release");
    run(&mut ecu, 200); // wash phase settles Idle on the tick after the wipe
    assert_eq!(ecu.swc().wash_phase(), wiper_swc::WashPhase::Idle);
}

/// verifies: REQ-WIP-012 — blocked linkage produces the stall current, the
/// SWC retries twice after cooldowns and latches on the 3rd failure; the
/// latch holds until an ignition cycle.
#[test]
fn blocked_linkage_retries_then_latches_until_ignition_cycle() {
    let mut ecu = Ecu::new();
    ignition_on_low(&mut ecu);
    wait_for_first_wipe(&mut ecu);
    assert!(ecu.swc().wipe_count() > 0, "wiping before the fault");

    ecu.plant_mut().set_blocked(true);
    let mut retries_seen = 0;
    let mut latched_at_ms = 0;
    for _ in 0..(20_000 / 10) {
        ecu.tick();
        if ecu.swc().stall_latched() {
            latched_at_ms = ecu.t_ms();
            break;
        }
        if ecu.dem().is_active(Dtc::MotorStall) && retries_seen == 0 {
            retries_seen += 1;
        }
    }
    assert!(
        ecu.swc().stall_latched(),
        "stall latch after 3 failed retries"
    );
    assert!(retries_seen == 1, "retry window passed with DTC active");
    assert!(latched_at_ms >= 6_000, "multiple cooldowns before latching");
    assert!(
        ecu.last_outputs().motor.speed == MotorSpeed::Off,
        "no drive while latched"
    );

    // Unblock: latch must persist until an ignition cycle.
    ecu.plant_mut().set_blocked(false);
    run(&mut ecu, 2_000);
    assert!(ecu.swc().stall_latched());
    assert!(ecu.dem().is_latched(Dtc::MotorStall));

    ecu.set_ignition(Ignition::Off);
    run(&mut ecu, 200);
    ecu.set_ignition(Ignition::On);
    ecu.sensors_mut().stalk.mode = StalkMode::Low;
    run(&mut ecu, 1_000);
    assert!(
        !ecu.swc().stall_latched(),
        "ignition cycle clears the latch"
    );
    assert!(ecu.swc().wipe_count() > 0, "wiping resumes after recovery");
}

/// verifies: REQ-WIP-014 — battery out of the 9–16 V window inhibits the
/// outputs end-to-end; recovery needs the 1 s debounce.
#[test]
fn undervoltage_inhibits_outputs_and_recovers_after_debounce() {
    let mut ecu = Ecu::new();
    ignition_on_low(&mut ecu);
    wait_for_first_wipe(&mut ecu);
    assert!(ecu.swc().wipe_count() > 0);

    ecu.plant_mut().set_battery_v(8.5); // below the 9 V lower limit
    let mut inhibited_seen = false;
    for _ in 0..110 {
        ecu.tick();
        if ecu.swc().voltage_inhibited() {
            inhibited_seen = true;
            break;
        }
    }
    assert!(inhibited_seen);
    run(&mut ecu, 150); // the DEM mirrors the DTC only at the next 100 ms cycle
    assert!(ecu.dem().is_active(Dtc::Undervoltage));
    let frozen_count = ecu.swc().wipe_count();
    run(&mut ecu, 2_000);
    assert_eq!(
        ecu.swc().wipe_count(),
        frozen_count,
        "FSM frozen while inhibited"
    );
    assert_eq!(ecu.last_outputs().motor.duty_pct, 0);

    // Back in range but below the recovery band → still inhibited.
    ecu.plant_mut().set_battery_v(9.2);
    run(&mut ecu, 1_500);
    assert!(
        ecu.swc().voltage_inhibited(),
        "9.2 V is inside 9–16 but outside 9.5 recovery"
    );
    // Recovery band 9.5 V: 1 s debounce then actuation resumes.
    ecu.plant_mut().set_battery_v(13.8);
    run(&mut ecu, 2_500);
    assert!(!ecu.swc().voltage_inhibited());
    assert!(!ecu.dem().is_active(Dtc::Undervoltage));
}

/// verifies: REQ-WIP-017 — the 100 ms task publishes the status frame 675 to
/// the cluster display; signals match the ECU's own state.
#[test]
fn status_frame_mirrors_diagnostic_state() {
    let mut ecu = Ecu::new();
    ignition_on_low(&mut ecu);
    run(&mut ecu, 1_000);
    let frame = ecu.status_frame();
    assert!(frame.is_some(), "status frame transmitted each 100 ms");
    let signals = decode_status(&frame.unwrap_or(bsw_sim::com::CanFrame {
        id: STATUS_FRAME_ID,
        dlc: 3,
        data: [0; 8],
    }));
    assert!(signals.is_some());
    let signals = signals.unwrap_or(bsw_sim::com::StatusSignals {
        wiper_active: false,
        speed_mode: MotorSpeed::Off,
        pump_on: false,
        fsm: WiperFsm::ReturningToPark,
        stall_fault: false,
        int_pause: false,
    });
    assert_eq!(signals.fsm, ecu.swc().fsm_state());
    assert_eq!(signals.speed_mode, ecu.last_outputs().motor.speed);
    assert!(signals.wiper_active);
    assert!(!signals.stall_fault);

    // After a stall block the status frame carries the fault bit.
    ecu.plant_mut().set_blocked(true);
    run(&mut ecu, 1_500);
    let signals = decode_status(&ecu.status_frame().unwrap_or(bsw_sim::com::CanFrame {
        id: STATUS_FRAME_ID,
        dlc: 3,
        data: [0; 8],
    }));
    assert!(signals.is_some());
    let signals = signals.unwrap_or(bsw_sim::com::StatusSignals {
        wiper_active: false,
        speed_mode: MotorSpeed::Off,
        pump_on: false,
        fsm: WiperFsm::ReturningToPark,
        stall_fault: false,
        int_pause: false,
    });
    assert!(signals.stall_fault);
}

/// verifies: REQ-WIP-019 — identical input histories produce identical
/// output/diagnostic histories on two independent ECU instances.
#[test]
fn two_ecus_replay_identically() {
    let script = |ecu: &mut Ecu| {
        ecu.set_ignition(Ignition::On);
        for step in 0..4_000u32 {
            if step % 400 == 200 {
                ecu.sensors_mut().stalk.mode = match step % 2_000 / 400 {
                    0 => StalkMode::Int,
                    1 => StalkMode::Low,
                    2 => StalkMode::High,
                    _ => StalkMode::Off,
                };
                ecu.sensors_mut().stalk.sensitivity =
                    u8::try_from((step / 400) % 4 + 1).unwrap_or(4);
                ecu.sensors_mut().stalk.auto = step % 800 == 400;
                let rain = u8::try_from((step / 700) * 23).unwrap_or(100) % 101;
                ecu.sensors_mut().rain.pct = rain;
                ecu.sensors_mut().speed.kph = u16::try_from((step / 500) * 17 % 200).unwrap_or(199);
            }
            ecu.tick();
        }
    };
    let run_scripted = || {
        let mut ecu = Ecu::new();
        script(&mut ecu);
        (
            ecu.swc().wipe_count(),
            ecu.last_diag().dtc_latched_bits,
            ecu.dem().events().len(),
            ecu.nvm().read().copied(),
        )
    };
    let a = run_scripted();
    let b = run_scripted();
    assert_eq!(a, b, "two ECUs must be tick-for-tick identical");
}

/// The diagnostic latch and wipe count are mirrored into the `NvM` RAM block
/// every 100 ms (interface contract of the simulated `NvM`).
#[test]
fn nvm_mirror_tracks_latched_diags_and_telemetry() {
    let mut ecu = Ecu::new();
    ignition_on_low(&mut ecu);
    run(&mut ecu, 2_000);
    let image = ecu.nvm().read().copied();
    assert!(image.is_some(), "diag task wrote the mirror");
    assert!(ecu.nvm().jobs_done() > 0);
    let image = image.unwrap_or(bsw_sim::nvm::NvmImage::default());
    assert_eq!(image.wipe_count, ecu.swc().wipe_count());
    assert_eq!(image.dtc_latched_bits, ecu.last_diag().dtc_latched_bits);
}
