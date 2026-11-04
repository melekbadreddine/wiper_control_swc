//! Scenario execution engine.
//!
//! Applies scenario setup + timed events onto [`bsw_sim::Ecu`], drives the
//! closed loop tick-by-tick in virtual time (one tick = one 10 ms OS tick),
//! records one [`output::SnapshotRow`] per tick plus wipe-completion
//! timestamps, and evaluates the pass/fail checks of the scenario.
//!
//! Determinism: identical scenario input ⇒ identical rows, byte for byte
//! (all time is the scheduler's virtual time, no wall clock involved).

use bsw_sim::ecu::{Ecu, RainTx, SpeedTx, StalkTx};
use wiper_swc::{Dtc, Ignition, StalkMode, WiperFsm};

use crate::output::SnapshotRow;
use crate::scenario::{
    Check, DtcSpec, Event, FaultSpec, FsmSpec, IgnitionSpec, PartialRain, PartialSpeed,
    PartialStalk, Scenario, Setup, StalkModeSpec,
};

/// Park-window tolerance for the [`Check::ArmInPark`] check (deg, plant
/// quantizes to whole degrees; arm settled at park reads 0..=1).
const PARK_WINDOW_DEG: u16 = 2;

/// One recorded run.
#[derive(Debug)]
pub struct RunResult {
    /// Scenario source of this run.
    pub scenario_name: String,
    /// One row per executed 10 ms tick, in tick order.
    pub rows: Vec<SnapshotRow>,
    /// Virtual times (ms) of completed wipes (wipe-count rising edges).
    pub wipe_times_ms: Vec<u32>,
}

impl RunResult {
    /// Completed wipes over the whole run.
    #[must_use]
    pub fn wipe_total(&self) -> u32 {
        self.rows.last().map_or(0, |r| r.wipe_count)
    }

    /// Completed wipes per minute over the whole run duration.
    #[must_use]
    pub fn wipe_cpm(&self) -> f64 {
        let Some(last) = self.rows.last() else {
            return 0.0;
        };
        if last.t_ms == 0 {
            return 0.0;
        }
        f64::from(last.wipe_count) * 60_000.0 / f64::from(last.t_ms)
    }
}

/// Evaluation of one scenario check.
#[derive(Debug)]
pub struct CheckOutcome {
    /// Stable identifier of the check instance (e.g. `wipe_rate:38.5..54.5`).
    pub id: String,
    /// Check passed.
    pub passed: bool,
    /// Human explanation with the measured value (for the report).
    pub detail: String,
}

/// Run a scenario from time zero and evaluate its checks.
#[must_use]
pub fn run(scenario: &Scenario) -> RunResult {
    let mut ecu = Ecu::new();
    apply_setup(&mut ecu, &scenario.setup);

    // Events are applied before the first tick whose time is >= at_ms.
    // Scenario files are expected sorted; sort defensively by `at_ms`.
    let mut events: Vec<&Event> = scenario.events.iter().collect();
    events.sort_by_key(|e| e.at_ms);
    let mut next_event = 0;

    let mut rows = Vec::new();
    let mut wipe_times_ms = Vec::new();
    let mut prev_wipe_count = ecu.swc().wipe_count();

    // duration_ms is the virtual runtime; the OS reaches it via 10 ms ticks.
    while ecu.t_ms() < scenario.duration_ms {
        while next_event < events.len() {
            let Some(event) = events.get(next_event) else {
                break;
            };
            if event.at_ms <= ecu.t_ms() {
                apply_event(&mut ecu, event);
                next_event += 1;
            } else {
                break;
            }
        }
        ecu.tick();
        let count = ecu.swc().wipe_count();
        if count > prev_wipe_count {
            prev_wipe_count = count;
            wipe_times_ms.push(ecu.t_ms());
        }
        rows.push(SnapshotRow::capture(
            &ecu,
            u64::from(ecu.t_ms()) * 1_000_000,
            u32::try_from(rows.len()).unwrap_or(u32::MAX),
        ));
    }
    RunResult {
        scenario_name: scenario.name.clone(),
        rows,
        wipe_times_ms,
    }
}

/// Evaluate all checks of `scenario` against `result`. No short-circuiting —
/// every check is evaluated so the report shows the full picture.
#[must_use]
pub fn evaluate(scenario: &Scenario, result: &RunResult) -> Vec<CheckOutcome> {
    let ecu_final = final_probe(result);
    scenario
        .checks
        .iter()
        .map(|c| check_outcome(c, result, &ecu_final))
        .collect()
}

/// Endpoint values the checks need (last row + derived flags).
struct FinalProbe {
    fsm: Option<WiperFsm>,
    arm_deg: u16,
    active_bits: u16,
}

fn final_probe(result: &RunResult) -> FinalProbe {
    FinalProbe {
        fsm: result.rows.last().map(|r| r.fsm),
        arm_deg: result.rows.last().map_or(0, |r| r.arm_deg),
        active_bits: result.rows.last().map_or(0, |r| r.dtc_active_bits),
    }
}

fn check_outcome(check: &Check, result: &RunResult, final_: &FinalProbe) -> CheckOutcome {
    match *check {
        Check::FinalFsm { value } => {
            let ok = final_.fsm.is_some_and(|fsm| fsm_matches(fsm, value));
            CheckOutcome {
                id: format!("final_fsm:{value:?}"),
                passed: ok,
                detail: format!(
                    "final FSM state {:?}, expected {value:?}",
                    final_.fsm.map_or("none".to_string(), |f| format!("{f}"))
                ),
            }
        }
        Check::Dtc {
            dtc,
            active,
            latched,
        } => {
            let bit = crate::scenario::dtc_index(dtc);
            let actual_active = final_.active_bits & bit != 0;
            let actual_latched = result
                .rows
                .last()
                .is_some_and(|r| r.dtc_latched_bits & bit != 0);
            let ok = active.is_none_or(|exp| exp == actual_active)
                && latched.is_none_or(|exp| exp == actual_latched);
            CheckOutcome {
                id: format!("dtc:{dtc:?}"),
                passed: ok,
                detail: format!(
                    "{dtc:?} active={actual_active} latched={actual_latched} \
                     (expected active={} latched={})",
                    OptionBool(active),
                    OptionBool(latched)
                ),
            }
        }
        Check::NoActiveDtcs => {
            let ok = final_.active_bits == 0;
            CheckOutcome {
                id: "no_active_dtcs".to_string(),
                passed: ok,
                detail: format!("active DTC bits = 0b{:b}", final_.active_bits),
            }
        }
        Check::WipeRate { min, max } => {
            let cpm = result.wipe_cpm();
            CheckOutcome {
                id: format!("wipe_rate:{min}..{max}"),
                passed: cpm >= min && cpm <= max,
                detail: format!("measured {cpm:.2} cpm over the run, expected ≥ {min} ≤ {max}"),
            }
        }
        Check::WipeCount { min, max } => {
            let total = result.wipe_total();
            let ok = total >= min && max.is_none_or(|m| total <= m);
            CheckOutcome {
                id: "wipe_count".to_string(),
                passed: ok,
                detail: format!(
                    "completed wipes {total}, expected ≥ {min} ≤ {}",
                    OptionMax(max)
                ),
            }
        }
        Check::WipeGapAfter { after_ms, min, max } => {
            // First consecutive pair of completions that both end at/after
            // the mark (see `scenario::Check::WipeGapAfter` semantics).
            let tail = result
                .wipe_times_ms
                .iter()
                .copied()
                .filter(|t| *t >= after_ms)
                .collect::<Vec<u32>>();
            let gap = if tail.len() >= 2 {
                tail.get(1)
                    .map(|second| second.saturating_sub(tail.first().map_or(0, |first| *first)))
            } else {
                None
            };
            let ok = gap.is_some_and(|g| g >= min && g <= max);
            CheckOutcome {
                id: format!("wipe_gap_after:{after_ms}:{min}..{max}"),
                passed: ok,
                detail: match gap {
                    Some(g) => format!("measured gap {g} ms, expected ≥ {min} ≤ {max}"),
                    None => "fewer than two wipe completions at/after the mark".to_string(),
                },
            }
        }
        Check::ArmInPark => {
            let ok = final_.arm_deg <= PARK_WINDOW_DEG;
            CheckOutcome {
                id: "arm_in_park".to_string(),
                passed: ok,
                detail: format!(
                    "final arm angle {} deg (park window ≤ {PARK_WINDOW_DEG})",
                    final_.arm_deg
                ),
            }
        }
    }
}

/// `Display`-helper for optional expected booleans in details.
struct OptionBool(Option<bool>);
impl core::fmt::Display for OptionBool {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self.0 {
            Some(v) => write!(f, "{v}"),
            None => f.write_str("don't-care"),
        }
    }
}

/// `Display`-helper for optional upper bounds.
struct OptionMax(Option<u32>);
impl core::fmt::Display for OptionMax {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self.0 {
            Some(m) => write!(f, "{m}"),
            None => f.write_str("∞"),
        }
    }
}

/// `FsmSpec` against the concrete final FSM state.
fn fsm_matches(fsm: WiperFsm, spec: FsmSpec) -> bool {
    match spec {
        FsmSpec::Parked => fsm == WiperFsm::Parked,
        FsmSpec::IntPause => fsm == WiperFsm::IntPause,
        FsmSpec::Sweeping => fsm == WiperFsm::Sweeping,
        FsmSpec::ReturningToPark => fsm == WiperFsm::ReturningToPark,
        FsmSpec::NotSweeping => fsm != WiperFsm::Sweeping,
    }
}

fn apply_setup(ecu: &mut Ecu, setup: &Setup) {
    if let Some(ign) = setup.ignition {
        ecu.set_ignition(ignition_of(ign));
    }
    let sensors = ecu.sensors_mut();
    if let Some(stalk) = &setup.stalk {
        sensors.stalk = stalk_tx(sensors.stalk, stalk);
    }
    if let Some(rain) = &setup.rain {
        sensors.rain = rain_tx(sensors.rain, rain);
    }
    if let Some(speed) = &setup.speed {
        sensors.speed = speed_tx(sensors.speed, speed);
    }
}

fn apply_event(ecu: &mut Ecu, event: &Event) {
    let sensors = ecu.sensors_mut();
    if let Some(stalk) = &event.stalk {
        sensors.stalk = stalk_tx(sensors.stalk, stalk);
    }
    if let Some(rain) = &event.rain {
        sensors.rain = rain_tx(sensors.rain, rain);
    }
    if let Some(speed) = &event.speed {
        sensors.speed = speed_tx(sensors.speed, speed);
    }
    if let Some(ign) = event.ignition {
        ecu.set_ignition(ignition_of(ign));
    }
    if let Some(fault) = event.fault {
        apply_fault(ecu, fault);
    }
    if let Some(v) = event.battery_v {
        ecu.plant_mut().set_battery_v(v);
    }
}

fn apply_fault(ecu: &mut Ecu, fault: FaultSpec) {
    match fault {
        FaultSpec::StalkOffline => ecu.sensors_mut().stalk_period_ms = 0,
        FaultSpec::RainOffline => ecu.sensors_mut().rain_period_ms = 0,
        FaultSpec::SpeedOffline => ecu.sensors_mut().speed_period_ms = 0,
        FaultSpec::BlockedLinkage => ecu.plant_mut().set_blocked(true),
        FaultSpec::UnblockLinkage => ecu.plant_mut().set_blocked(false),
        FaultSpec::ParkStuckOn => ecu.plant_mut().set_park_stuck(Some(true)),
        FaultSpec::ParkSwitchNormal => ecu.plant_mut().set_park_stuck(None),
        FaultSpec::ClearBus => ecu.bus_mut().clear(),
    }
}

fn ignition_of(spec: IgnitionSpec) -> Ignition {
    match spec {
        IgnitionSpec::Off => Ignition::Off,
        IgnitionSpec::Acc => Ignition::Acc,
        IgnitionSpec::On => Ignition::On,
    }
}

fn stalk_mode_of(spec: StalkModeSpec) -> StalkMode {
    match spec {
        StalkModeSpec::Off => StalkMode::Off,
        StalkModeSpec::Int => StalkMode::Int,
        StalkModeSpec::Low => StalkMode::Low,
        StalkModeSpec::High => StalkMode::High,
    }
}

/// Map the scenario's DTC name to the SWC diagnostic event.
#[must_use]
pub fn dtc_of(spec: DtcSpec) -> Dtc {
    match spec {
        DtcSpec::MotorStall => Dtc::MotorStall,
        DtcSpec::ParkSwitchImplausible => Dtc::ParkSwitchImplausible,
        DtcSpec::Undervoltage => Dtc::Undervoltage,
        DtcSpec::Overvoltage => Dtc::Overvoltage,
        DtcSpec::InputTimeout => Dtc::InputTimeout,
        DtcSpec::PumpOverrun => Dtc::PumpOverrun,
    }
}

// ---------------------------------------------------------------- helpers

fn stalk_tx(current: StalkTx, p: &PartialStalk) -> StalkTx {
    StalkTx {
        mode: p.mode.map_or(current.mode, stalk_mode_of),
        sensitivity: p.sensitivity.map_or(current.sensitivity, |s| s.clamp(1, 4)),
        auto: p.auto.unwrap_or(current.auto),
        wash_btn: p.wash_btn.unwrap_or(current.wash_btn),
    }
}

fn rain_tx(current: RainTx, p: &PartialRain) -> RainTx {
    RainTx {
        pct: p.pct.unwrap_or(current.pct).min(100),
        valid: p.valid.unwrap_or(current.valid),
    }
}

fn speed_tx(current: SpeedTx, p: &PartialSpeed) -> SpeedTx {
    SpeedTx {
        kph: p.kph.unwrap_or(current.kph).min(250),
        valid: p.valid.unwrap_or(current.valid),
    }
}
