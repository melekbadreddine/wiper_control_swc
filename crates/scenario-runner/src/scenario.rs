//! TOML scenario model (serialized on disk in `scenarios/*.toml`).
//!
//! A scenario is: warm setup, a list of timed events (stalk/rain/speed/ignition
//! changes, faults, battery injections), a duration in virtual time, and pass/
//! fail checks evaluated on the recorded run. All enums use lowercase TOML
//! values; unknown fields are rejected so typos in scenario files fail loudly
//! rather than silently cherry-picking defaults.

use serde::Deserialize;

/// Whole scenario file.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Scenario {
    /// Unique identifier, also used for report file names.
    pub name: String,
    /// Human description (goes into the report).
    pub description: String,
    /// Total virtual runtime (ms; the OS ticks 10 ms at a time).
    pub duration_ms: u32,
    /// Applied once at virtual time 0, before the first tick.
    #[serde(default)]
    pub setup: Setup,
    /// Timed action sets (absolute virtual time).
    #[serde(default)]
    pub events: Vec<Event>,
    /// Some pass/fail checks.
    #[serde(default)]
    pub checks: Vec<Check>,
}

/// Initial sensor/ignition state.
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Setup {
    /// Ignition at power-on (default `off`).
    pub ignition: Option<IgnitionSpec>,
    /// `BodyCtrl` node content.
    pub stalk: Option<StalkSpec>,
    /// `RainSensor` node content.
    pub rain: Option<RainSpec>,
    /// `AbsEcu` node content.
    pub speed: Option<SpeedSpec>,
}

/// One timed event; every present section is applied at `at_ms`.
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Event {
    /// Absolute virtual time (ms).
    pub at_ms: u32,
    /// New `BodyCtrl` content (partial: fields given are set, others keep).
    pub stalk: Option<PartialStalk>,
    /// New `RainSensor` content (partial).
    pub rain: Option<PartialRain>,
    /// New `AbsEcu` content (partial).
    pub speed: Option<PartialSpeed>,
    /// New ignition state.
    pub ignition: Option<IgnitionSpec>,
    /// A single physical/linkage/battery fault injection.
    pub fault: Option<FaultSpec>,
    /// Battery voltage (V) plant injection (leave absent to keep).
    pub battery_v: Option<f64>,
}

/// Stalk content as written by the scenario.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PartialStalk {
    /// Stalk position.
    pub mode: Option<StalkModeSpec>,
    /// Sensitivity level 1..=4.
    pub sensitivity: Option<u8>,
    /// Auto (rain sensor) flag.
    pub auto: Option<bool>,
    /// Momentary wash button state.
    pub wash_btn: Option<bool>,
}

/// Rain content as written by the scenario.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PartialRain {
    /// Rain intensity 0..=100 %.
    pub pct: Option<u8>,
    /// Validity flag.
    pub valid: Option<bool>,
}

/// Speed content as written by the scenario.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PartialSpeed {
    /// Vehicle speed km/h.
    pub kph: Option<u16>,
    /// Validity flag.
    pub valid: Option<bool>,
}

/// Ignition positions (`ignition = "on"`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum IgnitionSpec {
    /// Ignition off.
    Off,
    /// Accessory.
    Acc,
    /// Running.
    On,
}

/// Stalk positions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum StalkModeSpec {
    /// OFF.
    Off,
    /// INT (intermittent/auto).
    Int,
    /// LOW (~45 wipes/min).
    Low,
    /// HIGH (~65 wipes/min).
    High,
}

/// Setup-section node content shares the partial-event format: fields left
/// absent keep the `SensorNodes::new()` defaults (OFF/S1, dry, stopped car).
pub type StalkSpec = PartialStalk;
/// Rain section of [`Setup`].
pub type RainSpec = PartialRain;
/// Speed section of [`Setup`].
pub type SpeedSpec = PartialSpeed;

/// Fault injections (`fault = "stalk_offline"`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FaultSpec {
    /// Stalk node stops transmitting.
    StalkOffline,
    /// Rain node stops transmitting.
    RainOffline,
    /// Speed node stops transmitting.
    SpeedOffline,
    /// Linkage blocked (stall injection).
    BlockedLinkage,
    /// Unblock the linkage.
    UnblockLinkage,
    /// Park switch stuck active.
    ParkStuckOn,
    /// Park switch back to normal operation.
    ParkSwitchNormal,
    /// Wipe every frame off the bus (total network outage).
    ClearBus,
}

/// DTC names in check blocks (mirrors `wiper_swc::Dtc` order).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DtcSpec {
    /// Stalled motor.
    MotorStall,
    /// Park switch stuck.
    ParkSwitchImplausible,
    /// Battery below 9 V.
    Undervoltage,
    /// Battery above 16 V.
    Overvoltage,
    /// Stalk/inputs lost.
    InputTimeout,
    /// Pump over-run protection.
    PumpOverrun,
}

/// DTC name → bit index in the `DiagStatus` bitset.
#[must_use]
pub fn dtc_index(spec: DtcSpec) -> u16 {
    let idx = match spec {
        DtcSpec::MotorStall => 0,
        DtcSpec::ParkSwitchImplausible => 1,
        DtcSpec::Undervoltage => 2,
        DtcSpec::Overvoltage => 3,
        DtcSpec::InputTimeout => 4,
        DtcSpec::PumpOverrun => 5,
    };
    1u16 << idx
}

/// One pass/fail check (internally tagged: `kind = "wipe_rate"` etc.).
///
/// Note: internally tagged enums cannot use `deny_unknown_fields` (serde);
/// typo'd check fields therefore fail as "unknown/missing field" errors from
/// the toml parser, which is loud enough.
#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Check {
    /// Final FSM state (`value = "parked"` etc.).
    FinalFsm {
        /// Expected final FSM state (`value = "parked"` etc.).
        value: FsmSpec,
    },
    /// DTC active/latched expectation at the end of the run.
    Dtc {
        /// Which DTC.
        dtc: DtcSpec,
        /// Expected final active state (unset = don't care).
        active: Option<bool>,
        /// Expected final latched state (unset = don't care).
        latched: Option<bool>,
    },
    /// No DTC is active at the end of the run.
    NoActiveDtcs,
    /// Completed-wipe rate bounds in cpm over the whole run.
    WipeRate {
        /// Lower bound cpm.
        min: f64,
        /// Upper bound cpm.
        max: f64,
    },
    /// Total completed wipes at the end of the run.
    WipeCount {
        /// Minimum.
        min: u32,
        /// Optional maximum.
        max: Option<u32>,
    },
    /// Gap between the first two completed wipes that both end at ≥
    /// `after_ms` (the anchor completion may have started before `after_ms` —
    /// e.g. the last auto wipe whose pause is measured after a rain drop at
    /// `after_ms`).
    WipeGapAfter {
        /// Measure from this virtual time (ms).
        after_ms: u32,
        /// Minimum gap (ms).
        min: u32,
        /// Maximum gap (ms).
        max: u32,
    },
    /// The arm rests in the park window at the end of the run (REQ-WIP-008).
    ArmInPark,
}

/// FSM state names in check blocks.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FsmSpec {
    /// Parked.
    Parked,
    /// `IntPause`.
    IntPause,
    /// Sweeping.
    Sweeping,
    /// `ReturningToPark`.
    ReturningToPark,
    /// Any non-Sweeping state (park/INT pause/returning).
    NotSweeping,
}
