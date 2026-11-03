//! Data types and AUTOSAR-style port interfaces of the wiper SWC.
//!
//! Fixed-point representations everywhere in this crate: rain/voltage/current as
//! scaled integers — deterministic, allocation-free, no floating point (see
//! `AGENTS.md`). Nothing here depends on ROS, CAN, or the host OS.

// AUTOSAR data-element naming keeps mixed-case unit suffixes (`dA`, `v0p1`),
// so the standard non_snake_case rule is relaxed at crate level.
#![allow(non_snake_case)]

use core::fmt;

/// Stalk position (REQ-WIP-002).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum StalkMode {
    /// Off; arm returns to park (REQ-WIP-008).
    #[default]
    Off,
    /// Intermittent with sensitivity level + auto flag (REQ-WIP-005/006/007).
    Int,
    /// Low speed (~45 wipes/min, REQ-WIP-003).
    Low,
    /// High speed (~65 wipes/min, REQ-WIP-004).
    High,
}

/// Stalk sensitivity levels 1..=4 (0 = invalid, REQ-WIP-015).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Sensitivity(pub u8);

impl Sensitivity {
    /// Minimum valid level.
    pub const MIN: u8 = 1;
    /// Maximum valid level.
    pub const MAX: u8 = 4;

    /// Valid levels are 1..=4.
    #[must_use]
    pub fn is_valid(self) -> bool {
        (Self::MIN..=Self::MAX).contains(&self.0)
    }
}

/// Ignition state (REQ-WIP-001).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Ignition {
    /// Ignition off (no wiper, no washer).
    #[default]
    Off,
    /// Accessory (washer allowed, wiper not, REQ-WIP-011).
    Acc,
    /// Full power (wiper allowed, REQ-WIP-001).
    On,
}

/// Motor speed profile (REQ-WIP-018 maps it to PWM duty).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum MotorSpeed {
    /// Off.
    #[default]
    Off,
    /// Low profile (70 % duty).
    Low,
    /// High profile (100 % duty).
    High,
}

/// Motor command element (off / low / high + PWM duty, REQ-WIP-018).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotorCommand {
    /// Speed profile.
    pub speed: MotorSpeed,
    /// PWM duty in percent 0..=100.
    pub duty_pct: u8,
}

impl Default for MotorCommand {
    fn default() -> Self {
        Self {
            speed: MotorSpeed::Off,
            duty_pct: 0,
        }
    }
}

/// FSM state of the SWC (State Transitions panel / diagnostics).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum WiperFsm {
    /// At park, waiting.
    #[default]
    Parked,
    /// Intermittent pause between wipes.
    IntPause,
    /// Actively sweeping (out or back).
    Sweeping,
    /// Driving to park after OFF / fault.
    ReturningToPark,
}

impl fmt::Display for WiperFsm {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Parked => "Parked",
            Self::IntPause => "IntPause",
            Self::Sweeping => "Sweeping",
            Self::ReturningToPark => "ReturningToPark",
        })
    }
}

/// Wash request lifecycle phases (REQ-WIP-009/010).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum WashPhase {
    /// No wash activity.
    #[default]
    Idle,
    /// Button held: pump (capped) + continuous wiping.
    Hold,
    /// Button released: 3 more wipes, then resume stalk mode.
    AfterRelease {
        /// Wipes left until the sequence ends.
        wipes_left: u8,
    },
}

impl fmt::Display for WashPhase {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Idle => "Idle",
            Self::Hold => "Hold",
            Self::AfterRelease { wipes_left } => return write!(f, "AfterRelease({wipes_left})"),
        })
    }
}

/// DTC identifiers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dtc {
    /// Stalled motor / blocked linkage.
    MotorStall,
    /// Park switch stuck (arm out of park while switch active).
    ParkSwitchImplausible,
    /// Battery voltage below 9 V.
    Undervoltage,
    /// Battery voltage above 16 V.
    Overvoltage,
    /// Stalk/inputs lost — degraded mode active.
    InputTimeout,
    /// Washer pump over-run protection active.
    PumpOverrun,
}

impl fmt::Display for Dtc {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::MotorStall => "MotorStall",
            Self::ParkSwitchImplausible => "ParkSwitchImplausible",
            Self::Undervoltage => "Undervoltage",
            Self::Overvoltage => "Overvoltage",
            Self::InputTimeout => "InputTimeout",
            Self::PumpOverrun => "PumpOverrun",
        })
    }
}

impl Dtc {
    /// All DTCs in bit order used by `DiagStatus`.
    pub const ALL: [Self; 6] = [
        Self::MotorStall,
        Self::ParkSwitchImplausible,
        Self::Undervoltage,
        Self::Overvoltage,
        Self::InputTimeout,
        Self::PumpOverrun,
    ];
}

/// Diagnostic status snapshot published by `Wiper_Run100ms` (REQ-WIP-017).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DiagStatus {
    /// Bit set per currently active DTC (bit = index in `Dtc::ALL`).
    pub dtc_active_bits: u16,
    /// Bit set per DTC that has ever fired (history; not auto-cleared).
    pub dtc_latched_bits: u16,
    /// Aggregate inhibit indicator.
    pub fault_active: bool,
}

/// Per-cycle inputs from the plant/CAN (the SWC reads them all at the 10 ms tick;
/// in a real ECU each element is its own RTE port).
#[derive(Debug, Clone, Copy)]
pub struct Inputs {
    /// Ignition state (AUTOSAR mode switch port).
    pub ignition: Ignition,
    /// Rain intensity percent 0..=100.
    pub rain_pct: u8,
    /// Rain signal valid? (false = sensor loss → manual INT timing, REQ-WIP-015)
    pub rain_valid: bool,
    /// Vehicle speed km/h.
    pub speed_kph: u16,
    /// Speed signal valid? (false → treated as 0 km/h = maximal pause).
    pub speed_valid: bool,
    /// Park switch active (arm at rest).
    pub park_switch: bool,
    /// Motor current in 0.1 A units (84 = 8.4 A).
    pub motor_current_dA: u16,
    /// Battery voltage in 0.1 V units (138 = 13.8 V).
    pub battery_v0p1: u16,
    /// Arm angle 0..=110 degrees.
    pub arm_angle_deg: u16,
    /// Arm angular velocity deg/s (signed; near 0 while stalled).
    pub arm_speed_dps: i16,
}

impl Default for Inputs {
    fn default() -> Self {
        Self {
            ignition: Ignition::On,
            rain_pct: 0,
            rain_valid: true,
            speed_kph: 0,
            speed_valid: true,
            park_switch: true,
            motor_current_dA: 0,
            battery_v0p1: 135,
            arm_angle_deg: 0,
            arm_speed_dps: 0,
        }
    }
}

/// Per-cycle outputs to the actuators.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Outputs {
    /// Motor command.
    pub motor: MotorCommand,
    /// Washer pump on.
    pub pump_on: bool,
}
