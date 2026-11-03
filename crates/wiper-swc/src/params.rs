//! Parameter tables and pure helper computations for the wiper SWC.
//!
//! All functions here are total, pure, and unit-testable; the FSM in `swc.rs`
//! consumes them. Time is integer milliseconds, speeds in km/h, currents in 0.1 A
//! units, voltages in 0.1 V units (fixed-point by design, see `AGENTS.md`).

use crate::types::Sensitivity;

/// INT base pause between wipes per sensitivity level (ms). Levels 1..=4 index the
/// table (`index-1`); REQ-WIP-005.
pub const INT_BASE_PAUSE_MS: [u32; Sensitivity::MAX as usize] = [2000, 5000, 7000, 11000];

/// Auto-mode rain threshold (percent) per sensitivity level, REQ-WIP-007.
pub const AUTO_RAIN_THRESHOLD_PCT: [u8; Sensitivity::MAX as usize] = [60, 40, 25, 10];

/// Auto-mode hysteresis in percent points (drop below threshold to resume pausing).
pub const AUTO_HYSTERESIS_PCT: u8 = 5;

/// Park threshold: arm angle at/below which the arm is judged parked when angle
/// sensing is used (park switch stuck, REQ-WIP-008/013).
pub const PARK_ANGLE_DEG: u16 = 3;

/// Angle considered clearly out of park (park-switch plausibility monitoring).
pub const OUT_OF_PARK_ANGLE_DEG: u16 = 5;

/// Sweep end of travel (upper turnaround), degrees.
pub const SWEEP_TOP_DEG: u16 = 100;

/// Under-voltage limit in 0.1 V (9.0 V), REQ-WIP-014.
pub const UNDER_VOLTAGE_V0P1: u16 = 90;
/// Over-voltage limit in 0.1 V (16.0 V), REQ-WIP-014.
pub const OVER_VOLTAGE_V0P1: u16 = 160;
/// Recovery lower limit in 0.1 V (9.5 V) with 1 s debounce.
pub const RECOVERY_UNDER_V0P1: u16 = 95;
/// Recovery upper limit in 0.1 V (15.5 V) with 1 s debounce.
pub const RECOVERY_OVER_V0P1: u16 = 155;

/// Stall current threshold in 0.1 A (8.0 A), REQ-WIP-012.
pub const STALL_CURRENT_DA: u16 = 80;

/// "Arm not moving" speed threshold in deg/s (fixed-point: i16 deg/s).
pub const STALL_ARM_SPEED_LIMIT_DPS: i16 = 20;

/// Stall detection accumulation time, ms, REQ-WIP-012.
pub const STALL_DETECT_MS: u32 = 500;

/// Stall retry cool-down, ms, REQ-WIP-012.
pub const STALL_COOLDOWN_MS: u32 = 2000;

/// Maximum retries before latching, REQ-WIP-012.
pub const STALL_MAX_RETRIES: u8 = 3;

/// Wash pump continuous-run cap, ms, REQ-WIP-010.
pub const WASH_PUMP_CAP_MS: u32 = 10_000;

/// Minimum pump off-time between activations, ms, REQ-WIP-016.
pub const WASH_PUMP_OFFGUARD_MS: u32 = 3000;

/// Wipes after wash release, REQ-WIP-009.
pub const WASH_RELEASE_WIPES: u8 = 3;

/// Park-switch plausibility accumulation time, ms, REQ-WIP-013.
pub const PARK_IMPLAUSIBLE_MS: u32 = 200;

/// Speed-dependence: pause factor stays 1000 permille below this speed (REQ-WIP-006).
pub const SPEED_FLOOR_KPH: u16 = 20;
/// Speed-dependence: pause factor stops shrinking at this speed (REQ-WIP-006).
pub const SPEED_CEILING_KPH: u16 = 120;
/// Speed-dependence: factor at/above ceiling in permille (500 = 50 %).
pub const SPEED_CEIL_FACTOR_PERMILLE: u32 = 500;

/// Base pause for sensitivity level (level 0 → default 2 as documented fallback,
/// levels 1..=4 per REQ-WIP-005).
#[must_use]
pub fn int_base_pause_ms(level: u8) -> u32 {
    if level == 0 || level > Sensitivity::MAX {
        INT_BASE_PAUSE_MS[1]
    } else {
        INT_BASE_PAUSE_MS
            .get(usize::from(level - 1))
            .copied()
            .unwrap_or(5000)
    }
}

/// Auto-mode rain threshold for sensitivity level (level 0 → default 2).
#[must_use]
pub fn auto_threshold_pct(level: u8) -> u8 {
    if level == 0 || level > Sensitivity::MAX {
        AUTO_RAIN_THRESHOLD_PCT[1]
    } else {
        AUTO_RAIN_THRESHOLD_PCT
            .get(usize::from(level - 1))
            .copied()
            .unwrap_or(40)
    }
}

/// Pause factor in permille by vehicle speed: 1000 below 20 km/h, shrinking
/// linearly to 500 at 120 km/h and above (REQ-WIP-006).
#[must_use]
pub fn speed_pause_factor_permille(speed_kph: u16) -> u32 {
    if speed_kph <= SPEED_FLOOR_KPH {
        1000
    } else if speed_kph >= SPEED_CEILING_KPH {
        SPEED_CEIL_FACTOR_PERMILLE
    } else {
        let span = u32::from(SPEED_CEILING_KPH - SPEED_FLOOR_KPH);
        let into = u32::from(speed_kph - SPEED_FLOOR_KPH);
        let gamma = into * (1000 - SPEED_CEIL_FACTOR_PERMILLE);
        1000 - (gamma / span)
    }
}

/// Computed INT pause in ms (base × speed factor), REQ-WIP-005/006.
#[must_use]
pub fn int_pause_ms(level: u8, speed_kph: u16) -> u32 {
    (int_base_pause_ms(level) * speed_pause_factor_permille(speed_kph)) / 1000
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn base_pause_table() {
        // verifies: REQ-WIP-005
        assert_eq!(int_base_pause_ms(1), 2000);
        assert_eq!(int_base_pause_ms(2), 5000);
        assert_eq!(int_base_pause_ms(3), 7000);
        assert_eq!(int_base_pause_ms(4), 11000);
        // Documented fallback for invalid sensitivity.
        assert_eq!(int_base_pause_ms(0), 5000);
        assert_eq!(int_base_pause_ms(9), 5000);
    }

    #[test]
    fn auto_threshold_table() {
        // verifies: REQ-WIP-007
        assert_eq!(auto_threshold_pct(1), 60);
        assert_eq!(auto_threshold_pct(2), 40);
        assert_eq!(auto_threshold_pct(3), 25);
        assert_eq!(auto_threshold_pct(4), 10);
    }

    #[test]
    fn speed_pause_factor() {
        // verifies: REQ-WIP-006
        assert_eq!(speed_pause_factor_permille(0), 1000);
        assert_eq!(speed_pause_factor_permille(20), 1000);
        assert_eq!(speed_pause_factor_permille(120), 500);
        assert_eq!(speed_pause_factor_permille(200), 500);
        // Linear: at 80 km/h (60 % of the way) expect ~700 permille.
        assert_eq!(speed_pause_factor_permille(80), 700);
    }

    #[test]
    fn int_pause_monotonic() {
        // More speed ⇒ shorter (or equal) pause; higher sensitivity level ⇒ longer.
        for speed in [0, 20, 40, 60, 80, 100, 120] {
            for lvl in 1..=3u8 {
                assert!(int_pause_ms(lvl + 1, speed) >= int_pause_ms(lvl, speed));
                assert!(int_pause_ms(lvl, speed + 10) <= int_pause_ms(lvl, speed));
            }
        }
        // S2 at 80 km/h: 5000 ms × 0.7 = 3500 ms.
        assert_eq!(int_pause_ms(2, 80), 3500);
    }
}
