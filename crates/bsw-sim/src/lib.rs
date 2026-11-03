//! Simulated Basic Software (host-side stand-in for CAN stack, drivers, DEM, OS).
//!
//! Provides, in later milestones (see `PROGRESS.md`):
//! - an OSEK-like cyclic scheduler with deterministic virtual time (10 ms / 100 ms tasks),
//! - COM layer mimicking CAN signal packing with the committed `.dbc` mapping,
//! - `IoHwAb` ADC/PWM/DIO abstractions,
//! - `Dem`/`Det` for DTC storage and development-error tracking, and an `NvM` stub.
//!
//! Implemented in milestone M4.

#![forbid(unsafe_code)]

/// Placeholder used by the M0 bootstrap gate only.
pub const BSW_SIM_VERSION: &str = env!("CARGO_PKG_VERSION");

#[cfg(test)]
mod tests {
    use super::BSW_SIM_VERSION;

    #[test]
    fn crate_compiles() {
        assert!(!BSW_SIM_VERSION.is_empty());
    }
}
