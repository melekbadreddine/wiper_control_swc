//! Physical plant models for the wiper system (equations documented in M4).
//!
//! Models: DC motor with gear ratio and crank linkage, winding load (dry-glass
//! friction increase), battery voltage sag, rain intensity, vehicle-speed profile.
//! Integrator: fixed-step Runge-Kutta 4, step size documented in
//! `docs/plant-model.md` (written in milestone M4 — see `PROGRESS.md`).

#![forbid(unsafe_code)]

/// Placeholder used by the M0 bootstrap gate only.
pub const PLANT_SIM_VERSION: &str = env!("CARGO_PKG_VERSION");

#[cfg(test)]
mod tests {
    use super::PLANT_SIM_VERSION;

    #[test]
    fn crate_compiles() {
        assert!(!PLANT_SIM_VERSION.is_empty());
    }
}
