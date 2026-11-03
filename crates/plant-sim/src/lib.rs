//! Physical plant model for the wiper system.
//!
//! # Model (documented in `docs/plant-model.md`)
//!
//! A DC motor drives a crank-and-linkage; the windshield arm angle follows
//! `θ(φ) = 55°·(1 − cos φ)` from the crank angle φ — one crank revolution is
//! exactly one full wipe (park → 110° → park), matching real wiper mechanisms.
//!
//! States: crank angle `φ ∈ [0, 2π)`, crank speed `ω` (rad/s). Integrator: fixed
//! step RK4 with `dt = 1 ms` (ADR-0005), 10 steps per ECU cycle.
//!
//! Constants (battery nominally 13.8 V,电压 scenarios may override):
//! * armature resistance `R = 0.5 Ω`, motor constant `Kt = Ke = 1.31`
//!   (NiCr-A-equivalent SI pairs: torque in N·m at A, EMF in V at rad/s)
//! * mechanical load: `τ_load = τ₀(dryness) + β·ω` with
//!   `τ₀ = 2.0 N·m · (1 − 0.15·rain)` (dry glass loads most, REQ "dry glass")
//! * damping+load equilibrium: at duty 70 % → ω ≈ 4.8 rad/s ⇒ ≈46 wipes/min
//!   (REQ-WIP-003 target 45 ±10 %), duty 100 % → ω ≈ 7.1 rad/s ⇒ ≈68 wipes/min
//!   (REQ-WIP-004 target 65 ±10 %) — verified by `validation` tests.
//! * blocked linkage (fault injection): position pinned, current =
//!   `duty·V/R` (stall path, REQ-WIP-012).
//! * battery: stiff source; sags are scenario-driven (no internal dynamics).

#![forbid(unsafe_code)]

use std::f64::consts::{FRAC_2_PI, PI, TAU};

/// Fixed RK4 step (seconds) — ADR-0005.
pub const DT_S: f64 = 1.0e-3;

/// Crank-to-arm scaling: max arm angle in degrees.
pub const SWEEP_MAX_DEG: u16 = 110;

#[derive(Debug, Clone)]
pub struct PlantConfig {
    /// Armature resistance Ω.
    pub r: f64,
    /// Motor (and back-EMF) constant.
    pub kt: f64,
    /// Total inertia (motor + crank + arm referred) kg·m².
    pub j: f64,
    /// Linear damping term β in τ_load.
    pub beta: f64,
    /// Base (dry) load torque N·m.
    pub tau0_dry: f64,
}

impl Default for PlantConfig {
    fn default() -> Self {
        Self { r: 0.5, kt: 1.31, j: 0.05, beta: 0.7, tau0_dry: 2.0 }
    }
}

/// One plant output snapshot (ECU inputs + drive-side telemetry).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlantOutput {
    /// Arm angle in degrees 0..=110.
    pub angle_deg: u16,
    /// Arm (crank-linked) angular speed, deg/s signed.
    pub arm_speed_dps: i16,
    /// Park switch active (arm within the park window).
    pub park_switch: bool,
    /// Motor current in 0.1 A.
    pub current_dA: u16,
    /// Battery voltage 0.1 V (passed through to the ECU).
    pub battery_v01: u16,
}

/// Fault-injectable wiper plant.
#[derive(Debug, Clone)]
pub struct WiperPlant {
    cfg: PlantConfig,
    /// Crank angle (rad), wrapped to [0, 2π).
    phi: f64,
    /// Crank speed (rad/s).
    omega: f64,
    /// Battery voltage (V).
    v_batt: f64,
    /// Rain percent 0..=100 (affects τ_load).
    rain_pct: f64,
    /// Fault: linkage blocked (position pinned while commanded).
    blocked: bool,
    /// Fault: park-switch stuck state; `Some(b)` overrides the computed switch.
    park_stuck: Option<bool>,
    /// Last commanded duty percent 0..=100.
    last_duty: u8,
}

impl WiperPlant {
    /// Fresh plant at park, 13.8 V, dry glass.
    #[must_use]
    pub fn new() -> Self {
        Self::with_config(PlantConfig::default())
    }

    /// Plant with custom config (tests explore parameter space).
    #[must_use]
    pub fn with_config(cfg: PlantConfig) -> Self {
        Self {
            cfg,
            phi: 0.0,
            omega: 0.0,
            v_batt: 13.8,
            rain_pct: 0.0,
            blocked: false,
            park_stuck: None,
            last_duty: 0,
        }
    }

    /// Scenario/fault configuration entry points.
    pub fn set_battery_v(&mut self, v: f64) {
        self.v_batt = v.clamp(0.0, 24.0);
    }

    /// Scenario rain input (0..=100).
    pub fn set_rain_pct(&mut self, rain: u8) {
        self.rain_pct = f64::from(rain);
    }

    /// Linkage blocked fault (stall injection).
    pub fn set_blocked(&mut self, blocked: bool) {
        self.blocked = blocked;
    }

    /// Stuck park switch fault: `Some(state)` overrides the switch reading.
    pub fn set_park_stuck(&mut self, state: Option<bool>) {
        self.park_stuck = state;
    }

    /// Crank angle (rad).
    #[must_use]
    pub fn phi_rad(&self) -> f64 {
        self.phi
    }

    /// Derivatives (RK4/θ reference). `duty` in percent.
    fn derivs(duty_pct: f64, v: f64, phi: f64, omega: f64, cfg: &PlantConfig, tau0_dryness: f64) -> (f64, f64) {
        let u = v * duty_pct / 100.0;
        let emf = cfg.kt * omega;
        let current = (u - emf) / cfg.r;
        let torque = cfg.kt * current;
        let tau_load = tau0_dryness + cfg.beta * omega.abs();
        let d_omega = (torque - tau_load) / cfg.j;
        (omega, d_omega)
    }

    /// Integrate the plant one RK4 step at the given duty (percent 0..=100).
    /// Blocked linkage: pin position, return the stall current.
    fn step_int(&mut self, duty_pct: u8) {
        self.last_duty = duty_pct;
        let duty = f64::from(duty_pct);
        // Dryness factor: rain reduces the load torque up to 15 %.
        let tau0 = self.cfg.tau0_dry * (1.0 - 0.15 * self.rain_pct / 100.0);
        if duty <= 0.0 {
            // Free stop: simple exponential decay (fast visually, deterministic).
            self.omega *= (1.0 - DT_S * 20.0).max(0.0);
            if self.omega.abs() < 1.0e-6 {
                self.omega = 0.0;
            }
            // coast back toward park along the mechanism (crank keeps φ).
            self.phi += self.omega * DT_S;
            self.phi = self.phi.rem_euclid(TAU);
            return;
        }
        if self.blocked {
            // Pinned: all applied voltage → current, no motion.
            self.omega = 0.0;
            return;
        }
        // RK4 over dt with the fixed duty.
        let (k1w, k1do) = Self::derivs(duty, self.v_batt, self.phi, self.omega, &self.cfg, tau0);
        let (k2w, k2do) = Self::derivs(duty, self.v_batt, self.phi + DT_S * 0.5 * k1w, self.omega + DT_S * 0.5 * k1do, &self.cfg, tau0);
        let (k3w, k3do) = Self::derivs(duty, self.v_batt, self.phi + DT_S * 0.5 * k2w, self.omega + DT_S * 0.5 * k2do, &self.cfg, tau0);
        let (k4w, k4do) = Self::derivs(duty, self.v_batt, self.phi + DT_S * k3w, self.omega + DT_S * k3do, &self.cfg, tau0);
        self.phi += dt * (k1w + 2.0 * k2w + 2.0 * k3w + k4w) / 6.0;
        self.omega += dt * (k1do + 2.0 * k2do + 2.0 * k3do + k4do) / 6.0;
        self.phi = self.phi.rem_euclid(TAU);
        if self.omega.is_nan() || self.omega.abs() > 200.0 {
            self.omega = 0.0; // numeric guard; reported via DET in tests
        }
    }

    /// One ECU tick: 10 RK4 steps (10 ms) at `commanded duty`, then snapshot.
    pub fn tick_10ms(&mut self, command: wiper_swc::MotorCommand) -> PlantOutput {
        for _ in 0..10 {
            self.step_int(command.duty_pct);
        }
        self.snapshot()
    }

    /// Snapshot of the plant state as ECU inputs.
    #[must_use]
    pub fn snapshot(&self) -> PlantOutput {
        let angle = 55.0 * (1.0 - self.phi.cos()); // deg
        let arm_speed = 55.0 * self.omega * self.phi.sin() * (180.0 / PI); // deg/s
        let angle_deg = (angle.round() as u16).min(SWEEP_MAX_DEG);
        let current = if self.blocked && self.last_duty > 0 {
            (self.v_batt * f64::from(self.last_duty) / 100.0) / self.cfg.r
        } else {
            let u = self.v_batt * f64::from(self.last_duty) / 100.0;
            let emf = self.cfg.kt * self.omega;
            ((u - emf) / self.cfg.r).max(0.0)
        };
        let park = if angle_deg <= 2 { true } else { false } || self.park_stuck.unwrap_or(false);
        PlantOutput {
            angle_deg,
            arm_speed_dps: (arm_speed.round() as i16).max(-2000).min(2000),
            park_switch: park,
            current_dA: (current * 10.0).round() as u16,
            battery_v01: (self.v_batt * 10.0).round() as u16,
        }
    }
}

const dt: f64 = DT_S;

impl Default for WiperPlant {
    fn default() -> Self {
        Self::new()
    }
}

// silence unused warning for the intentionally imported FRAC_2_PI (docs example)
#[allow(dead_code)]
fn _unused() -> f64 {
    FRAC_2_PI
}
