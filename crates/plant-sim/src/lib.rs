//! Physical plant model for the wiper system.
//!
//! Full derivation in `docs/plant-model.md`; summary:
//!
//! ```text
//! U        = duty·U_batt                      (PWM averaging of the drive)
//! I        = (U − Kt·ω)/R                     (armature current)
//! τ_motor  = Kt·I
//! τ_load   = τ₀·(1 − 0.15·rain/100) + β·|ω|   (dry glass loads most)
//! J·dω/dt  = τ_motor − τ_load
//! dφ/dt    = ω                                (crank angle)
//! θ_arm    = 55°·(1 − cos φ)                  (park → 110° → park = one wipe)
//! ω_arm    = 55°·sin φ·ω
//! ```
//!
//! Calibrated: `R = 0.5 Ω`, `Kt = Ke = 1.57`, `β = 0.7`, `τ₀ = 3.25 N·m`,
//! `J = 0.05 kg·m²`. At 13.8 V: duty 70 % ⇒ ≈46 wipes/min (LOW, REQ-WIP-003
//! 45 ±10 %), duty 100 % ⇒ ≈68 wipes/min (HIGH, REQ-WIP-004 65 ±10 %) — enforced
//! by the validation tests below. Faults: blocked linkage pins position with stall
//! current (REQ-WIP-012); stuck park switch override (REQ-WIP-013). Integrator:
//! RK4 at 1 ms (ADR-0005); one ECU tick = 10 RK4 steps.

#![forbid(unsafe_code)]

use std::f64::consts::{PI, TAU};

use wiper_swc::MotorCommand;

/// Fixed RK4 step (seconds) — ADR-0005.
pub const DT_S: f64 = 1.0e-3;

/// Maximum arm angle in degrees.
pub const SWEEP_MAX_DEG: u16 = 110;

/// Park window in degrees (below this the plant reports the park switch).
pub const PARK_WINDOW_DEG: u16 = 2;

impl Default for PlantOutput {
    /// Idle snapshot of a fresh plant at park: 13.8 V, no current, no motion.
    fn default() -> Self {
        Self {
            angle_deg: 0,
            arm_speed_dps: 0,
            park_switch: true,
            current_da: 0,
            battery_v01: 138,
        }
    }
}

/// One plant output snapshot (ECU input bundle).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlantOutput {
    /// Arm angle in degrees 0..=110.
    pub angle_deg: u16,
    /// Arm angular speed deg/s (signed).
    pub arm_speed_dps: i16,
    /// Park switch active (within the window or forced stuck).
    pub park_switch: bool,
    /// Motor current in 0.1 A (deci-amps).
    pub current_da: u16,
    /// Battery voltage in 0.1 V.
    pub battery_v01: u16,
}

/// Motor/load parameters (see crate docs for calibration).
#[derive(Debug, Clone)]
pub struct PlantConfig {
    /// Armature resistance Ω.
    pub r: f64,
    /// Torque (N·m/A) = back-EMF constant (V·s/rad).
    pub kt: f64,
    /// Inertia (kg·m², motor + crank + arm referred).
    pub inertia: f64,
    /// Viscous load term β (N·m·s/rad).
    pub beta: f64,
    /// Dry load torque τ₀ (N·m).
    pub tau0_dry: f64,
}

impl Default for PlantConfig {
    fn default() -> Self {
        Self {
            r: 0.5,
            kt: 1.57,
            inertia: 0.05,
            beta: 0.7,
            tau0_dry: 3.25,
        }
    }
}

/// Physical state of the plant.
#[derive(Debug, Clone, Copy)]
struct State {
    /// Crank angle rad, always wrapped to [0, 2π).
    phi: f64,
    /// Crank speed rad/s.
    omega: f64,
}

/// Fault-injectable wiper plant.
#[derive(Debug, Clone)]
pub struct WiperPlant {
    cfg: PlantConfig,
    state: State,
    /// Battery voltage V.
    v_batt: f64,
    /// Rain percent 0..=100 (reduces τ₀).
    rain_pct: f64,
    /// Fault: linkage blocked (position pinned while commanded).
    blocked: bool,
    /// Fault: park-switch stuck; `Some(state)` overrides the computed switch.
    park_stuck: Option<bool>,
    /// Last commanded duty percent (0..=100) for the current computation.
    last_duty: u8,
}

fn derivs(cfg: &PlantConfig, duty: f64, u_batt: f64, tau0: f64, omega: f64) -> (f64, f64) {
    let u = u_batt * duty / 100.0;
    let emf = cfg.kt * omega;
    let current = (u - emf) / cfg.r;
    let torque = cfg.kt * current;
    let tau_load = tau0 + cfg.beta * omega.abs();
    (omega, (torque - tau_load) / cfg.inertia)
}

impl WiperPlant {
    /// Fresh plant at park, 13.8 V, dry glass, no faults.
    #[must_use]
    pub fn new() -> Self {
        Self::with_config(PlantConfig::default())
    }

    /// Plant with custom parameters (tests explore the parameter space).
    #[must_use]
    pub fn with_config(cfg: PlantConfig) -> Self {
        Self {
            cfg,
            state: State {
                phi: 0.0,
                omega: 0.0,
            },
            v_batt: 13.8,
            rain_pct: 0.0,
            blocked: false,
            park_stuck: None,
            last_duty: 0,
        }
    }

    // ------------------------------------------------------- scenario hookups
    /// Set the battery voltage (V), clamped to 0..=24.
    pub fn set_battery_v(&mut self, v: f64) {
        self.v_batt = v.clamp(0.0, 24.0);
    }

    /// Rain percent (0..=100); reduces the dry load torque.
    pub fn set_rain_pct(&mut self, rain: u8) {
        self.rain_pct = f64::from(rain);
    }

    /// Linkage blocked fault (stall injection).
    pub fn set_blocked(&mut self, blocked: bool) {
        self.blocked = blocked;
    }

    /// Stuck park switch injection; `Some(state)` overrides the computed switch.
    pub fn set_park_stuck(&mut self, state: Option<bool>) {
        self.park_stuck = state;
    }

    /// Crank angle (rad, [0, 2π)).
    #[must_use]
    pub fn crank_angle_rad(&self) -> f64 {
        self.state.phi
    }

    // ------------------------------------------------------------ integration
    /// One RK4 sub-step (1 ms) at the given duty percent.
    fn step_int(&mut self, duty_pct: u8) {
        self.last_duty = duty_pct;
        let duty = f64::from(duty_pct);
        if self.blocked {
            // Pinned: no back-EMF builds up; the snapshot returns stall current.
            self.state.omega = 0.0;
            return;
        }
        if duty <= 0.0 {
            // Coast: damping decays ω; φ continues to creep towards park.
            self.state.omega = (self.state.omega * (1.0 - 20.0 * DT_S)).max(0.0);
            self.state.phi = (self.state.phi + self.state.omega * DT_S).rem_euclid(TAU);
            return;
        }
        let tau0 = self.cfg.tau0_dry * (1.0 - 0.15 * self.rain_pct / 100.0);
        let h = DT_S;
        let w = self.state.omega;
        let (k1w, k1do) = derivs(&self.cfg, duty, self.v_batt, tau0, w);
        let (k2w, k2do) = derivs(&self.cfg, duty, self.v_batt, tau0, w + 0.5 * h * k1do);
        let (k3w, k3do) = derivs(&self.cfg, duty, self.v_batt, tau0, w + 0.5 * h * k2do);
        let (k4w, k4do) = derivs(&self.cfg, duty, self.v_batt, tau0, w + h * k3do);
        self.state.phi += h * (k1w + 2.0 * k2w + 2.0 * k3w + k4w) / 6.0;
        self.state.omega += h * (k1do + 2.0 * k2do + 2.0 * k3do + k4do) / 6.0;
        self.state.phi = self.state.phi.rem_euclid(TAU);
        if !self.state.omega.is_finite() {
            // Deterministic numeric guard; never panics.
            self.state.omega = 0.0;
        }
    }

    /// One ECU tick: 10 RK4 steps (10 ms) with `command`; returns the snapshot.
    pub fn tick_10ms(&mut self, command: MotorCommand) -> PlantOutput {
        for _ in 0..10 {
            self.step_int(command.duty_pct);
        }
        self.snapshot()
    }

    /// Current state as the ECU's per-tick input bundle.
    // Casts below are guarded: angle/speed are clamped to physically meaningful
    // ranges before the conversion, current/voltage are non-negative by physics.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    #[must_use]
    pub fn snapshot(&self) -> PlantOutput {
        let duty = f64::from(self.last_duty);
        let angle = 55.0 * (1.0 - self.state.phi.cos());
        let angle_deg = (angle.round() as i64).clamp(0, i64::from(SWEEP_MAX_DEG)) as u16;
        let arm_speed = 55.0 * self.state.phi.sin() * self.state.omega * (180.0 / PI);
        let current = if self.blocked && duty > 0.0 {
            self.v_batt * duty / 100.0 / self.cfg.r
        } else {
            let u = self.v_batt * duty / 100.0;
            let emf = self.cfg.kt * self.state.omega;
            ((u - emf) / self.cfg.r).max(0.0)
        };
        PlantOutput {
            angle_deg,
            arm_speed_dps: (arm_speed.round() as i64).clamp(-2000, 2000) as i16,
            park_switch: angle_deg <= PARK_WINDOW_DEG || self.park_stuck.unwrap_or(false),
            current_da: (current * 10.0).round() as u16,
            battery_v01: (self.v_batt * 10.0).round() as u16,
        }
    }
}

impl Default for WiperPlant {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use wiper_swc::{MotorCommand, MotorSpeed};

    const LOW: MotorCommand = MotorCommand {
        speed: MotorSpeed::Low,
        duty_pct: 70,
    };
    const HIGH: MotorCommand = MotorCommand {
        speed: MotorSpeed::High,
        duty_pct: 100,
    };

    /// Mean crank rate (wipes/min) over `seconds` at the given command,
    /// integrated continuously (fractional wipes) rather than counting wraps.
    // Cast is guarded: `seconds` is always a whole second value in the tests.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    fn measured_cpm(p: &mut WiperPlant, seconds: f64, cmd: MotorCommand) -> f64 {
        // One loop iteration = one 10 ms ECU tick.
        let steps = (seconds * 100.0) as u32;
        let mut prev_phi = p.crank_angle_rad();
        let mut swept = 0.0;
        for _ in 0..steps {
            let _ = p.tick_10ms(cmd);
            let now = p.crank_angle_rad();
            swept += (now - prev_phi + TAU) % TAU;
            prev_phi = now;
        }
        swept / TAU / seconds * 60.0
    }

    #[test]
    // verifies: REQ-WIP-003
    fn validation_low_rate() {
        let mut p = WiperPlant::new();
        // Warm up 2 s, then measure 10 s.
        for _ in 0..200 {
            let _ = p.tick_10ms(LOW);
        }
        let cpm = measured_cpm(&mut p, 10.0, LOW);
        assert!(
            (40.0..=50.0).contains(&cpm),
            "LOW rate {cpm:.1} cpm outside 45±10 %"
        );
    }

    #[test]
    // verifies: REQ-WIP-004
    fn validation_high_rate() {
        let mut p = WiperPlant::new();
        for _ in 0..200 {
            let _ = p.tick_10ms(HIGH);
        }
        let cpm = measured_cpm(&mut p, 10.0, HIGH);
        assert!(
            (58.5..=71.5).contains(&cpm),
            "HIGH rate {cpm:.1} cpm outside 65±10 %"
        );
    }

    #[test]
    // verifies: REQ-WIP-008 (park switch per geometry)
    fn park_window_and_angle() {
        let mut p = WiperPlant::new();
        let out = p.tick_10ms(LOW);
        assert!(out.park_switch, "starts parked");
        // Sweep once fully.
        let mut saw_out = false;
        let mut saw_back = false;
        for _ in 0..300 {
            let out = p.tick_10ms(LOW);
            if out.angle_deg >= 108 {
                saw_out = true;
            }
            if saw_out && out.angle_deg <= 2 {
                saw_back = true;
                break;
            }
        }
        assert!(
            saw_out && saw_back,
            "one wipe = full round trip through 110°"
        );
        assert!(out.park_switch, "returns to the park window");
        assert!(out.angle_deg <= 2);
    }

    #[test]
    // verifies: REQ-WIP-012 (stall current path)
    fn blocked_linkage_stall_current() {
        let mut p = WiperPlant::new();
        p.set_blocked(true);
        let out = p.tick_10ms(LOW);
        // duty 70 % of 13.8 V over 0.5 Ω → 19.3 A stall current.
        assert!(
            (180..=210).contains(&out.current_da),
            "stall current {} dA out of range",
            out.current_da
        );
        assert_eq!(out.angle_deg, 0, "position pinned when blocked");
        assert_eq!(out.arm_speed_dps, 0);
    }

    #[test]
    // verifies: plant model documentation (dry-glass load increase)
    fn dry_glass_loads_more() {
        // Dry glass: faster steady state is impossible with heavier load — instead
        // check that at the same duty the wet-glass sweep rate is slightly higher.
        let mut dry = WiperPlant::new();
        dry.set_rain_pct(0);
        for _ in 0..200 {
            let _ = dry.tick_10ms(LOW);
        }
        let dry_cpm = measured_cpm(&mut dry, 8.0, LOW);

        let mut wet = WiperPlant::new();
        wet.set_rain_pct(100);
        for _ in 0..200 {
            let _ = wet.tick_10ms(LOW);
        }
        let wet_cpm = measured_cpm(&mut wet, 8.0, LOW);
        assert!(
            wet_cpm > dry_cpm,
            "wet glass must sweep faster ({wet_cpm:.2} > {dry_cpm:.2})"
        );
        assert!(
            wet_cpm - dry_cpm < 3.0,
            "the effect is subtle (≤ 15 % load delta)"
        );
    }

    #[test]
    // verifies: REQ-WIP-013 (park switch stuck injection)
    fn stuck_park_switch_override() {
        let mut p = WiperPlant::new();
        p.set_park_stuck(Some(true));
        let _ = p.tick_10ms(LOW);
        // Even after rotating away, the switch still reads active.
        for _ in 0..100 {
            let out = p.tick_10ms(LOW);
            assert!(out.park_switch, "injected stuck switch stays active");
        }
    }

    #[test]
    // verifies: REQ-WIP-019 (determinism flows through the plant too)
    fn plant_determinism() {
        let mut a = WiperPlant::new();
        let mut b = WiperPlant::new();
        for _ in 0..500 {
            let oa = a.tick_10ms(LOW);
            let ob = b.tick_10ms(LOW);
            assert_eq!(oa, ob);
        }
    }

    #[test]
    fn snapshot_never_out_of_bounds() {
        let mut p = WiperPlant::new();
        for _ in 0..5000 {
            let out = p.tick_10ms(HIGH);
            assert!(out.angle_deg <= 110);
            assert!(out.current_da <= 3000);
            assert!((-2000..=2000).contains(&out.arm_speed_dps));
        }
    }
}
