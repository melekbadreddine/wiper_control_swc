# Plant model: wiper motor + crank linkage

Physical model simulated by `crates/plant-sim`. It closes the control loop around
the ECU so that every dynamic requirement (REQ-WIP-003/004/008/012/013) is checked
against a real actuator, not against the SWC's own assumptions.

## 1. Physics

The actuator is a permanent-magnet DC motor geared to a crank–linkage mechanism.
A crank revolution maps to one complete wipe through an approximately symmetric
coupling (crank angle `φ` → arm angle `θ`, in degrees):

```text
θ(φ) = 55° · (1 − cos φ)          (park → 110° → park = one wipe)
ω_arm = 55° · sin φ · ω           (fair lead at the two stroke ends)
```

The full drivetrain (motor + gear + crank + arm) is referred to the motor shaft
with an effective inertia `J`. The electrical and mechanical equations after PWM
averaging (duty `d` of battery voltage `U_batt`):

```text
U      = d · U_batt                      (PWM average, 0 ≤ d ≤ 1)
I      = (U − Ke·ω) / R                  (armature current, Ke = back-EMF const.)
τ_m    = Kt · I                          (motor torque)
τ_load = τ₀ · (1 − 0.15·rain/100) + β·|ω| (contact friction + viscous drag)
J·dω/dt = τ_m − τ_load                   (rigid drivetrain)
dφ/dt   = ω
```

The dry-glass contact term dominates (`τ₀`), which is the physical basis of the
"wet glass wipes slightly faster" behaviour tested in `dry_glass_loads_more`
(dry load is `1/(1 − 0.15)` ≈ 1.18× the fully wetted load).

## 2. Parameters and calibration

Table of the calibrated values (all compiled into `PlantConfig::default()`):

| Symbol | Value | Meaning |
|---|---|---|
| `R`    | 0.5 Ω | armature resistance (12 V wiper motor class) |
| `Kt = Ke` | 1.57 N·m/A = V·s/rad | torque / back-EMF constant (SI symmetry) |
| `J`    | 0.05 kg·m² | drivetrain inertia referred to the motor shaft |
| `β`    | 0.7 N·m·s/rad | viscous (linkage + bearing) drag |
| `τ₀`   | 3.25 N·m | dry-glass contact load |

Calibration method (two-point solve, constraints fixed): the steady state of the
open-loop plant is

```text
ω_eq(d) = (Kt·d·U_batt/R − τ₀) / (Kt²/R + β)
```

Requirements fix two speeds at `U_batt = 13.8 V` (nominal charging voltage):
`d = 0.7` ⇒ `ω_eq = 4.81 rad/s` (≈46 wipes/min, REQ-WIP-003) and `d = 1.0` ⇒
`ω_eq = 7.12 rad/s` (≈68 wipes/min, REQ-WIP-004). Subtracting the two equations
eliminates `τ₀` and gives quadratic `4.62·Kt² − 8.28·Kt + 1.617 = 0` with roots
`Kt ∈ {1.568, 0.223}`; the small root inverts the load-torque ratio between
duties (unphysical) and is rejected. Back-substitution gives `τ₀ = 3.25 N·m`.

### Calibration proof (real command output)

From `cargo test -p plant-sim` (RK4 at 1 ms, see §3; warm-up 2 s, measured 10 s):

```text
LOW duty=70%  rate=45.94 cpm     (acceptance window 40.5–49.5, REQ-WIP-003)
HIGH duty=100% rate=67.99 cpm    (acceptance window 58.5–71.5, REQ-WIP-004)
wet LOW  rate=46.76 cpm          (+0.82 cpm at 100 % rain — subtle, as designed)
stall current=19.3 A (193 dA)    (blocked linkage at duty 70 %, REQ-WIP-012)
```

These are also re-enforced permanently by the unit tests `validation_low_rate`,
`validation_high_rate`, `dry_glass_loads_more` and `blocked_linkage_stall_current`
(`crates/plant-sim/src/lib.rs`, tags `// verifies: REQ-WIP-003/004/012`).

### Electrical sanity cross-check

At duty 70 % and parked (ω = 0) the stall current is `0.7·13.8/0.5 = 19.3 A` —
the exact value the stall monitor (REQ-WIP-011/012, STALL_CURRENT_DA = 100 dA
from `wiper-swc::params`) distinguishes from normal running current
`(9.66 − 1.57·ω)/0.5 ≈ 5 A` near the operating point. The plant therefore
exercises the SWC's stall discrimination with a physically consistent margin.

## 3. Integration

- Fixed-step **RK4 at 1 ms** (`DT_S = 1.0e-3`), ADR-0005. One ECU tick
  (`tick_10ms`) = 10 RK4 sub-steps at the SWC's duty command — this reproduces
  the real scheduling (10 ms control period, 1 ms drive layer).
- Stability: the mechanical pole is `−(Kt²/R + β)/J = −117 s⁻¹`; at h = 1 ms the
  RK4 region easily covers it, so rates from the sim are integrator-independent.
- Discontinuities are handled outside the ODE: command = 0 coasts with an
  exponential decay (ω → 0 as the arm creeps to park through load drag), and the
  `blocked` fault pins the position while the plant reports the electric stall
  current instead of integrating.
- `φ` is wrapped to `[0, 2π)` every step; all snapshot quantities are derived
  (angle, arm speed, park switch, current) and clamped to physical bounds, so
  sequential fault injections cannot produce non-physical outputs
  (`snapshot_never_out_of_bounds`).

## 4. Fault injections (scenario hooks)

| API | Fault | Requirement exercised |
|---|---|---|
| `set_battery_v(v)` | weak battery / load dump | REQ-WIP-017 (voltage window) |
| `set_rain_pct(n)` | weather variation 0–100 % | REQ-WIP-007 |
| `set_blocked(true)` | linkage blocked/debris | REQ-WIP-012 (stall detect + retries) |
| `set_park_stuck(Some(x))` | park switch stuck at x | REQ-WIP-013 (override rule) |

All injections are deterministic; the same input sequence reproduces the same
output tick-for-tick (`plant_determinism`, REQ-WIP-019).

## 5. Known simplifications

- No mechanical backlash or stiction (wiper blades on modern cars judder at
  higher speeds — not modelled; documented in `docs/limitations.md`).
- The rain-dependence of `τ₀` is linear and bounded at 15 %; a real film-drag
  model is beyond the scope.
- Battery voltage does not sag under motor load (the source is ideal; the SWC's
  voltage-window logic is exercised by direct injection instead).
