# Requirements

Single source of truth for the windshield wiper control function. IDs are stable;
design elements reference the implementing Rust crate/module; tests link via the
machine-checkable tag `// verifies: <ID>` in Rust sources. `cargo xtask traceability`
generates `docs/traceability.md` and fails when a requirement has no verifying test
(strict mode from M5; `pending` entries are allowed until their milestone ships).

## System scope

Front windshield wiper system of a passenger car: two arms, one DC motor with gear
and crank linkage, one washer pump.

**Inputs**
| Name | Spec | Notes |
|---|---|---|
| Stalk position | `OFF` / `INT` (sensitivity 1–4, auto flag) / `LOW` / `HIGH` | primary user command |
| Wash request | momentary `WASH` button | pump + 3 wipes after release |
| Rain intensity | 0–100 % | rain/light sensor, auto mode input |
| Vehicle speed | 0…250 km/h | shortens INT pause |
| Ignition | `OFF` / `ACC` / `ON` | gating |
| Park switch | boolean, arm at rest position | end-of-cycle control |
| Motor current | 0–30 A | stall detection |
| Battery voltage | 0–24 V (valid 9–16) | over/under-voltage supervision |

**Outputs**: motor command (`off` / `low` / `high` + PWM duty 0–100 %), washer pump
request, diagnostic status + DTCs.

## Functional requirements

| ID | Title | Text | Design | Tests |
|---|---|---|---|---|
| REQ-WIP-001 | Ignition gating | The wiper motor shall run only with ignition `ON`; the washer pump is additionally allowed in `ACC`. With ignition `OFF`, motor and pump commands shall be `off`. | `wiper-swc::fsm` | `fsm_ignition_gate (M3)` |
| REQ-WIP-002 | Stalk modes | Stalk supports `OFF`, `INT` (with 4 sensitivity levels and auto flag), `LOW`, `HIGH`. Mode changes take effect within one 10 ms task tick. | `wiper-swc::fsm` | `fsm_mode_switching (M3)` |
| REQ-WIP-003 | LOW sweep rate | `LOW` shall produce ≈45 wipes/min (one wipe = complete sweep and return through park). | `wiper-swc::fsm`, `plant-sim` | `scenario_low_rate (M5)` |
| REQ-WIP-004 | HIGH sweep rate | `HIGH` shall produce ≈65 wipes/min. | `wiper-swc::fsm`, `plant-sim` | `scenario_high_rate (M5)` |
| REQ-WIP-005 | INT base pause | In `INT` (manual timing), the pause between wipes depends on sensitivity: level 1 = 2 s, 2 = 5 s, 3 = 7 s, 4 = 11 s (public reference time = time from one wipe end to next wipe start). | `wiper-swc::intermittent` | `intermittent_base_pause (M3)` |
| REQ-WIP-006 | INT speed dependence | In `INT`, the pause is shortened as vehicle speed rises: factor = 1.0 below 20 km/h decreasing linearly to 0.5 at ≥120 km/h. | `wiper-swc::intermittent` | `intermittent_speed_dependence (M3)` |
| REQ-WIP-007 | Auto mode (rain sensor) | With stalk `INT` + auto flag, wipes trigger when rain intensity exceeds a sensitivity-dependent threshold; sensitivity 1–4 thresholds: 60/40/25/10 %. While wiping, the wipe repeats (continuous wiping) as long as rain exceeds the threshold + hysteresis of −5 %; else pause per REQ-WIP-005/006. | `wiper-swc::intermittent` | `interop_auto_mode (M3)` |
| REQ-WIP-008 | Park discipline | When the mode switches to `OFF` (or a fault forces stop), the arm shall complete the current sweep from its position and stop only in park position (park switch active); never stop mid-windshield. | `wiper-swc::fsm`, `wiper-swc::park` | `fsm_park_completion (M3)`, `proptest_park_only_stop (M3)` |
| REQ-WIP-009 | Wash sequence | While `WASH` is pressed: pump on and continuous wiping. After release: exactly 3 more wipes, then return to the previous stalk mode. | `wiper-swc::fsm` | `fsm_wash_sequence (M3)` |
| REQ-WIP-010 | Wash pump cap | The washer pump shall run at most 10 s continuously regardless of button state (over-run protection). | `wiper-swc::wash` | `fsm_wash_pump_cap (M3)` |
| REQ-WIP-011 | Wash in ACC | `Wash` operation is permitted in `ACC` and `ON` ignition. | `wiper-swc::fsm` | `fsm_ignition_gate` (M3) |
| REQ-WIP-012 | Stall detection & retries | Motor current > 8 A with no arm movement for > 500 ms ⇒ stall: motor off, DTC set, retry after 2 s cool-down; after 3 failed retries latch the failure (no further retries until ignition cycle or mode change away and back). | `wiper-swc::diag` | `diag_stall_retry_latch (M3)` |
| REQ-WIP-013 | Park-switch plausibility | If the arm moves out of park while the park switch remains active (stuck) for > 200 ms, set DTC and use the arm-angle estimate for park detection. | `wiper-swc::diag` | `diag_park_implausible (M3)` |
| REQ-WIP-014 | Voltage window | Outside 9–16 V, motor and pump commands inhibited; DTC for under-/over-voltage. Recovery inside 9.5–15.5 V with 1 s debounce. | `wiper-swc::diag` | `diag_voltage_window (M3)` |
| REQ-WIP-015 | Sensor fallback | On stalk-signal loss (> 1 s without valid frame) the function degrades to `LOW` (visible wiping) and sets DTC; on rain-sensor loss, INT auto mode falls back to the manual timing of the active sensitivity level. | `wiper-swc::fsm`, `wiper-swc::diag` | `fsm_stalk_timeout_fallback`, `scenario_sensor_timeout (M5)` |
| REQ-WIP-016 | Pump over-run interval | Washer pump retrigger requires ≥ 3 s off-time between activations (duty-cycle protection). | `wiper-swc::wash` | `fsm_wash_pump_cap` (M3) |
| REQ-WIP-017 | Diagnostic exposure | Diagnostics runnable (`Wiper_Run100ms`) shall publish status + active DTC snapshot each cycle. | `wiper-swc::diag`, `bsw-sim` | `diag_snapshot_cycle (M3)` |
| REQ-WIP-018 | PWM mapping | Motor command duty: `HIGH` = 100 %, `LOW` = 70 %, INT sweeps use the `low` profile; duty respects the plant's voltage compensation limit. | `wiper-swc::fsm`, `plant-sim` | `scenario_low_rate` (M5) |
| REQ-WIP-019 | Determinism | Given the same input history, `wiper-swc` shall produce identical output histories (no randomness, no wall clock; time enters only via simulated task timers). | whole crate | `proptest_determinism (M3)` |
| REQ-WIP-020 | Timing budget | The 10 ms control runnable shall have simulated worst-case execution < 1 ms on the reference host (measured by the criterion benchmark) and is scheduled at 10 ms by bsw-sim OS regardless. | `bsw-sim::os` | `os_timing_budget (M4)`, `criterion::bench_run10ms (M5)` |
| REQ-WIP-021 | Runnables | Expose AUTOSAR-style runnables `Wiper_Init`, `Wiper_Run10ms`, `Wiper_Run100ms`, `Wiper_OnStalkChange` with the documented triggers. | `wiper-swc` API | `arxml_consistency_ports (M6)` (M6), Rust API itself |
| REQ-WIP-022 | Port mapping | AUTOSAR sender/receiver ports map 1:1 to ROS 2 topics, client/server to services, mode switch to latched topics (table in docs/ros2-mapping.md). | `ros2-nodes` (M7) | node launch test (M7) |

## Verification requirement

| ID | Title | Text | Design | Tests |
|---|---|---|---|---|
| REQ-WIP-090 | Coverage gate | `wiper-swc` shall hold ≥ 90 % line coverage measured by `cargo llvm-cov` in CI. | Makefile `coverage` | CI run (M9) |

## Safety goals (from HARA-lite, see hara.md)

| ID | Safety goal | ASIL (assumed, academic) |
|---|---|---|
| SG-001 | Prevent unintended wiper stop in `HIGH`-demand rain conditions at speed | A (assumed) |
| SG-002 | Prevent wiping when stationary & user off (no unintended activation) | QM→A (assumed) |
| SG-003 | Ensure arm reaches park after `OFF` | B (assumed) |
