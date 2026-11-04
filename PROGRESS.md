# PROGRESS.md

Status: **in progress**. Checklist below is the source of truth; update after every
milestone. Resume work from the first unchecked box.

| ✅ | Milestone | Deliverables | Tag | Notes |
|---|-----------|--------------|-----|-------|
| ✅ | M0 Bootstrap | git, workspace skeleton, toolchain pin, lints, configs, Makefile, AGENTS/PROGRESS/README/LICENSE | `m0` | cf35cee |
| ✅ | M1 Requirements & safety | requirements.md, HARA-lite, Classic vs Adaptive, ADRs, mdBook skeleton | `m1` | 4e55254 (mdbook 0.5.4 build verified) |
| ✅ | M2 AUTOSAR RTE & interfaces | autosar-rte crate + tests, port/interface defs, DBC | `m2` | 647be4e (8 tests, clippy -D warnings green) |
| ✅ | M3 Wiper SWC logic | FSM, intermittent/park/diag logic, unit+property tests, coverage ≥90 % gate | `m3` | 19 unit + 4 prop tests; wiper-swc 94.3 % / autosar-rte 91.2 % lines (llvm-cov --fail-under-lines 90, exit 0) |
| ✅ | M4 BSW sim + plant | bsw-sim (sched/COM/IoHwAb/DEM/NvM/Ecu), plant-sim RK4, docs/plant-model.md + docs/bsw-sim.md, 28 bsw-sim tests | `m4` | plant calibrated Kt=1.57/τ₀=3.25 (LOW 45.94, HIGH 67.99 cpm, wet +0.82, stall 193 dA, measured over real `cargo test` output); `make check` green; DBC sensitivity field encoding fixed (see PROGRESS decisions) |
| ☐ | M5 Closed loop + scenarios | scenario-runner CLI, scenarios, fault injection, CSV/MCAP, golden tests | `m5` | |
| ☐ | M6 ARXML gen & validation | arxml-gen, ARXML 4.x model, consistency tests vs Rust code | `m6` | |
| ☐ | M7 ROS 2 integration | msg/srv, nodes (r2r), launch/mapping doc, launch tests | `m7` | |
| ☐ | M8 URDF + Foxglove | xacro car, joint_states, rain markers, bridge, layout, record/replay | `m8` | |
| ☐ | M9 Docker + CI/CD | Dockerfile, compose, GH workflows, devcontainer, release pipeline | `m9` | |
| ☐ | M10 Docs & polish | full mdBook, traceability + coverage reports, presentation, demo script, `v1.0.0` | `m10` / `v1.0.0` | |

## Decisions log (append as they are made)

- **Toolchain:** stable 1.97.1 (installed locally), pinned in `rust-toolchain.toml`,
  MSRV 1.85.
- **License:** MIT (fast, permissive, fine for academic artifact; re-evaluate if a
  supervisor requires something else).
- **no_std boundary:** `autosar-rte` and `wiper-swc` are `no_std` (real ECU code);
  `bsw-sim`/`plant-sim` host-side simulators use `std` (documented in ADR-0003).
- **ROS 2 behind a cargo feature:** `ros2-nodes` builds ROS-free by default so the whole
  workspace tests natively without ROS installed; real node code lives behind the
  `ros` feature and is built inside the Jazzy Docker image (ADR-0004).
- **ROS 2 distribution:** Jazzy Jalisco (LTS, Ubuntu 24.04 base) — to be re-verified
  against docker hub availability in M9 and recorded in ADR.
- **Plant recalibration (2026-11-04):** two-point equilibrium solve with `R`, `β`, `J`
  fixed gave `Kt = 1.57`, `τ₀ = 3.25 N·m` (0.22 root rejected as unphysical);
  measured LOW 45.94 / HIGH 67.99 cpm, wet LOW 46.76 (−15 % τ₀ rain effect),
  stall 193 dA. Derivation + real output in `docs/plant-model.md`.
- **`measured_cpm` swept-angle integration:** rate test integrates the swept crank
  angle (fractional wipes) instead of counting integer crosses; also fixes the old
  10× over-count (helper ran `seconds·1000` 10 ms ticks).
- **DBC sensitivity encoding fixed (2026-11-04):** the 2-bit `Sensitivity` field
  `[0|3]` cannot hold levels 1..4 directly; comment + `VAL_` now use level-1
  offset encoding (raw 0..3 = S1..S4). No code parsed the DBC before this change.
- **COM alive supervision (M4):** frame 672's `StalkCounter` must alternate on
  every accepted frame; the 1 s window (REQ-WIP-015) triggers an invalid-push
  fallback. Rain/speed use RTE receiver staleness timeouts (also 1 s). Frame-675
  `DelayCounter` = INT delay-pause indicator (documented interpretation).
- **Virtual CAN bus receive-once:** each physical transmission is consumed
  exactly once (`CanBus::take`); frame 675 is a published latest-frame with
  `read()` for display consumers.
- **NvM scope (M4):** RAM-mirror job-queue block (`dtc_latched_bits`,
  `wipe_count`); no power-cycle persistence — recorded in `docs/limitations.md`.

## Session journal

- 2026-11-03 — M0 completed: workspace + 8 crates bootstrapped, strict lints green,
  tagged `m0` (see PROGRESS.md for the exact commit).
- 2026-11-04 — M4 completed: plant-sim calibrated + validated (8 tests, real measured
  rates in docs/plant-model.md); bsw-sim implemented (sched/COM/IoHwAb/DEM/DET/NvM/Ecu,
  18 unit + 10 closed-loop tests; closed-loop LOW 45–46 / HIGH 68 cpm verified
  end-to-end; stalk/rain loss fallbacks, undervoltage window, stall retry latch,
  wash sequence, status frame via `make check`-verified suite). docs: plant-model.md,
  bsw-sim.md, requirements/traceability/limitations updated; DBC sensitivity
  encoding fixed (offset encoding, see decisions). Tagged `m4`.
