# PROGRESS.md

Status: **in progress**. Checklist below is the source of truth; update after every
milestone. Resume work from the first unchecked box.

| ✅ | Milestone | Deliverables | Tag | Notes |
|---|-----------|--------------|-----|-------|
| ✅ | M0 Bootstrap | git, workspace skeleton, toolchain pin, lints, configs, Makefile, AGENTS/PROGRESS/README/LICENSE | `m0` | cf35cee |
| ✅ | M1 Requirements & safety | requirements.md, HARA-lite, Classic vs Adaptive, ADRs, mdBook skeleton | `m1` | 4e55254 (mdbook 0.5.4 build verified) |
| ✅ | M2 AUTOSAR RTE & interfaces | autosar-rte crate + tests, port/interface defs, DBC | `m2` | 647be4e (8 tests, clippy -D warnings green) |
| ☐ | M3 Wiper SWC logic | FSM, intermittent/park/diag logic, unit+property tests, coverage ≥90 % gate | `m3` | |
| ☐ | M4 BSW sim + plant | bsw-sim (tasks/COM/IO/DEM), plant-sim RK4, equations doc, plant tests | `m4` | |
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
