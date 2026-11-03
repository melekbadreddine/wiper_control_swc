# AGENTS.md — Project Conventions (Wiper Control SWC)

> Read this file first in every session. If context is lost, resume from `PROGRESS.md`
> (contains the milestone checklist and status journal).

## Project

**Windshield Wiper Control Software Component (SWC)** — AUTOSAR Classic Platform
conventions, implemented in Rust, simulated closed-loop (plant + ECU), integrated with
ROS 2 and visualized in Foxglove Studio (URDF 3D car, plots, gauges, FSM panel).

University-grade but professional quality. **No MATLAB/Simulink** — the workflow is
recreated in code: requirements → architecture → executable model → simulation →
verification.

## Working rules (binding)

1. Work milestone by milestone (see `PROGRESS.md`). After each milestone:
   - run `make check`, fix everything until green,
   - update `PROGRESS.md` and docs,
   - `git add -A && git commit` with **Conventional Commits** (`feat:`, `test:`, `docs:`,
     `ci:`, `build:`, `chore:`), then `git tag m<N>`.
   - Never commit a failing build. Never skip a milestone.
2. Check current crate versions/APIs before using them (do not rely on memory). If a
   crate or tool misbehaves, pick the closest alternative and record the decision in
   `docs/adr/`.
3. **Nothing may be faked.** Every claim in docs (coverage, passing tests, timing) must
   come from real command output. No TODO stubs — out-of-scope items go explicitly into
   `docs/limitations.md`.
4. Ask the user nothing; make reasonable decisions, document them (ADR + `PROGRESS.md`
   journal).
5. All documentation in English.
6. Everything must run in Docker; natively where possible.

## Engineering conventions

### Rust
- **Edition 2021**, stable toolchain pinned in `rust-toolchain.toml` (currently 1.97.1),
  MSRV 1.85.
- Workspace-level lints in `[workspace.lints]`; every crate registers via
  `[lints] workspace = true`.
- `autosar-rte`, `wiper-swc`: `#![no_std]`, no heap allocation;
  `wiper-swc`: additionally `#![forbid(unsafe_code)]`, no `unwrap/expect/panic/indexing`
  (clippy `deny`). Floats avoided where fixed-point suffices (document choice).
- `bsw-sim`, `plant-sim`, `scenario-runner`, `arxml-gen`, `xtask`: std allowed (offline
  desktop simulation crates — documented in ADR-0003).
- `ros2-nodes`: ROS-free by default; real ROS code behind the `ros` cargo feature
  (r2r), built inside the ROS 2 Docker image. Keeps native `cargo test --workspace`
  working without ROS installed (ADR-0004).
- Naming: AUTOSAR style in application crates — runnables `Wiper_Init`, `Wiper_Run10ms`,
  `Wiper_Run100ms`, `Wiper_OnStalkChange`; port accessors styled after
  `Rte_Read_<port>_<elem>` / `Rte_Write_<port>_<elem>` (via typed traits/macros).
- Hexagonal architecture: `wiper-swc` knows nothing about ROS, CAN, or files. ROS nodes
  and `bsw-sim`/`scenario-runner` are adapters.

### Process
- Makefile is the single entry point -> target names in README and `make help`.
- Requirements IDs: `REQ-WIP-001`, `SG-001` (safety goals). Traceability tag in tests:
  `// verifies: REQ-WIP-xxx`; `cargo xtask traceability` generates `docs/traceability.md`
  and fails if any requirement lacks a test.
- Diagrams: Mermaid in mdBook sources.
- Every claim in docs must cite the command that produced it.

## Layout

```
crates/{autosar-rte,wiper-swc,bsw-sim,plant-sim,scenario-runner,ros2-nodes,arxml-gen,xtask}
ros2_ws/src/{wiper_interfaces,wiper_description}
scenarios/  arxml/  foxglove/layouts/  docs/(adr|diagrams)/  docker/  .devcontainer/  .github/  tests/
```

## Session journal

Append dated one-line notes here at the end of a session (keeps cross-session memory):

- 2026-11-03 — M0 completed: workspace + 8 crates bootstrapped, strict lints green,
  tagged `m0` (see PROGRESS.md for the exact commit).
