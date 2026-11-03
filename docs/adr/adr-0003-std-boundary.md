# ADR-0003 — `no_std` boundary between ECU code and host simulators

* Status: **Accepted** (milestone M0)
* Date: 2026-11-03

## Context
The SWC must be deployable on an MCU; host-side simulators only need to run on the
development PC. Mixing both freely would let hidden host assumptions leak into
"ECU-like" code (heaps, floats, unwinding).

## Decision
- `crates/autosar-rte` and `crates/wiper-swc`: `#![no_std]`, no heap, and for
  `wiper-swc` additionally `#![forbid(unsafe_code)]` and clippy `deny` on
  `unwrap/expect/panic/indexing_slicing`.
- `crates/bsw-sim`, `crates/plant-sim`, `crates/scenario-runner`, `crates/arxml-gen`,
  `crates/xtask`: ordinary `std` crates — they are simulation/host tooling by nature.

## Consequences
`wiper-swc` uses deterministic fixed-point arithmetic and bounded buffers. Host crates
may use floats for physics (documented in `docs/plant-model.md`). The boundary is
enforced by the compiler, so drift is not a code-review concern.
