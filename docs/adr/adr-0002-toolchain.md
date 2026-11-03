# ADR-0002 — Toolchain: stable pinning, MSRV 1.85

* Status: **Accepted** (milestone M0)
* Date: 2026-11-03

## Context
CI reproducibility and feature stability need a pinned toolchain; older university
machines should still build (MSRV).

## Decision
- `rust-toolchain.toml` pins **channel 1.97.1** (the host machine's installed stable,
  2026-06-30).
- `rust-version = "1.85"` (workspace): first stable that shipped edition-2021-era
  quality-of-life we rely on (and the value accepted broadly in 2025 as an MSRV
  baseline in vehicle code).

## Consequences
CI matrix runs stable + MSRV (1.85). Toolchain updates are deliberate ADR bumps.
