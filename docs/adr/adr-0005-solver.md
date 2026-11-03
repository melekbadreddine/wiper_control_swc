# ADR-0005 — Fixed-step solver: RK4 at 1 ms for the plant

* Status: **Accepted** (milestone M4, chosen in M0 planning)
* Date: 2026-11-03

## Context
The plant (DC motor + linkage + load) is integrated on the host without Simulink.
Options: explicit Euler (simple, inaccurate), RK4 (4 evaluations), implicit/CVODE
(heavy deps).

## Decision
Fixed-step **Runge-Kutta 4** with a **1 ms** step, running at a 10:1 ratio against the
ECU's 10 ms control tick (documented in `docs/plant-model.md`). Chosen after
validating the plant against analytic expectations (tests in M4); if stability fails,
step size is reduced before considering an implicit solver.

## Consequences
Determinism and reproducibility are easy (same step, same result). Computational cost
is 4 derivative evaluations per ms — negligible for this system size.
