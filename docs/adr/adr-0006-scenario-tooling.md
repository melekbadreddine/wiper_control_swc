# ADR-0006 — Scenario runner tooling and dependency choices

* Status: **Accepted** (milestone M5)
* Date: 2026-11-04

## Context
M5 needs to run TOML-defined closed-loop scenarios headlessly, record CSV and MCAP
output, evaluate pass/fail checks against golden data, and time the 10 ms runnable.
Candidate dependencies for each need:

| Need | Candidate | Costs |
|---|---|---|
| Scenario format | `serde` + `toml` | two crates, cached; de-facto standard |
| CLI | `clap` vs hand-rolled | clap pulls `clap_builder`/`clap_lex`; existing tools (`xtask`, `arxml-gen`) hand-roll args over ~4 flags |
| MCAP output | `mcap` crate vs hand-rolled writer | official foxglove crate; schema/format correctness handled upstream — hand-rolling a spec-correct MCAP (chunks, CRCs, summary) is error-prone |
| Benchmark | `criterion` vs timing test | criterion pulls a large transitive tree mainly for statistical niceties; the requirement (WCE < 1 ms) is a single number per tick |

## Decision
1. `serde` (derive) + `toml` v1 for the scenario schema — standard, validated parsing,
   explicit `deny_unknown_fields` so typos in scenario files fail loudly.
2. **Hand-rolled argument parsing** in `scenario-runner::main` (consistent with the
   other workspace CLIs): `--scenario-dir`, `--out`, optional repeated `--scenario`,
   `--no-csv`, `--no-mcap`, `--list`.
3. `mcap` v0.26 with `default-features = false` (no lz4/zstd/tokio) for real MCAP
   output; messages are JSON payloads (`schema: json`) so Foxglove renders them in M8.
4. **No criterion** in the workspace for now: the 10 ms WCE gate is enforced by
   `run10ms_worst_case_under_1ms` (`std::time::Instant` around `N` `Ecu::tick` calls,
   worst case over the loop) so it runs under plain `cargo test`. The requirement
   text (REQ-WIP-020) is updated to name this test instead of `criterion`; if finer
   statistics are ever needed, criterion can be added as a `benches/` target later.

## Consequences
All checks stay reproducible from `cargo test`/`make check` without extra tooling.
Costs: custom arg parsing must grow by hand (acceptable at this CLI size); the mcap
writer carries no compression (files stay slightly larger but simpler).
