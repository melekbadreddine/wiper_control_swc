# Contributing

- Follow `AGENTS.md` (Conventional Commits, milestone flow, quality gates).
- Every feature needs tests; requirements need traceability tags
  (`// verifies: REQ-WIP-xxx`) — `cargo xtask traceability` enforces it.
- `make check` must pass before every commit; milestones are tagged `m0..m10`.
- Run `make setup` once to install `cargo-llvm-cov`, `cargo-deny`, `cargo-audit`,
  `mdbook`.
