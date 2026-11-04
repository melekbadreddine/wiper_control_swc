//! Headless closed-loop scenario runner (library part).
//!
//! TOML scenarios (`scenarios/`) describe setup + timed events + checks; the
//! runner drives [`bsw_sim::Ecu`] in virtual time, records CSV/MCAP output
//! (`report/`), evaluates pass/fail checks. CLI in the same crate
//! (`crate::main`): see M5 documentation in `docs/`.

#![forbid(unsafe_code)]

pub mod output;
pub mod runner;
pub mod scenario;

pub use output::{McapLog, SnapshotRow};
pub use runner::{run, CheckOutcome, RunResult};
pub use scenario::{Scenario, Setup, StalkSpec};
