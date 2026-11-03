//! Minimal AUTOSAR Classic **RTE-style** abstraction layer.
//!
//! This crate is part of the wiper-control-swc project. It provides the port and
//! runnable abstractions that application software components (e.g. `wiper-swc`)
//! use to stay independent of the actual communication stack: non-queued
//! sender/receiver ports (last-is-best semantics like `Rte_Read_/Rte_Write_`),
//! queued receiver ports, client/server calls, mode switching, and runnable
//! definitions triggered by real-time events (e.g. a 10 ms cycle).
//!
//! Implemented in milestone M2 — see `PROGRESS.md`. Full behavioral tests live with
//! the implementations that land there.

#![no_std]
#![forbid(unsafe_code)]

/// Placeholder used by the M0 bootstrap gate only; replaced by real API surface.
pub const RTE_MODEL_VERSION: &str = "AUTOSAR 4.x (mapping in crate docs, milestone M2)";

#[cfg(test)]
mod tests {
    use super::RTE_MODEL_VERSION;

    #[test]
    fn crate_compiles_no_std() {
        assert!(!RTE_MODEL_VERSION.is_empty());
    }
}
