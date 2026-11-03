//! AUTOSAR ARXML generator and consistency checker (M6 — see `PROGRESS.md`).
//!
//! Will use the `autosar-data` crate (0.23.x, AUTOSAR 4.x schema) to emit the SWC
//! description (ports, interfaces, data types, runnable entities + timing events,
//! internal behavior) and an ECU composition / system description, plus a CLI
//! (`generate`, `check`) that validates consistency with the Rust implementation.

#![forbid(unsafe_code)]

fn main() {
    println!("arxml-gen: AUTOSAR 4.x generation lands in milestone M6 (see PROGRESS.md)");
}
