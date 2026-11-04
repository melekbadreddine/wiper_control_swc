//! Simulated AUTOSAR **basic software** for the wiper ECU (host-side stand-in
//! for the OS, COM, `IoHwAb`, DEM, `NvM` layers).
//!
//! Built around the real application ([`wiper_swc::WiperSwc`], unchanged) and
//! the physical plant ([`plant_sim::WiperPlant`]), this crate closes the loop:
//! virtual CAN bus → COM decode/alive supervision → RTE ports → scheduled
//! `Wiper_*` runnables → `IoHwAb` PWM/DIO → plant integration → ADC feedback →
//! diagnostics (DEM/NvM) → CAN status frame 675.
//!
//! # Modules
//!
//! | Module | AUTOSAR counterpart |
//! |---|---|
//! | [`sched`] | OS schedule table with cyclic activation events (OSEK-like) |
//! | [`com`] | COM stack: DBC signal pack/decode, alive supervision |
//! | [`iohwab`] | `IoHwAb`: PWM/DIO outputs, sampled ADC feedback |
//! | [`dem`] | DEM (diagnostic events) + DET (development errors) |
//! | [`nvm`] | `NvM` job-queue RAM mirror (no flash persistence) |
//! | [`ecu`] | RTE wiring + bus/sensor nodes + [`Ecu`] integration |
//!
//! Determinism: everything runs on the scheduler's virtual time; the whole
//! simulation is reproducible tick-for-tick (REQ-WIP-019). Same input history
//! ⇒ same output history — the base for the golden/scenario tests of M5.
//!
//! Mapping documentation: `docs/bsw-sim.md` (AUTOSAR→COM signal table, schedule,
//! timeouts). Component limitations: `docs/limitations.md`.

#![forbid(unsafe_code)]

pub mod com;
pub mod dem;
pub mod ecu;
pub mod iohwab;
pub mod nvm;
pub mod sched;

pub use com::{CanFrame, Com, ComEvent, ComPorts, RainRx, SpeedRx, StalkRx, StatusSignals};
pub use dem::{Dem, DemEvent, Det};
pub use ecu::{CanBus, Ecu, RainTx, SensorNodes, SpeedTx, StalkTx, Task};
pub use iohwab::IoHwAb;
pub use nvm::{NvmImage, NvmStub};
pub use sched::{SchedError, ScheduledTask, Scheduler, TaskId};

/// Crate version (of this BSW simulation layer).
pub const BSW_SIM_VERSION: &str = env!("CARGO_PKG_VERSION");
