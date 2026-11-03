//! Runner and trigger abstractions (runnable entities).
//!
//! An AUTOSAR *runnable entity* is a schedulable function activated by events. Here
//! the activation model is: [`Trigger`] enumerates the event discipline and
//! [`Runnable`] exposes name/trigger/actions so the simulated OS (`bsw-sim`) can
//! schedule it deterministically and the ARXML generator (M6) can render the same
//! declarations as AUTOSAR XML.

/// What activates a runnable, mirroring AUTOSAR timing events.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Trigger {
    /// `RTE_EVENT_INIT` — executed once before cyclic operation.
    Init,
    /// `RTE_EVENT_CYCLIC` with the period in ms (e.g. 10 for `Wiper_Run10ms`).
    Cyclic(u32),
    /// `RTE_EVENT_DATA_RECEIVED` on the named sender/receiver port.
    DataReceived(&'static str),
}

/// A runnable entity: activation info + the action to execute.
pub trait Runnable {
    /// AUTOSAR-style name of the runnable, e.g. `Wiper_Run10ms`.
    fn name(&self) -> &'static str;
    /// Activation event.
    fn trigger(&self) -> Trigger;
    /// Execute (one job). SWC runnables read/write their RTE ports internally.
    fn run(&mut self);
}
