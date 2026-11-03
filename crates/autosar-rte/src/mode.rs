//! Mode switch abstraction (AUTOSAR Mode Management, simulated).
//!
//! One mode manager writes the current mode; any number of mode users read it.
//! The simulation uses *immediate switch* semantics (latched latest mode), which
//! matches the RTE's "mode switch port group" behaviour for non-timestamped modes.
//!
//! `#![allow(non_snake_case)]`: methods mirror AUTOSAR RTE mode API names.

#![allow(non_snake_case)]

use core::cell::Cell;

/// Mode group port shared between manager and users.
pub struct ModeSwitchPort<T> {
    current: Cell<Option<T>>,
}

impl<T> ModeSwitchPort<T> {
    /// Port before first switch (mode users see `ModeValue::NotYetAvailable`).
    #[must_use]
    pub const fn new() -> Self {
        Self {
            current: Cell::new(None),
        }
    }
}

impl<T> Default for ModeSwitchPort<T> {
    fn default() -> Self {
        Self::new()
    }
}

/// What a mode user's `Rte_Mode` read returned.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModeValue<M> {
    /// Current mode (switch complete in this simulation).
    Mode(M),
    /// No mode has been declared yet since power-up.
    NotYetAvailable,
}

impl<T: Copy> ModeSwitchPort<T> {
    /// Mode manager side: `Rte_Switch_<group>` — declare a new current mode.
    pub fn Rte_Switch(&self, mode: T) {
        self.current.set(Some(mode));
    }

    /// Mode user side: `Rte_Mode_<group>` — read the active mode.
    #[must_use]
    pub fn Rte_Mode(&self) -> ModeValue<T> {
        match self.current.get() {
            Some(m) => ModeValue::Mode(m),
            None => ModeValue::NotYetAvailable,
        }
    }
}

/// Vehicle ignition modes (used by the wiper SWC and the ROS layer).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IgnitionMode {
    /// Ignition off.
    Off,
    /// Accessory position.
    Acc,
    /// Running.
    On,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn switch_before_users_read() {
        let p: ModeSwitchPort<IgnitionMode> = ModeSwitchPort::new();
        assert_eq!(p.Rte_Mode(), ModeValue::NotYetAvailable);
        p.Rte_Switch(IgnitionMode::On);
        assert_eq!(p.Rte_Mode(), ModeValue::Mode(IgnitionMode::On));
        p.Rte_Switch(IgnitionMode::Acc);
        assert_eq!(p.Rte_Mode(), ModeValue::Mode(IgnitionMode::Acc));
    }
}
