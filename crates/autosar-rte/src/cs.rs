//! Client/Server ports. In this deterministic simulation the port holds one
//! pending request and one response slot (single server, single client by design).
//!
//! `#![allow(non_snake_case)]`: the method names deliberately mirror the AUTOSAR
//! RTE API (`Rte_Call_<op>` / `Rte_Result_<op>`), which is CamelCase by convention.

#![allow(non_snake_case)]

use core::cell::RefCell;

/// Client/server pair for one operation with identical request/response type.
/// (Multiple operations are modelled by using an operation enum as the type.)
pub struct ClientServerPort<T> {
    pending: RefCell<Option<T>>,
    response: RefCell<Option<T>>,
}

impl<T> ClientServerPort<T> {
    /// Empty port.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            pending: RefCell::new(None),
            response: RefCell::new(None),
        }
    }
}

impl<T> Default for ClientServerPort<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Copy> ClientServerPort<T> {
    /// Client side: `Rte_Call_<op>` — queues the request (one slot; fails if busy).
    ///
    /// # Errors
    /// `InUseError` when the single request slot is occupied (previous call not yet
    /// served) or when the slot cannot be allocated in this deterministic sim.
    pub fn Rte_Call(&self, request: T) -> Result<(), InUseError> {
        if let Ok(mut slot) = self.pending.try_borrow_mut() {
            if slot.is_some() {
                return Err(InUseError);
            }
            *slot = Some(request);
            Ok(())
        } else {
            Err(InUseError)
        }
    }

    /// Server side: `Rte_CDataInvocableEntryPoint`-style poll of the pending call.
    /// For polling servers: return-and-clear semantics.
    #[must_use]
    pub fn Rte_PollInvocations(&self) -> Option<T> {
        self.pending.try_borrow_mut().ok()?.take()
    }

    /// Server side: `Rte_Result_<op>` — post the response for the served request.
    ///
    /// # Errors
    /// `InUseError` when the response slot is busy or unavailable.
    pub fn Rte_Result(&self, response: T) -> Result<(), InUseError> {
        if let Ok(mut slot) = self.response.try_borrow_mut() {
            *slot = Some(response);
            Ok(())
        } else {
            Err(InUseError)
        }
    }

    /// Client side: `Rte_ResultPoll` — fetch the response if the server delivered;
    /// does not block, mirroring the simulated two-step server.
    #[must_use]
    pub fn Rte_PollResult(&self) -> Option<T> {
        self.response.try_borrow_mut().ok()?.take()
    }
}

/// Request slot already occupied / response slot busy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InUseError;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn call_result_roundtrip() {
        let p: ClientServerPort<u16> = ClientServerPort::new();
        assert!(p.Rte_Call(4).is_ok());
        assert_eq!(p.Rte_Call(5), Err(InUseError));
        assert_eq!(p.Rte_PollInvocations(), Some(4));
        assert_eq!(p.Rte_PollInvocations(), None);
        assert!(p.Rte_Result(16).is_ok());
        assert_eq!(p.Rte_PollResult(), Some(16));
        assert_eq!(p.Rte_PollResult(), None);
        assert!(p.Rte_Call(7).is_ok());
    }
}
