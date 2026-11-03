//! Sender/Receiver ports. AUTOSAR non-queued ports are last-is-best
//! (implicit/explicit communication); queued ports keep a FIFO of samples.
//!
//! `#![allow(non_snake_case)]`: methods deliberately mirror AUTOSAR RTE names
//! (`Rte_Write_`/`Rte_Read_`).

#![allow(non_snake_case)]

use core::cell::RefCell;

/// AUTOSAR data quality of a received data element.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Quality {
    /// Sent with `Valid` flag (normal operation).
    Valid,
    /// Sender invalidated data (e.g. sensor fault reported).
    Invalid,
}

/// Result status of a non-queued read, mirroring the RTE `DataStatus`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReadStatus {
    /// New/unchanged value available and valid.
    Valid,
    /// Value carried an invalid flag (symbolic invalidation).
    Invalid,
    /// No new value within the configured timeout (stale / alive supervision trip).
    Timeout,
    /// Nothing was ever written (before first message).
    NoData,
}

/// The value plus its status, from [`RteReceiver::Rte_Read`].
#[derive(Debug)]
pub struct Received<T> {
    /// Last stored value (`Some` after the first message).
    pub value: Option<T>,
    /// Freshness/validity status.
    pub status: ReadStatus,
    /// Age of the stored value in ms at the requested instant.
    pub age_ms: u32,
}

#[derive(Debug)]
struct Stored<T> {
    value: Option<T>,
    quality: Quality,
    stamp_ms: Option<u32>,
}

/// Shared non-queued buffer for one data element. A sender/receiver pair reference
/// the same `NonQueuedPort`; the RTE in a real system generates these bindings.
pub struct NonQueuedPort<T> {
    stored: RefCell<Stored<T>>,
}

impl<T> NonQueuedPort<T> {
    /// Create the port with an optional init value (AUTOSAR init-value). A provided
    /// init value counts as valid data; `None` starts with no data at all.
    #[must_use]
    pub const fn new(init: Option<T>) -> Self {
        let quality = if init.is_some() {
            Quality::Valid
        } else {
            Quality::Invalid
        };
        Self {
            stored: RefCell::new(Stored {
                value: init,
                quality,
                stamp_ms: None,
            }),
        }
    }
}

/// Write-side handle (application side of `Rte_Write_<port>_<elem>`).
pub struct RteSender<'a, T> {
    port: &'a NonQueuedPort<T>,
}

impl<'a, T> RteSender<'a, T> {
    /// Bind to a port.
    #[must_use]
    pub const fn new(port: &'a NonQueuedPort<T>) -> Self {
        Self { port }
    }
}

impl<T: Copy> RteSender<'_, T> {
    /// Last-is-best write; `now_ms` is the dimension of the sample.
    pub fn Rte_Write(&self, value: T, now_ms: u32) {
        if let Ok(mut s) = self.port.stored.try_borrow_mut() {
            s.value = Some(value);
            s.quality = Quality::Valid;
            s.stamp_ms = Some(now_ms);
        }
    }

    /// Symbolic invalidation (AUTOSAR invalidation policy).
    pub fn Rte_Invalidate(&self) {
        if let Ok(mut s) = self.port.stored.try_borrow_mut() {
            s.quality = Quality::Invalid;
        }
    }
}

/// Read-side handle with optional staleness supervision.
pub struct RteReceiver<'a, T> {
    port: &'a NonQueuedPort<T>,
    timeout_ms: Option<u32>,
}

impl<'a, T> RteReceiver<'a, T> {
    /// Bind; `timeout_ms` enables alive supervision (`None` = never stale).
    #[must_use]
    pub const fn new(port: &'a NonQueuedPort<T>, timeout_ms: Option<u32>) -> Self {
        Self { port, timeout_ms }
    }
}

impl<T: Copy> RteReceiver<'_, T> {
    /// Last-is-best read; statuses per AUTOSAR semantics (invalid/timeout honoured),
    /// last value still returned so the application can implement fallbacks.
    #[must_use]
    pub fn Rte_Read(&self, now_ms: u32) -> Received<T> {
        if let Ok(s) = self.port.stored.try_borrow() {
            let age = match s.stamp_ms {
                Some(stamp) => now_ms.saturating_sub(stamp),
                // No stamp (init value / never written): age is meaningless → 0.
                None => 0,
            };
            let stale = self
                .timeout_ms
                .is_some_and(|t| s.stamp_ms.is_some() && age > t);
            let status = if s.value.is_none() {
                ReadStatus::NoData
            } else if s.quality == Quality::Invalid {
                ReadStatus::Invalid
            } else if stale {
                ReadStatus::Timeout
            } else {
                ReadStatus::Valid
            };
            Received {
                value: s.value,
                status,
                age_ms: age,
            }
        } else {
            Received {
                value: None,
                status: ReadStatus::NoData,
                age_ms: 0,
            }
        }
    }
}

/// Queued (FIFO) port with a compile-time capacity ring buffer.
pub struct QueuedPort<T, const CAP: usize> {
    ring: RefCell<Ring<T, CAP>>,
}

#[derive(Debug)]
struct Ring<T, const CAP: usize> {
    buf: [Option<T>; CAP],
    head: usize,
    len: usize,
}

impl<T: Copy, const CAP: usize> QueuedPort<T, CAP> {
    /// Empty queue.
    #[must_use]
    pub fn new() -> Self {
        Self {
            ring: RefCell::new(Ring {
                buf: [const { None }; CAP],
                head: 0,
                len: 0,
            }),
        }
    }
}

impl<T: Copy, const CAP: usize> Default for QueuedPort<T, CAP> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Copy, const CAP: usize> QueuedPort<T, CAP> {
    /// Sender: enqueue; `Ok(())`, or the value is handed back when the queue is full
    /// (AUTOSAR queue-full handling is on the sender's shoulders).
    ///
    /// # Errors
    /// The same `value` is returned when the ring capacity is exhausted or the port
    /// is briefly unborrowable.
    pub fn send(&self, value: T) -> Result<(), T> {
        if let Ok(mut r) = self.ring.try_borrow_mut() {
            if r.len == CAP {
                return Err(value);
            }
            let idx = (r.head + r.len) % CAP;
            if let Some(slot) = r.buf.get_mut(idx) {
                *slot = Some(value);
            }
            r.len += 1;
            Ok(())
        } else {
            Err(value)
        }
    }

    /// Receiver: dequeue; `None` when empty.
    #[must_use]
    pub fn receive(&self) -> Option<T> {
        if let Ok(mut r) = self.ring.try_borrow_mut() {
            if r.len == 0 {
                return None;
            }
            let idx = r.head;
            r.head = (r.head + 1) % CAP;
            r.len -= 1;
            if let Some(slot) = r.buf.get_mut(idx) {
                slot.take()
            } else {
                None
            }
        } else {
            None
        }
    }

    /// Number of queued samples.
    #[must_use]
    pub fn len(&self) -> usize {
        self.ring.borrow().len
    }

    /// Queue empty?
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.ring.borrow().len == 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nonqueued_last_is_best() {
        let p: NonQueuedPort<u8> = NonQueuedPort::new(None);
        let tx = RteSender::new(&p);
        let rx: RteReceiver<u8> = RteReceiver::new(&p, Some(10));

        let r = rx.Rte_Read(0);
        assert_eq!(r.status, ReadStatus::NoData);
        assert!(r.value.is_none());

        tx.Rte_Write(5, 2);
        let r = rx.Rte_Read(3);
        assert_eq!(r.status, ReadStatus::Valid);
        assert_eq!(r.value, Some(5));
        assert_eq!(r.age_ms, 1);
    }

    #[test]
    fn nonqueued_init_value_is_valid_without_stamp() {
        let p: NonQueuedPort<u16> = NonQueuedPort::new(Some(7));
        let _tx = RteSender::new(&p);
        let rx: RteReceiver<u16> = RteReceiver::new(&p, Some(10));
        // Init value counts as valid data; no stamp yet -> age 0, never stale.
        let r = rx.Rte_Read(100);
        assert_eq!(r.status, ReadStatus::Valid);
        assert_eq!(r.value, Some(7));
        assert_eq!(r.age_ms, 0);
    }

    #[test]
    fn nonqueued_timeout_hits() {
        let p: NonQueuedPort<u8> = NonQueuedPort::new(None);
        let tx = RteSender::new(&p);
        let rx: RteReceiver<u8> = RteReceiver::new(&p, Some(10));
        tx.Rte_Write(5, 0);
        // Fresh within timeout.
        assert_eq!(rx.Rte_Read(9).status, ReadStatus::Valid);
        // Beyond the 10 ms window: timeout, last value still delivered.
        let r = rx.Rte_Read(11);
        assert_eq!(r.status, ReadStatus::Timeout);
        assert_eq!(r.value, Some(5));
    }

    #[test]
    fn nonqueued_invalidate() {
        let p: NonQueuedPort<u8> = NonQueuedPort::new(None);
        let tx = RteSender::new(&p);
        let rx: RteReceiver<u8> = RteReceiver::new(&p, None);
        tx.Rte_Write(3, 1);
        tx.Rte_Invalidate();
        let r = rx.Rte_Read(2);
        assert_eq!(r.status, ReadStatus::Invalid);
        // Sender can recover the data path.
        tx.Rte_Write(4, 4);
        let r = rx.Rte_Read(6);
        assert_eq!(r.status, ReadStatus::Valid);
        assert_eq!(r.value, Some(4));
    }

    #[test]
    fn queued_fifo_and_full() {
        let p: QueuedPort<u8, 2> = QueuedPort::new();
        assert!(p.send(1).is_ok());
        assert!(p.send(2).is_ok());
        assert_eq!(p.send(3), Err(3));
        assert_eq!(p.receive(), Some(1));
        assert_eq!(p.receive(), Some(2));
        assert_eq!(p.receive(), None);
        assert!(p.is_empty());
    }

    #[test]
    fn queued_wraparound() {
        let p: QueuedPort<u8, 2> = QueuedPort::new();
        let _ = p.send(1);
        assert_eq!(p.receive(), Some(1));
        assert!(p.send(2).is_ok());
        assert!(p.send(3).is_ok());
        assert_eq!(p.receive(), Some(2));
        assert!(p.send(4).is_ok());
        assert_eq!(p.receive(), Some(3));
        assert_eq!(p.receive(), Some(4));
        assert_eq!(p.len(), 0);
    }
}
