//! DEM — diagnostic event manager (simplified, per REQ-WIP-017).
//!
//! Stores one [`DemEvent`] per monitored fault identifier with the classic
//! DEM status semantics used by this project: *active* (fault currently
//! present), *latched* (has ever fired until diagnostic clear) and occurrence
//! count (incremented on every rising edge). Events are reported here by the
//! 100 ms diagnostic task from the SWC's `DiagStatus` snapshot.
//!
//! `DET` is included as the development-error counter for protocol violations
//! caught by the simulated BSW (COM rejects, unexpected frames).

#![forbid(unsafe_code)]

use wiper_swc::Dtc;

/// One diagnostic event in the DEM store.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DemEvent {
    /// Fault identifier.
    pub dtc: Dtc,
    /// Fault is present in the current cycle.
    pub active: bool,
    /// Fault fired at least once since the last `clear` (no auto-clear).
    pub latched: bool,
    /// Number of detected rising edges since `clear`.
    pub occurrences: u32,
    /// First detection time (ms).
    pub first_ms: u32,
    /// Latest report time (ms).
    pub last_ms: u32,
}

/// Diagnostic event manager.
#[derive(Debug, Default)]
pub struct Dem {
    events: Vec<DemEvent>,
}

impl Dem {
    /// Empty store.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Report the presence of a fault in one diagnostic cycle (each 100 ms).
    pub fn report(&mut self, dtc: Dtc, active: bool, now_ms: u32) {
        if let Some(ev) = self.events.iter_mut().find(|e| e.dtc == dtc) {
            if active && !ev.active {
                ev.occurrences += 1;
            }
            ev.active = active;
            ev.latched |= active;
            ev.last_ms = now_ms;
            return;
        }
        self.events.push(DemEvent {
            dtc,
            active,
            latched: active,
            occurrences: u32::from(active),
            first_ms: now_ms,
            last_ms: now_ms,
        });
    }

    /// Latest event for `dtc`, `None` if never reported.
    #[must_use]
    pub fn event(&self, dtc: Dtc) -> Option<&DemEvent> {
        self.events.iter().find(|e| e.dtc == dtc)
    }

    /// Is the fault active in the latest cycle it was reported?
    #[must_use]
    pub fn is_active(&self, dtc: Dtc) -> bool {
        self.event(dtc).is_some_and(|e| e.active)
    }

    /// Is the fault present in the current cycle?
    #[must_use]
    pub fn is_latched(&self, dtc: Dtc) -> bool {
        self.event(dtc).is_some_and(|e| e.latched)
    }

    /// Diagnostic clear (UDS service 0x14 semantics): the whole store is reset;
    /// occurrence counters and latch flags restart empty.
    pub fn clear(&mut self) {
        self.events.clear();
    }

    /// All events currently stored.
    #[must_use]
    pub fn events(&self) -> &[DemEvent] {
        &self.events
    }

    /// Number of stored events.
    #[must_use]
    pub fn len(&self) -> usize {
        self.events.len()
    }

    /// No events stored?
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.events.is_empty()
    }
}

/// Development error tracker (DET): counts BSW-internal protocol violations
/// so tests and scenarios can prove the simulated stack stayed consistent.
#[derive(Debug, Default, Clone, Copy)]
pub struct Det {
    violations: u32,
}

impl Det {
    /// Zero violations.
    #[must_use]
    pub const fn new() -> Self {
        Self { violations: 0 }
    }

    /// Record one violation.
    pub fn report(&mut self) {
        self.violations += 1;
    }

    /// Total violations seen so far.
    #[must_use]
    pub const fn violations(&self) -> u32 {
        self.violations
    }
}

#[cfg(test)]
mod tests {
    use super::{Dem, Det};
    use wiper_swc::Dtc;

    /// verifies: REQ-WIP-017 — DEM stores active/latched status and rising-edge
    /// occurrence counts per DTC, and diagnostic clear resets the store.
    #[test]
    fn dem_report_semantics_and_clear() {
        let mut dem = Dem::new();
        dem.report(Dtc::MotorStall, true, 100);
        dem.report(Dtc::MotorStall, true, 200); // still active, no new occurrence
        dem.report(Dtc::MotorStall, false, 300); // recovered
        dem.report(Dtc::MotorStall, true, 400); // second occurrence

        let ev = *dem.event(Dtc::MotorStall).unwrap_or({
            &super::DemEvent {
                dtc: Dtc::MotorStall,
                active: false,
                latched: false,
                occurrences: 0,
                first_ms: 0,
                last_ms: 0,
            }
        });
        assert!(ev.active);
        assert!(ev.latched);
        assert_eq!(ev.occurrences, 2);
        assert_eq!(ev.first_ms, 100);
        assert_eq!(ev.last_ms, 400);
        assert!(dem.is_active(Dtc::MotorStall));
        assert!(dem.is_latched(Dtc::MotorStall));

        assert!(dem.event(Dtc::Undervoltage).is_none());
        assert_eq!(dem.len(), 1);

        dem.clear();
        assert!(dem.is_empty());
        assert!(!dem.is_latched(Dtc::MotorStall));
        dem.report(Dtc::MotorStall, false, 500);
        assert_eq!(dem.event(Dtc::MotorStall).map_or(0, |e| e.occurrences), 0);
        assert_eq!(dem.event(Dtc::MotorStall).map_or(0, |e| e.first_ms), 500);
    }

    #[test]
    fn det_counts_violations() {
        let mut det = Det::new();
        assert_eq!(det.violations(), 0);
        det.report();
        det.report();
        assert_eq!(det.violations(), 2);
    }
}
