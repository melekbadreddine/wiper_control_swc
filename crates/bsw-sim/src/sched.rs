//! OSEK-like cyclic scheduler with deterministic virtual time.
//!
//! The simulated OS advances time by a fixed base tick (10 ms). Every
//! registered task has a period that is an exact multiple of the base tick;
//! a task is activated when `t_ms % period == 0`. This mirrors an OSEK/AUTOSAR
//! OS schedule table with cyclic activation events: no interarrival jitter, no
//! wall-clock reads, fully reproducible activation sequences (REQ-WIP-019/020).

#![forbid(unsafe_code)]

use core::fmt::Debug;

/// Anything usable as a task identifier: plain `Copy + Eq + Debug` enum.
pub trait TaskId: Copy + Eq + Debug {}

/// Period must be nonzero and a multiple of the base tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SchedError {
    /// `period_ms` is zero.
    ZeroPeriod,
    /// `period_ms` is not a multiple of the configured base tick.
    NotTickAligned,
}

impl core::fmt::Display for SchedError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Self::ZeroPeriod => "task period must be nonzero",
            Self::NotTickAligned => "task period must be a multiple of the base tick",
        })
    }
}

/// One registered task: identifier, OSEK-style symbol and cyclic period.
#[derive(Debug, Clone, Copy)]
pub struct ScheduledTask<T: TaskId> {
    /// Task identifier (user enum value).
    pub id: T,
    /// Runnable/task name as it appears in the schedule table.
    pub name: &'static str,
    /// Activation period in ms (multiple of the base tick).
    pub period_ms: u32,
}

/// Cyclic scheduler. `.tick()` advances one base tick and returns the
/// activation list (registration order) for the new time point.
#[derive(Debug)]
pub struct Scheduler<T: TaskId> {
    tick_ms: u32,
    t_ms: u32,
    tasks: Vec<ScheduledTask<T>>,
    activations: Vec<u32>,
}

impl<T: TaskId> Scheduler<T> {
    /// Scheduler at t = 0 with the given base tick granularity.
    #[must_use]
    pub fn new(tick_ms: u32) -> Self {
        Self {
            tick_ms,
            t_ms: 0,
            tasks: Vec::new(),
            activations: Vec::new(),
        }
    }

    /// Register a cyclic task. Period must be an exact multiple of the tick.
    ///
    /// # Errors
    /// [`SchedError::ZeroPeriod`] / [`SchedError::NotTickAligned`] as documented.
    pub fn register(
        &mut self,
        id: T,
        name: &'static str,
        period_ms: u32,
    ) -> Result<(), SchedError> {
        if period_ms == 0 {
            return Err(SchedError::ZeroPeriod);
        }
        if period_ms % self.tick_ms != 0 {
            return Err(SchedError::NotTickAligned);
        }
        self.tasks.push(ScheduledTask {
            id,
            name,
            period_ms,
        });
        self.activations.push(0);
        Ok(())
    }

    /// Advance one base tick and return the ids of the due tasks.
    pub fn tick(&mut self) -> Vec<T> {
        self.t_ms += self.tick_ms;
        let mut due = Vec::new();
        for (i, task) in self.tasks.iter().enumerate() {
            if self.t_ms % task.period_ms == 0 {
                if let Some(count) = self.activations.get_mut(i) {
                    *count += 1;
                }
                due.push(task.id);
            }
        }
        due
    }

    /// Current virtual time.
    #[must_use]
    pub const fn now_ms(&self) -> u32 {
        self.t_ms
    }

    /// Number of activations recorded per task id (0 when unknown).
    #[must_use]
    pub fn activations(&self, id: T) -> u32 {
        self.tasks
            .iter()
            .position(|t| t.id == id)
            .and_then(|i| self.activations.get(i))
            .copied()
            .unwrap_or(0)
    }

    /// Registered task table.
    #[must_use]
    pub fn schedule(&self) -> &[ScheduledTask<T>] {
        &self.tasks
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum T {
        A,
        B,
    }

    impl TaskId for T {}

    /// verifies: REQ-WIP-020 — the 10 ms task is activated at exactly 10 ms
    /// cadence and the 100 ms task at exactly every 10th tick (OS-side half of
    /// the timing budget; the WCET measurement is a M5 criterion benchmark).
    #[test]
    fn os_cyclic_schedule() {
        let mut sched = Scheduler::new(10);
        let _ = sched.register(T::A, "Wiper_Run10ms", 10);
        let _ = sched.register(T::B, "Wiper_Run100ms", 100);
        for _ in 0..1_000 {
            sched.tick();
        }
        assert_eq!(sched.activations(T::A), 1_000);
        assert_eq!(sched.activations(T::B), 100);
    }

    #[test]
    fn activations_are_exactly_on_period_boundaries() {
        let mut sched = Scheduler::new(10);
        let _ = sched.register(T::A, "A", 10);
        let _ = sched.register(T::B, "B", 100);
        let mut last_a = 0;
        let mut b_times = Vec::new();
        loop {
            let due = sched.tick();
            let now = sched.now_ms();
            if due.contains(&T::A) {
                assert_eq!(now - last_a, 10);
                last_a = now;
            }
            if due.contains(&T::B) {
                b_times.push(now);
            }
            if now >= 300 {
                break;
            }
        }
        assert_eq!(b_times, vec![100, 200, 300]);
    }

    #[test]
    fn identical_sequences_are_reproducible() {
        let build = |periods| -> (Vec<Vec<T>>, u32) {
            let mut s = Scheduler::new(10);
            for (id, p, name) in periods {
                let _ = s.register(id, name, p);
            }
            let mut seen = Vec::new();
            for _ in 0..250 {
                seen.push(s.tick());
            }
            (seen, s.now_ms())
        };
        let a = build([(T::A, 10, "A"), (T::B, 100, "B")]);
        let b = build([(T::A, 10, "A"), (T::B, 100, "B")]);
        assert_eq!(a, b);
    }

    #[test]
    fn period_must_be_tick_aligned() {
        let mut sched = Scheduler::new(10);
        assert_eq!(
            sched.register(T::A, "A", 15),
            Err(SchedError::NotTickAligned)
        );
        assert_eq!(sched.register(T::A, "A", 0), Err(SchedError::ZeroPeriod));
        assert!(sched.register(T::A, "A", 10).is_ok());
        assert_eq!(sched.schedule().len(), 1);
    }
}
