//! `NvM` stub — RAM-mirror block store (job queue, like the real `NvM` service).
//!
//! Scope (see `docs/limitations.md`): the mirror is plain RAM; there is **no**
//! persistence across power cycles — declared ECU power-down would be needed
//! for flash-backed behaviour and is out of M4 scope. The ECU uses it to keep
//! the diagnostic latch bits and the wipe counter available for the 100 ms
//! task, exercising the `NvM` job/ready pattern.

#![forbid(unsafe_code)]

/// Image of the single simulated `NvM` block (Wiper diagnostics + telemetry).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct NvmImage {
    /// Latched DTC bitset (same bit order as `wiper_swc::DiagStatus`).
    pub dtc_latched_bits: u16,
    /// Completed wipe cycles since power-on.
    pub wipe_count: u32,
}

/// RAM-mirrored `NvM` with an asynchronous write-job queue.
#[derive(Debug, Default)]
pub struct NvmStub {
    mirror: Option<NvmImage>,
    pending: Option<NvmImage>,
    jobs_done: u32,
}

impl NvmStub {
    /// Empty mirror (block invalid until the first completed write job).
    #[must_use]
    pub const fn new() -> Self {
        Self {
            mirror: None,
            pending: None,
            jobs_done: 0,
        }
    }

    /// Queue a write job; the image becomes durable after `process_jobs`.
    pub fn request_write(&mut self, image: NvmImage) {
        self.pending = Some(image);
    }

    /// Complete all queued write jobs; returns how many jobs finished.
    pub fn process_jobs(&mut self) -> u32 {
        let mut n = 0;
        if let Some(image) = self.pending.take() {
            self.mirror = Some(image);
            n = 1;
            self.jobs_done += 1;
        }
        n
    }

    /// Current mirror image if at least one write completed.
    #[must_use]
    pub const fn read(&self) -> Option<&NvmImage> {
        self.mirror.as_ref()
    }

    /// A write job is queued but not yet processed.
    #[must_use]
    pub const fn has_pending(&self) -> bool {
        self.pending.is_some()
    }

    /// Total completed write jobs.
    #[must_use]
    pub const fn jobs_done(&self) -> u32 {
        self.jobs_done
    }
}

#[cfg(test)]
mod tests {
    use super::{NvmImage, NvmStub};

    /// `NvM` is a job-queue RAM mirror: invalid until the first write job
    /// completes, then it serves the last processed image.
    #[test]
    fn mirror_follows_processed_jobs() {
        let mut nvm = NvmStub::new();
        assert!(nvm.read().is_none(), "nothing written yet");
        nvm.request_write(NvmImage {
            dtc_latched_bits: 0b1,
            wipe_count: 5,
        });
        assert!(nvm.has_pending());
        assert!(nvm.read().is_none(), "job not processed yet");
        assert_eq!(nvm.process_jobs(), 1);
        assert!(!nvm.has_pending());
        assert_eq!(
            nvm.read().copied(),
            Some(NvmImage {
                dtc_latched_bits: 1,
                wipe_count: 5
            })
        );
        assert_eq!(nvm.jobs_done(), 1);

        nvm.request_write(NvmImage {
            dtc_latched_bits: 0,
            wipe_count: 6,
        });
        nvm.process_jobs();
        assert_eq!(
            nvm.read().copied(),
            Some(NvmImage {
                dtc_latched_bits: 0,
                wipe_count: 6
            })
        );
        assert_eq!(nvm.jobs_done(), 2);
        assert!(!nvm.has_pending());
        assert_eq!(nvm.process_jobs(), 0, "no job queued");
    }
}
