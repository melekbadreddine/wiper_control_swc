//! Windshield wiper control **software component** (AUTOSAR Classic conventions).
//!
//! Pure application logic in Rust: stalk input handling, wiper state machine
//! (OFF / INT / LOW / HIGH / wash sequence), intermittent timing with rain-sensor
//! auto mode and vehicle-speed dependent pause, park-position management, and
//! diagnostics (stall detection, park-switch plausibility, voltage supervision).
//!
//! Design rules (see `AGENTS.md`):
//! - `no_std`, no heap, `forbid(unsafe_code)`, no `unwrap/expect/panic/indexing`.
//! - Fixed-point arithmetic where floats are unnecessary (documented per site).
//! - AUTOSAR-style runnable naming: `Wiper_Init`, `Wiper_Run10ms`, `Wiper_Run100ms`,
//!   `Wiper_OnStalkChange`.
//! - Knows nothing about ROS, CAN, files, or the host OS.
//!
//! The full state machine is implemented in milestone M3 (see `PROGRESS.md`).

#![no_std]
#![forbid(unsafe_code)]

/// Fixed single-line scratch buffer for diagnostics text formatting without heap.
pub struct DebugBuf {
    buf: [u8; 64],
    len: usize,
}

impl DebugBuf {
    /// Create an empty buffer.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            buf: [0; 64],
            len: 0,
        }
    }

    /// Bytes written so far. Returns an empty string if the buffer is (somehow) not
    /// valid UTF-8, never panics.
    #[must_use]
    pub fn as_str(&self) -> &str {
        // No unsafe and no panic: `get(..len)` bounds-checks the slice, `from_utf8`
        // rejects nonsense, `unwrap_or_default` keeps the pedantic clippy lint happy.
        self.buf
            .get(..self.len)
            .and_then(|s| core::str::from_utf8(s).ok())
            .unwrap_or_default()
    }
}

impl Default for DebugBuf {
    fn default() -> Self {
        Self::new()
    }
}

impl core::fmt::Write for DebugBuf {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        if let Some(dst) = self.buf.get_mut(self.len..self.len + s.len()) {
            dst.copy_from_slice(s.as_bytes());
            self.len += s.len();
            Ok(())
        } else {
            Err(core::fmt::Error)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::fmt::Write;

    #[test]
    fn debug_buf_fits_and_truncates_safely() {
        let mut b = DebugBuf::new();
        assert!(b.write_str("Wiper_Run10ms").is_ok());
        assert_eq!(b.as_str(), "Wiper_Run10ms");
        assert!(b.write_str("x".repeat(65).as_str()).is_err());
        assert_eq!(b.as_str(), "Wiper_Run10ms");
    }
}
