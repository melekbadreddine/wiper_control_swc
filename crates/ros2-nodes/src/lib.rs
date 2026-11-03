//! ROS 2 adapter nodes wrapping the ROS-independent wiper SWC and plant.
//!
//! Per ADR-0004: the real node implementation (using the `r2r` client library) is
//! gated behind the `ros` cargo feature and is built inside the ROS 2 Jazzy Docker
//! image. Without the feature this crate compiles ROS-free, so the whole workspace
//! unit-tests natively without ROS installed. The default build is the M0 bootstrap
//! placeholder; the real implementation lands in milestone M7 (`PROGRESS.md`).
//!
//! The SWC itself (`wiper-swc`) never depends on this crate — nodes are adapters
//! on its edge (hexagonal architecture). A test in M7 proves the same SWC logic runs
//! here and in the headless scenario runner.

#![forbid(unsafe_code)]

/// Placeholder used by the M0 bootstrap gate only.
pub const ADAPTER_STATUS: &str = "ros feature not enabled: adapters compiled out";

#[cfg(test)]
mod tests {
    use super::ADAPTER_STATUS;

    #[test]
    fn default_build_compiles_without_ros() {
        assert!(!ADAPTER_STATUS.is_empty());
    }
}
