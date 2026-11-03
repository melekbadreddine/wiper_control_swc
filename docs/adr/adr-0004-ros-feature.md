# ADR-0004 — ROS independence of `ros2-nodes` (default build compiles out ROS)

* Status: **Accepted** (milestone M0; implementation validated in M7)
* Date: 2026-11-03

## Context
The wiper logic must be provably ROS-independent (hexagonal architecture), and the
native toolchain must work without ROS installed; on the other hand the same code must
build inside the ROS 2 docker image.

## Decision
- The whole workspace builds without a ROS installation by default.
- `crates/ros2-nodes` compiles ROS-free by default; the real node implementation is
  behind the `ros` cargo feature which enables the ROS client dependency (`r2r`) and is
  built into binaries only inside the ROS 2 image / CI ROS job.

## Consequences
Rust adapters are pure data shims; a single cargo feature toggles the target. CI runs
the feature-on build only in the ROS 2 docker pipeline (M9). This decision relies on a
`r2r` spike in M7; if `r2r` proves fragile, `ros2-client`/`rclrs` is the fallback and
the spike outcome must be recorded here (revision bump).
