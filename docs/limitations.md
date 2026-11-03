# Limitations (out of scope)

Kept honest and explicit — none of these are silently faked:

* No real hardware, no Flash/MCU build target in CI (Rust for e.g. Cortex-M targets is
  possible for `wiper-swc`/`autosar-rte` but untested here).
* ASIL ratings are assumed (see `hara.md`) — no formal ISO 26262 assessment.
* The plant model is a first-principles simplified model; parameters are academic
  estimates, not calibrated from a real vehicle.
* ARXML generation covers the SWC description + composition level used for
  consistency checking, not a full OEM ECU-Extract with all BSW modules configured.
* ROS 2 integration simulates the vehicle network in-message; no CAN bus hardware.
* MCU/PIL level, OTA, cybersecurity hardening, ComM/CanSM state machines: out of scope.
