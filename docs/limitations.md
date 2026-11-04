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
* `bsw-sim` NvM is a RAM-mirrored job-queue stub (`nvm::NvmStub`): the mirror never
  survives a power cycle; flash-backed persistence is not simulated.
* The virtual CAN bus has no arbitration, bit timing, or transport protocols:
  it stores the latest frame per identifier with receive-once delivery. Nodes are
  modeled as perfect periodic senders; offline faults are injected as period 0.
* The washer pump is a binary DIO indicator; plant-side pump pressure dynamics
  are not modeled (the SWC's pump cap/off-guard logic is still exercised).
* COM alive supervision uses the 1-bit alternating counter of frame 672 (per the
  committed DBC). Rolling counters or CRC checking are out of scope.
* The scenario MCAP recording writes one fixed-schema JSON channel
  (`/wiper/state`) without compression (`mcap` default features off) — record/
  replay of Foxglove *layouts* still works, but no additional channels or
  binary/compressed payloads are emitted (see `docs/adr/adr-0006-scenario-tooling.md`).

