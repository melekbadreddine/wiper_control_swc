# BSW simulation: OS, COM, IoHwAb, DEM, NvM (`bsw-sim`)

`crates/bsw-sim` implements the simulated basic software of the wiper ECU so the
real application (`wiper-swc::WiperSwc`, untouched) runs its scheduled runnables
against the real plant (`plant-sim::WiperPlant`) with realistic data pipelines.

All claims in this document are re-checkable from the test suite
(`cargo test -p bsw-sim`, 18 module tests + 10 closed-loop integration tests as
of M4).

## 1. AUTOSAR BSW module ↔ crate mapping

| AUTOSAR module | `bsw-sim` module | What it provides |
|---|---|---|
| OS (schedule table, OSEK) | `sched::Scheduler` | virtual time, cyclic activation events, activation statistics |
| COM (transport + signal layer) | `com` | DBC signal pack/decode, alternating-counter alive supervision, data elements |
| RTE data elements | `com::ComPorts` + `autosar-rte::NonQueuedPort` | last-is-best buffers, sender/receiver handles, ReadStatus semantics |
| RTE mode switch | `autosar-rte::ModeSwitchPort` (in `ecu::Ecu`) | ignition mode manager/user pair |
| IoHwAb | `iohwab::IoHwAb` | motor PWM + pump DIO latch, ADC sample of the plant feedback |
| DEM | `dem::Dem` | diagnostic events: active/latched/occurrences per DTC |
| DET | `dem::Det` | development-error counter (COM protocol deviations) |
| NvM | `nvm::NvmStub` | job-queued RAM-mirror block (`NvmImage`) |
| ECU wiring | `ecu::Ecu` | runs the schedule table, connects everything |

## 2. OS schedule table

`Ecu::new()` represents power-on: the init event executes `Wiper_Init` during
construction. Each `Ecu::tick()` advances the virtual OS time by the 10 ms base
tick and activates due tasks (activation iff `t_ms % period == 0`):

| Virtual time (ms) | Activation | Runnable |
|---|---|---|
| 10, 20, 30, … | cyclic 10 ms | `Wiper_Run10ms` |
| 100, 200, 300, … | cyclic 100 ms | `Wiper_Run100ms` |

The scheduler rejects periods that are not multiples of the base tick
(`SchedError::NotTickAligned`); activation sequences are exact and reproducible
(verified by `os_cyclic_schedule` and the determinism tests, REQ-WIP-020/019).

## 3. COM: signal encoding (source: `arxml/wiper.dbc`)

Bit numbering follows the DBC convention `bit 0 = LSB of byte 0` (little-endian
byte order, Motorola ordering within the table below noted as `byte.bit`).

| Frame (`BO_`) | Sender | DLC | Signals (`SG_`, DBC bit layout) |
|---|---|---|---|
| 672 `WiperStalkFrame` | `BodyCtrl` | 2 | `StalkMode` [0|3], `WashButton` [3|1], `Sensitivity` [4|2], `AutoFlag` [6|1], `StalkCounter` [15|1] |
| 673 `RainFrame` | `RainSensor` | 2 | `RainPct` [0|8], `RainValid` [8|1] |
| 674 `VehicleSpeedFrame` | `AbsEcu` | 3 | `SpeedKph` [0|13], `SpeedValid` [15|1] |
| 675 `WiperStatusFrame` | `WiperEcu` | 3 | `WiperActive` [0|1], `MotorSpeedMode` [1|2], `PumpOn` [3|1], `FsmState` [4|4], `StallFault` [12|1], `DelayCounter` [15|1] |

Physical encodings (both directions deterministic integer math, no floats):

| Signal | Encoding | Decoding |
|---|---|---|
| `RainPct` (0–100 %) | `raw = round(pct · 255/100)`, `pct = 100 ⇒ raw = 255` | `pct = (raw·100 + 127) / 255` (round-half-up) |
| `SpeedKph` (0–250 km/h) | `raw = min(kph · 20, 5000)` (DBC factor 0.05) | `kph = min(round(raw / 20), 250)` |
| `Sensitivity` | `raw = level − 1` (0..3 → S1..S4; corrected DBC comment/`VAL_` in M4 — the 2-bit field cannot hold level numbers 1..4 directly) | `level = raw + 1` |
| `StalkMode` | raw 0=OFF 1=INT 2=LOW 3=HIGH (per `VAL_`) | raw 4..7 = undefined → element marked invalid |
| `MotorSpeedMode` (`VAL_` 675) | 0=OFF 1=LOW 2=HIGH | — |
| `FsmState` | 0=Parked 1=IntPause 2=Sweeping 3=ReturningToPark (`WiperFsm` declaration order) | — |
| `DelayCounter` (bit 15, exp) | INT delay-pause indicator = `fsm == IntPause` | — |

## 4. COM alive supervision and timeout policy

- The stalk frame's `StalkCounter` bit **must change on every accepted frame**
  (alternating counter; `SensorNodes` toggles it per transmission).
  A frame whose counter equals the previous one is stale (`ComEvent:
  StalkDuplicateCounter`) and does **not** refresh the data path. These
  protocol deviations are recorded by DET for test visibility.
- Losing the window: if no fresh counter arrives for
  `STALK_RX_TIMEOUT_MS = 1000` (REQ-WIP-015 ">1 s without valid frame"), COM
  declares the stalk element lost, re-publishes the *last* stalk content with
  `valid = false` (safe fallback push through the RTE port) and trips exactly
  once (`poll()` edge). The SWC then degrades to visible LOW wiping and sets
  `InputTimeout`; a later fresh counter restores `valid = true`.
- Rain/speed: no alive counter in the DBC — transport staleness is supervised
  by the RTE non-queued port receivers (`RAIN_RX_TIMEOUT_MS`/
  `SPEED_RX_TIMEOUT_MS` = 1000 ms each). The ECU treats
  `ReadStatus::Timeout` (or a missing element) as *invalid* input; the SWC
  implements the REQ-WIP-015 fallbacks (manual INT timing / speed 0).
- Frame 672 raw `StalkMode` values 4..7 are undefined (`VAL_` defines 0..3);
  such frames are dropped without refreshing the supervision
  (`ComEvent::StalkInvalidMode`).

## 5. IoHwAb mapping

| SWC element | Source/sink |
|---|---|
| `motor_current_dA`, `battery_v0p1`, `arm_angle_deg`, `arm_speed_dps`, `park_switch` | ADC: sampled `PlantOutput` (see `iohwab::IoHwAb::swc_inputs`) |
| `rain_pct`, `speed_kph` (+validity) | COM element ports (frames 673/674) |
| `ignition` | mode switch port (`Ecu::set_ignition`) |
| `Outputs.motor` → plant | PWM latch, applied 1:1 to `WiperPlant::tick_10ms` |
| `Outputs.pump_on` | pump DIO latch (plant side has no pump dynamics) |

Sampling order per tick: `Wiper_Run10ms` gets the ADC value captured at the
*previous* tick boundary (one-sample delay, like a real ADC at the task start),
then the fresh command is applied and integrated; the resulting snapshot is
captured for the next cycle.

## 6. DEM / DET / NvM

- **DEM**: one `DemEvent` per DTC with `active`, `latched`, occurrence count
  (rising edges) and first/last timestamps; the 100 ms task mirrors the SWC's
  `DiagStatus` bitset into it. `clear()` implements diagnostic-clear.
- **DET**: counts COM-level anomalies (duplicate counter, invalid mode value,
  alive-window trips) so tests can prove protocol cleanliness.
- **NvM**: `NvmStub` stores `dtc_latched_bits` + `wipe_count` behind a
  write-job queue (`request_write` → `process_jobs`). It is a RAM mirror —
  **no** persistence across power cycles (see `docs/limitations.md`).

## 7. Known simulation simplifications

All documented in `docs/limitations.md`: no CAN arbitration/bit timing (the bus
is a latest-frame-per-ID store with receive-once semantics), sensor nodes are
perfect periodic senders (fault injection = period 0 = node offline), the NvM
mirror is RAM-backed, and the pump is a binary indicator without plant-side
pressure dynamics.
