//! COM layer of the simulated basic software: CAN frame packing/decoding for the
//! four frames defined in `arxml/wiper.dbc` plus alive supervision.
//!
//! Frames (see the signal table and the AUTOSAR→COM mapping in
//! `docs/bsw-sim.md`):
//!
//! | ID (dec) | Name | Sender | DLC | Signals |
//! |---|---|---|---|---|
//! | 672 | `WiperStalkFrame` | `BodyCtrl` | 2 | `StalkMode` 0..2, `WashButton` 3, `Sensitivity` 4..5, `AutoFlag` 6, `StalkCounter` 15 |
//! | 673 | `RainFrame` | `RainSensor` | 2 | `RainPct` 0..7 (×100/255 %), `RainValid` 8 |
//! | 674 | `VehicleSpeedFrame` | `AbsEcu` | 3 | `SpeedKph` 0..12 (×0.05 km/h), `SpeedValid` 15 |
//! | 675 | `WiperStatusFrame` | `WiperEcu` | 3 | `WiperActive` 0, `MotorSpeedMode` 1..2, `PumpOn` 3, `FsmState` 4..7, `StallFault` 12, `DelayCounter` 15 |
//!
//! Alive supervision: the stalk frame carries a 1-bit alternating counter
//! (`StalkCounter`); the COM layer declares the stalk input lost when no *new
//! counter value* is seen for [`STALK_RX_TIMEOUT_MS`] (REQ-WIP-015 > 1 s rule)
//! and pushes `valid = false` data so the SWC can degrade to LOW. Rain/speed
//! transport staleness is supervised by the RTE port receivers in `ecu.rs`
//! ([`RAIN_RX_TIMEOUT_MS`] / [`SPEED_RX_TIMEOUT_MS`]).

#![forbid(unsafe_code)]

use autosar_rte::port::{NonQueuedPort, RteSender};
use wiper_swc::{MotorSpeed, StalkMode, WiperFsm};

/// CAN identifier of `WiperStalkFrame` (`BodyCtrl` → `WiperEcu`).
pub const STALK_FRAME_ID: u16 = 672;
/// CAN identifier of `RainFrame` (`RainSensor` → `WiperEcu`).
pub const RAIN_FRAME_ID: u16 = 673;
/// CAN identifier of `VehicleSpeedFrame` (`AbsEcu` → `WiperEcu`).
pub const SPEED_FRAME_ID: u16 = 674;
/// CAN identifier of `WiperStatusFrame` (`WiperEcu` → `ClusterDisplay`).
pub const STATUS_FRAME_ID: u16 = 675;

/// Stalk loss window: REQ-WIP-015 declares input loss after > 1 s.
pub const STALK_RX_TIMEOUT_MS: u32 = 1000;
/// Rain ele­ment loss window (same REQ-WIP-015 rule).
pub const RAIN_RX_TIMEOUT_MS: u32 = 1000;
/// Speed element loss window; a stale speed counts as 0 km/h.
pub const SPEED_RX_TIMEOUT_MS: u32 = 1000;

/// Classic CAN frame (standard 11-bit IDs in this network, payload ≤ 8 bytes).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanFrame {
    /// CAN identifier.
    pub id: u16,
    /// Data length code (bytes of [`CanFrame::data`] in use).
    pub dlc: u8,
    /// Payload; bytes beyond `dlc` are zero padding (kept for determinism).
    pub data: [u8; 8],
}

impl CanFrame {
    /// Assemble a frame.
    #[must_use]
    pub const fn new(id: u16, dlc: u8, data: [u8; 8]) -> Self {
        Self { id, dlc, data }
    }
}

/// Decoded stalk data element (port payload).
///
/// The bool set mirrors the on-frame bitfield 1:1 — allowed struct style.
#[allow(clippy::struct_excessive_bools)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StalkRx {
    /// Stalk position.
    pub mode: StalkMode,
    /// Sensitivity level 1..=4 (raw field + 1; S1..S4).
    pub sensitivity: u8,
    /// Auto (rain-sensor) flag.
    pub auto: bool,
    /// Wash button pressed (momentary.
    pub wash_btn: bool,
    /// Alternating counter bit of this reception.
    pub counter: bool,
    /// COM validity: fresh decoded frame within the alive window.
    pub valid: bool,
}

/// Decoded rain data element (port payload).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RainRx {
    /// Rain intensity 0..=100 %.
    pub pct: u8,
    /// Sensor-reported validity bit.
    pub valid: bool,
}

/// Decoded speed data element (port payload).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SpeedRx {
    /// Vehicle speed in km/h.
    pub kph: u16,
    /// Sender-reported validity bit.
    pub valid: bool,
}

/// Signals of frame 675 (`WiperStatusFrame`).
///
/// The bool set mirrors the on-frame bitfield 1:1 — allowed struct style.
#[allow(clippy::struct_excessive_bools)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StatusSignals {
    /// Wiper active (motor run output).
    pub wiper_active: bool,
    /// Motor speed profile (`OFF`/`LOW`/`HIGH`, see DBC `VAL_`).
    pub speed_mode: MotorSpeed,
    /// Washer pump output.
    pub pump_on: bool,
    /// Wiper FSM state (0=Parked 1=IntPause 2=Sweeping 3=ReturningToPark).
    pub fsm: WiperFsm,
    /// Motor-stall DTC active.
    pub stall_fault: bool,
    /// INT delay pause indicator (bit 15 `DelayCounter`).
    pub int_pause: bool,
}

/// What [`Com::receive`] did with a frame (test/DET visibility).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ComEvent {
    /// Fresh stalk frame accepted.
    StalkAccepted,
    /// Alive counter did not change since the previous frame (stale sender).
    StalkDuplicateCounter,
    /// Undefined `StalkMode` raw value (4..7); frame dropped.
    StalkInvalidMode,
    /// Rain frame decoded and stored.
    RainAccepted,
    /// Speed frame decoded and stored.
    SpeedAccepted,
    /// Frame was not accepted (wrong id/short payload).
    FrameRejected,
}

/// Encode rain percent to the raw 8-bit field (round-half-up, 100 % → 255).
#[must_use]
pub fn rain_encode_raw(pct: u8) -> u8 {
    // (100·255 + 50)/100 = 255 stays in range: no clamping needed for pct ≤ 100.
    let raw = (u32::from(pct) * 255 + 50) / 100;
    u8::try_from(raw).unwrap_or(255)
}

/// Decode the rain raw field back to percent (round-half-up).
#[must_use]
pub fn rain_decode_pct(raw: u8) -> u8 {
    let pct = (u32::from(raw) * 100 + 127) / 255;
    u8::try_from(pct).unwrap_or(100)
}

/// Encode km/h to the raw 13-bit field (factor 0.05, physical max 250 km/h).
#[must_use]
pub fn speed_encode_raw(kph: u16) -> u32 {
    u32::from(kph).saturating_mul(20).min(5000)
}

/// Decode the speed raw field back to km/h (round-half-up, clamped to the
/// DBC physical range 0..=250).
#[must_use]
pub fn speed_decode_kph(raw: u32) -> u16 {
    u16::try_from((raw + 10) / 20).map_or(250, |v| v.min(250))
}

/// Little-endian bit-field read over the DBC `0|n@1+` layout (bit 0 = LSB).
fn get_le_bits(data: [u8; 8], start: usize, len: usize) -> u32 {
    let mut out = 0u32;
    for i in 0..len {
        let bit = start + i;
        let Some(byte) = data.get(bit / 8) else {
            break;
        };
        let set = (u32::from(*byte) >> (bit % 8)) & 1 == 1;
        out |= u32::from(set) << i;
    }
    out
}

/// Little-endian bit-field write over the DBC `0|n@1+` layout.
fn set_le_bits(data: &mut [u8; 8], start: usize, len: usize, value: u32) {
    for i in 0..len {
        let bit = start + i;
        let byte = bit / 8;
        let mask = 1u8 << (bit % 8);
        if let Some(slot) = data.get_mut(byte) {
            if (value >> i) & 1 == 1 {
                *slot |= mask;
            } else {
                *slot &= !mask;
            }
        }
    }
}

/// Pack frame 672. `sensitivity` is the level 1..=4 (encoded raw = level − 1).
#[must_use]
pub fn pack_stalk(
    mode: StalkMode,
    sensitivity: u8,
    auto: bool,
    wash_btn: bool,
    counter: bool,
) -> CanFrame {
    let mut data = [0u8; 8];
    let mode_raw = match mode {
        StalkMode::Off => 0,
        StalkMode::Int => 1,
        StalkMode::Low => 2,
        StalkMode::High => 3,
    };
    set_le_bits(&mut data, 0, 3, mode_raw);
    set_le_bits(&mut data, 3, 1, u32::from(wash_btn));
    set_le_bits(&mut data, 4, 2, u32::from(sensitivity.saturating_sub(1)));
    set_le_bits(&mut data, 6, 1, u32::from(auto));
    set_le_bits(&mut data, 15, 1, u32::from(counter));
    CanFrame::new(STALK_FRAME_ID, 2, data)
}

/// Decode frame 672; `None` when the frame is another id / too short.
#[must_use]
pub fn decode_stalk(frame: &CanFrame) -> Option<StalkRx> {
    if frame.id != STALK_FRAME_ID || frame.dlc < 2 {
        return None;
    }
    let mode = match get_le_bits(frame.data, 0, 3) {
        0 => Some(StalkMode::Off),
        1 => Some(StalkMode::Int),
        2 => Some(StalkMode::Low),
        3 => Some(StalkMode::High),
        _ => None,
    };
    let sensitivity = (get_le_bits(frame.data, 4, 2) + 1).min(4);
    Some(StalkRx {
        mode: mode.unwrap_or(StalkMode::Off),
        sensitivity: u8::try_from(sensitivity).unwrap_or(4),
        auto: get_le_bits(frame.data, 6, 1) == 1,
        wash_btn: get_le_bits(frame.data, 3, 1) == 1,
        counter: get_le_bits(frame.data, 15, 1) == 1,
        valid: mode.is_some(),
    })
}

/// Pack frame 673.
#[must_use]
pub fn pack_rain(pct: u8, valid: bool) -> CanFrame {
    let mut data = [0u8; 8];
    set_le_bits(&mut data, 0, 8, u32::from(rain_encode_raw(pct)));
    set_le_bits(&mut data, 8, 1, u32::from(valid));
    CanFrame::new(RAIN_FRAME_ID, 2, data)
}

/// Decode frame 673; `None` when the frame is another id / too short.
#[must_use]
pub fn decode_rain(frame: &CanFrame) -> Option<RainRx> {
    if frame.id != RAIN_FRAME_ID || frame.dlc < 2 {
        return None;
    }
    Some(RainRx {
        pct: rain_decode_pct(u8::try_from(get_le_bits(frame.data, 0, 8)).unwrap_or(100)),
        valid: get_le_bits(frame.data, 8, 1) == 1,
    })
}

/// Pack frame 674.
#[must_use]
pub fn pack_speed(kph: u16, valid: bool) -> CanFrame {
    let mut data = [0u8; 8];
    set_le_bits(&mut data, 0, 13, speed_encode_raw(kph));
    set_le_bits(&mut data, 15, 1, u32::from(valid));
    CanFrame::new(SPEED_FRAME_ID, 3, data)
}

/// Decode frame 674; `None` when the frame is another id / too short.
#[must_use]
pub fn decode_speed(frame: &CanFrame) -> Option<SpeedRx> {
    if frame.id != SPEED_FRAME_ID || frame.dlc < 3 {
        return None;
    }
    Some(SpeedRx {
        kph: speed_decode_kph(get_le_bits(frame.data, 0, 13)),
        valid: get_le_bits(frame.data, 15, 1) == 1,
    })
}

/// Pack frame 675.
#[must_use]
pub fn pack_status(s: &StatusSignals) -> CanFrame {
    let mut data = [0u8; 8];
    let speed_raw = match s.speed_mode {
        MotorSpeed::Off => 0,
        MotorSpeed::Low => 1,
        MotorSpeed::High => 2,
    };
    let fsm_raw = match s.fsm {
        WiperFsm::Parked => 0,
        WiperFsm::IntPause => 1,
        WiperFsm::Sweeping => 2,
        WiperFsm::ReturningToPark => 3,
    };
    set_le_bits(&mut data, 0, 1, u32::from(s.wiper_active));
    set_le_bits(&mut data, 1, 2, speed_raw);
    set_le_bits(&mut data, 3, 1, u32::from(s.pump_on));
    set_le_bits(&mut data, 4, 4, fsm_raw);
    set_le_bits(&mut data, 12, 1, u32::from(s.stall_fault));
    set_le_bits(&mut data, 15, 1, u32::from(s.int_pause));
    CanFrame::new(STATUS_FRAME_ID, 3, data)
}

/// Decode frame 675; `None` when the frame is another id / too short.
#[must_use]
pub fn decode_status(frame: &CanFrame) -> Option<StatusSignals> {
    if frame.id != STATUS_FRAME_ID || frame.dlc < 3 {
        return None;
    }
    let speed_mode = match get_le_bits(frame.data, 1, 2) {
        0 => Some(MotorSpeed::Off),
        1 => Some(MotorSpeed::Low),
        2 => Some(MotorSpeed::High),
        _ => None,
    };
    let fsm = match get_le_bits(frame.data, 4, 4) {
        0 => Some(WiperFsm::Parked),
        1 => Some(WiperFsm::IntPause),
        2 => Some(WiperFsm::Sweeping),
        3 => Some(WiperFsm::ReturningToPark),
        _ => None,
    };
    Some(StatusSignals {
        wiper_active: get_le_bits(frame.data, 0, 1) == 1,
        speed_mode: speed_mode.unwrap_or(MotorSpeed::Off),
        pump_on: get_le_bits(frame.data, 3, 1) == 1,
        fsm: fsm.unwrap_or(WiperFsm::ReturningToPark),
        stall_fault: get_le_bits(frame.data, 12, 1) == 1,
        int_pause: get_le_bits(frame.data, 15, 1) == 1,
    })
}

/// The three received data-element buffers (RTE bindings of the wiper ECU).
/// In a real ECU the RTE generates these from the ARXML port declarations.
pub struct ComPorts {
    /// Stalk data element (frame 672).
    pub stalk: NonQueuedPort<StalkRx>,
    /// Rain data element (frame 673).
    pub rain: NonQueuedPort<RainRx>,
    /// Speed data element (frame 674).
    pub speed: NonQueuedPort<SpeedRx>,
}

impl ComPorts {
    /// Empty ports (no init values: no data before the first frame).
    #[must_use]
    pub const fn new() -> Self {
        Self {
            stalk: NonQueuedPort::new(None),
            rain: NonQueuedPort::new(None),
            speed: NonQueuedPort::new(None),
        }
    }
}

impl Default for ComPorts {
    fn default() -> Self {
        Self::new()
    }
}

fn write_port<T: Copy>(port: &NonQueuedPort<T>, value: T, now_ms: u32) {
    RteSender::new(port).Rte_Write(value, now_ms);
}

/// COM receiver state: alive-counter supervision of the stalk frame.
///
/// The counter bit must change on every accepted frame (alternating
/// supervision). When no acceptance happens inside [`STALK_RX_TIMEOUT_MS`],
/// the last stalk content is re-published with `valid = false` (safe-fallback
/// push) so the SWC degrades to LOW wiping and raises `InputTimeout`.
#[derive(Debug, Clone, Copy)]
pub struct Com {
    last_counter: Option<bool>,
    last_fresh_ms: u32,
    timed_out: bool,
    last_stalk: Option<StalkRx>,
}

impl Default for Com {
    fn default() -> Self {
        Self::new()
    }
}

impl Com {
    /// No frames seen yet.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            last_counter: None,
            last_fresh_ms: 0,
            timed_out: false,
            last_stalk: None,
        }
    }

    /// Receive one frame from the bus: decode and (for the stalk frame)
    /// supervise the alternating counter, then publish the data element.
    pub fn receive(&mut self, ports: &ComPorts, frame: &CanFrame, now_ms: u32) -> ComEvent {
        match frame.id {
            STALK_FRAME_ID => self.receive_stalk(ports, frame, now_ms),
            RAIN_FRAME_ID => match decode_rain(frame) {
                Some(rx) => {
                    write_port(&ports.rain, rx, now_ms);
                    ComEvent::RainAccepted
                }
                None => ComEvent::FrameRejected,
            },
            SPEED_FRAME_ID => match decode_speed(frame) {
                Some(rx) => {
                    write_port(&ports.speed, rx, now_ms);
                    ComEvent::SpeedAccepted
                }
                None => ComEvent::FrameRejected,
            },
            _ => ComEvent::FrameRejected,
        }
    }

    fn receive_stalk(&mut self, ports: &ComPorts, frame: &CanFrame, now_ms: u32) -> ComEvent {
        let Some(rx) = decode_stalk(frame) else {
            return ComEvent::FrameRejected;
        };
        if !rx.valid {
            return ComEvent::StalkInvalidMode;
        }
        let fresh = self.last_counter.is_none_or(|prev| prev != rx.counter);
        if !fresh {
            return ComEvent::StalkDuplicateCounter;
        }
        self.last_counter = Some(rx.counter);
        self.last_fresh_ms = now_ms;
        self.timed_out = false;
        self.last_stalk = Some(rx);
        write_port(&ports.stalk, rx, now_ms);
        ComEvent::StalkAccepted
    }

    /// Declare stalk loss when the alive window has elapsed. Returns `true`
    /// only on the trip edge; the fallback data is pushed once per outage.
    pub fn poll(&mut self, ports: &ComPorts, now_ms: u32) -> bool {
        if self.timed_out || self.last_counter.is_none() {
            return false;
        }
        if now_ms.saturating_sub(self.last_fresh_ms) > STALK_RX_TIMEOUT_MS {
            self.timed_out = true;
            if let Some(core) = self.last_stalk {
                write_port(
                    &ports.stalk,
                    StalkRx {
                        valid: false,
                        ..core
                    },
                    now_ms,
                );
            }
            return true;
        }
        false
    }

    /// Is the stalk element currently declared lost?
    #[must_use]
    pub const fn is_stalk_timed_out(&self) -> bool {
        self.timed_out
    }
}

#[cfg(test)]
mod tests {
    use autosar_rte::port::{ReadStatus, RteReceiver};
    use wiper_swc::{StalkMode, WiperFsm};

    use super::*;

    #[test]
    fn stalk_frame_roundtrip() {
        let f = pack_stalk(StalkMode::Int, 3, true, true, false);
        assert_eq!((f.id, f.dlc), (672, 2));
        let rx = decode_stalk(&f);
        assert!(rx.is_some());
        let rx = rx.unwrap_or(StalkRx {
            mode: StalkMode::Off,
            sensitivity: 0,
            auto: false,
            wash_btn: false,
            counter: false,
            valid: false,
        });
        assert_eq!(rx.mode, StalkMode::Int);
        assert_eq!(rx.sensitivity, 3);
        assert!(rx.auto && rx.wash_btn && rx.valid);
        assert!(!rx.counter);
    }

    #[test]
    fn sensitivity_levels_roundtrip() {
        for level in 1..=4u8 {
            let rx = decode_stalk(&pack_stalk(StalkMode::Int, level, false, false, true));
            assert!(rx.is_some_and(|r| r.sensitivity == level));
        }
    }

    #[test]
    fn undefined_stalk_mode_is_marked_invalid() {
        let mut f = pack_stalk(StalkMode::Off, 1, false, false, true);
        set_le_bits(&mut f.data, 0, 3, 5); // raw StalkMode 5 = undefined
        let rx = decode_stalk(&f);
        assert!(rx.is_some());
        assert!(rx.is_none_or(|r| !r.valid));
    }

    #[test]
    fn rain_percent_roundtrip() {
        for pct in [0u8, 1, 9, 39, 50, 51, 80, 99, 100] {
            let raw = rain_encode_raw(pct);
            assert_eq!(rain_decode_pct(raw), pct, "pct {pct} ↔ raw {raw}");
        }
        assert_eq!(rain_encode_raw(100), 255);
    }

    #[test]
    fn speed_roundtrip_and_clamp() {
        assert_eq!(speed_decode_kph(speed_encode_raw(0)), 0);
        assert_eq!(speed_decode_kph(speed_encode_raw(113)), 113);
        assert_eq!(speed_decode_kph(speed_encode_raw(200)), 200);
        // Physical max 250 km/h regardless of the implausible raw value.
        assert_eq!(speed_encode_raw(400), 5000);
        assert_eq!(speed_decode_kph(8000), 250);
    }

    #[test]
    fn status_frame_roundtrip() {
        let s = StatusSignals {
            wiper_active: true,
            speed_mode: MotorSpeed::Low,
            pump_on: true,
            fsm: WiperFsm::ReturningToPark,
            stall_fault: true,
            int_pause: false,
        };
        let rx = decode_status(&pack_status(&s));
        assert_eq!(rx, Some(s));
    }

    #[test]
    fn wrong_id_and_short_dlc_rejected() {
        let mut f = pack_stalk(StalkMode::Low, 1, false, false, true);
        f.id = 700;
        assert!(decode_stalk(&f).is_none());
        f.id = 672;
        let mut short = f;
        short.dlc = 1;
        assert!(decode_stalk(&short).is_none());
        assert!(decode_rain(&f).is_none());
        assert!(decode_speed(&f).is_none());
        assert!(decode_status(&f).is_none());
    }

    fn port_state(ports: &ComPorts, now: u32) -> (Option<StalkRx>, ReadStatus) {
        let r = RteReceiver::new(&ports.stalk, None).Rte_Read(now);
        (r.value, r.status)
    }

    /// verifies: REQ-WIP-015 — alive-counter supervision trips after the
    /// 1 s window and the invalid-push drives the SWC-side LOW fallback.
    #[test]
    fn stalk_alive_supervision_falls_back() {
        let ports = ComPorts::new();
        let mut com = Com::new();

        // Fresh frames alternate the counter bit.
        assert_eq!(
            com.receive(
                &ports,
                &pack_stalk(StalkMode::Low, 2, false, false, true),
                10
            ),
            ComEvent::StalkAccepted
        );
        // Same counter again → stale sender, data element not refreshed.
        assert_eq!(
            com.receive(
                &ports,
                &pack_stalk(StalkMode::High, 2, false, false, true),
                20
            ),
            ComEvent::StalkDuplicateCounter
        );
        let (val, status) = port_state(&ports, 21);
        assert_eq!(val.map(|v| v.mode), Some(StalkMode::Low));
        assert_eq!(status, ReadStatus::Valid);

        // Next toggle → fresh again.
        assert_eq!(
            com.receive(
                &ports,
                &pack_stalk(StalkMode::Low, 2, false, false, false),
                30
            ),
            ComEvent::StalkAccepted
        );

        // Wait past the 1 s window → timeout edge, exactly once.
        assert!(!com.poll(&ports, 999));
        assert!(com.poll(&ports, 1031));
        assert!(!com.poll(&ports, 1041));
        assert!(com.is_stalk_timed_out());
        let (val, status) = port_state(&ports, 1_041);
        assert!(
            !val.is_none_or(|v| v.valid),
            "fallback data must be invalid"
        );
        assert_eq!(status, ReadStatus::Valid, "fallback push is a fresh write");

        // New toggled frame within the window recovers the data path.
        assert_eq!(
            com.receive(
                &ports,
                &pack_stalk(StalkMode::Low, 2, false, false, true),
                1_050
            ),
            ComEvent::StalkAccepted
        );
        assert!(!com.is_stalk_timed_out());
        let (val, _) = port_state(&ports, 1_051);
        assert!(val.is_some_and(|v| v.valid));
    }

    /// verifies: REQ-WIP-015 (undefined mode must not be treated as valid input)
    #[test]
    fn undefined_mode_does_not_refresh_supervision() {
        let ports = ComPorts::new();
        let mut com = Com::new();
        let mut f = pack_stalk(StalkMode::Off, 1, false, false, false);
        set_le_bits(&mut f.data, 0, 3, 7);
        assert_eq!(com.receive(&ports, &f, 5), ComEvent::StalkInvalidMode);
        // The alive counter is still unarmed: nothing was accepted.
        assert!(!com.is_stalk_timed_out() && com.last_counter.is_none());
        // No timeout can trip without a first accepted frame.
        assert!(!com.poll(&ports, 5000));
    }
}
