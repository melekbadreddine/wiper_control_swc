//! Recording formats: one CSV line per 10 ms tick plus an MCAP stream whose
//! messages are JSON snapshots of this row (Foxglove-ready; see ADR-0006).
//!
//! Column contract (golden CSVs in `tests/golden/` are byte-compared):
//!
//! ```text
//! t_ms,wipe_count,fsm,wash_phase,motor_speed,duty_pct,pump_on,
//! dtc_active_bits,dtc_latched_bits,arm_deg,arm_speed_dps,current_da,
//! battery_v01,rain_pct,speed_kph,stalk_valid
//! ```

use std::io::Write;
use std::path::Path;

use wiper_swc::{MotorSpeed, WiperFsm};

/// One recorded tick of the closed loop.
#[derive(Debug, Clone)]
pub struct SnapshotRow {
    /// Virtual time (ms) of this tick.
    pub t_ms: u32,
    /// Completed wipes so far.
    pub wipe_count: u32,
    /// Wiper FSM state.
    pub fsm: WiperFsm,
    /// Wash-sequencing phase (`Display`: e.g. `AfterRelease(2)`).
    pub wash_phase: String,
    /// Motor speed profile.
    pub motor_speed: MotorSpeed,
    /// PWM duty percent.
    pub duty_pct: u8,
    /// Washer pump DIO.
    pub pump_on: bool,
    /// Active DTC bitset (bit order = `Dtc::ALL`).
    pub dtc_active_bits: u16,
    /// Latched DTC bitset.
    pub dtc_latched_bits: u16,
    /// Arm angle (deg; plant feedback).
    pub arm_deg: u16,
    /// Arm angular speed (deg/s).
    pub arm_speed_dps: i16,
    /// Motor current (dA).
    pub current_da: u16,
    /// Battery voltage in 0.1 V units.
    pub battery_v01: u16,
    /// Rain intensity percent as received by the COM layer (0 when stale).
    pub rain_pct: u8,
    /// Vehicle speed km/h as received (0 when stale).
    pub speed_kph: u16,
    /// Stalk data element valid (alive supervision passes).
    pub stalk_valid: bool,
    /// MCAP log time (ns).
    pub log_time_ns: u64,
    /// MCAP message sequence number.
    pub sequence: u32,
}

impl SnapshotRow {
    /// Capture one row from the ECU at the end of a tick.
    pub fn capture(ecu: &bsw_sim::Ecu, log_time_ns: u64, sequence: u32) -> Self {
        let out = ecu.last_outputs();
        let adc = ecu.iohwab().adc();
        let diag = ecu.last_diag();
        let rain = ecu.read_rain().value;
        let speed = ecu.read_speed().value;
        SnapshotRow {
            t_ms: ecu.t_ms(),
            wipe_count: ecu.swc().wipe_count(),
            fsm: ecu.swc().fsm_state(),
            wash_phase: ecu.swc().wash_phase().to_string(),
            motor_speed: out.motor.speed,
            duty_pct: out.motor.duty_pct,
            pump_on: out.pump_on,
            dtc_active_bits: diag.dtc_active_bits,
            dtc_latched_bits: diag.dtc_latched_bits,
            arm_deg: adc.angle_deg,
            arm_speed_dps: adc.arm_speed_dps,
            current_da: adc.current_da,
            battery_v01: adc.battery_v01,
            rain_pct: rain.map_or(0, |r| r.pct),
            speed_kph: speed.map_or(0, |s| s.kph),
            stalk_valid: ecu.read_stalk().value.is_some_and(|s| s.valid),
            log_time_ns,
            sequence,
        }
    }

    /// CSV header line (exactly one).
    #[must_use]
    pub const fn csv_header() -> &'static str {
        "t_ms,wipe_count,fsm,wash_phase,motor_speed,duty_pct,pump_on,\
         dtc_active_bits,dtc_latched_bits,arm_deg,arm_speed_dps,current_da,\
         battery_v01,rain_pct,speed_kph,stalk_valid"
    }

    /// One CSV line, including the trailing newline.
    #[must_use]
    pub fn csv_line(&self) -> String {
        let motor = match self.motor_speed {
            MotorSpeed::Off => "Off",
            MotorSpeed::Low => "Low",
            MotorSpeed::High => "High",
        };
        format!(
            "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n",
            self.t_ms,
            self.wipe_count,
            self.fsm,
            self.wash_phase,
            motor,
            self.duty_pct,
            u8::from(self.pump_on),
            self.dtc_active_bits,
            self.dtc_latched_bits,
            self.arm_deg,
            self.arm_speed_dps,
            self.current_da,
            self.battery_v01,
            self.rain_pct,
            self.speed_kph,
            u8::from(self.stalk_valid),
        )
    }

    /// JSON message payload for the MCAP `/wiper/state` channel
    /// (hand-serialized: flat JSON object of all row fields).
    #[must_use]
    pub fn json_payload(&self) -> String {
        let motor = match self.motor_speed {
            MotorSpeed::Off => "Off",
            MotorSpeed::Low => "Low",
            MotorSpeed::High => "High",
        };
        format!(
            concat!(
                r#"{{"t_ms":{0},"wipe_count":{1},"fsm":"{2}","wash_phase":"{3}","#,
                r#""motor_speed":"{4}","duty_pct":{5},"pump_on":{6},"#,
                r#""dtc_active_bits":{7},"dtc_latched_bits":{8},"arm_deg":{9},"#,
                r#""arm_speed_dps":{10},"current_da":{11},"battery_v01":{12},"#,
                r#""rain_pct":{13},"speed_kph":{14},"stalk_valid":{15}}}"#
            ),
            self.t_ms,
            self.wipe_count,
            self.fsm,
            self.wash_phase,
            motor,
            self.duty_pct,
            self.pump_on,
            self.dtc_active_bits,
            self.dtc_latched_bits,
            self.arm_deg,
            self.arm_speed_dps,
            self.current_da,
            self.battery_v01,
            self.rain_pct,
            self.speed_kph,
            self.stalk_valid,
        )
    }
}

/// Writer for `<name>.csv` (header + one line per row).
///
/// # Errors
/// Filesystem I/O errors (create/write/flush) propagate.
pub fn write_csv(path: &Path, rows: &[SnapshotRow]) -> std::io::Result<()> {
    let mut file = std::fs::File::create(path)?;
    file.write_all(SnapshotRow::csv_header().as_bytes())?;
    file.write_all(b"\n")?;
    let mut buf = Vec::with_capacity(rows.len() * 64);
    for row in rows {
        buf.extend_from_slice(row.csv_line().as_bytes());
    }
    file.write_all(&buf)?;
    file.flush()
}

/// Incremental MCAP logger: one JSON message per tick on `/wiper/state`.
pub struct McapLog {
    writer: mcap::Writer<std::io::BufWriter<std::fs::File>>,
    channel_id: u16,
}

impl McapLog {
    /// Create `<path>` and register the JSON schema + `/wiper/state` channel.
    ///
    /// # Errors
    /// I/O or MCAP framing errors propagate from the `mcap` crate.
    pub fn create(path: &Path) -> mcap::McapResult<Self> {
        let file = std::fs::File::create(path)?;
        let mut writer = mcap::Writer::new(std::io::BufWriter::new(file))?;
        // Minimal JSON schema so Foxglove renders the message as raw JSON.
        let schema_id =
            writer.add_schema("wiper.JsonState", "jsonschema", br#"{"type":"object"}"#)?;
        let channel_id = writer.add_channel(
            schema_id,
            "/wiper/state",
            "json",
            &std::collections::BTreeMap::new(),
        )?;
        Ok(Self { writer, channel_id })
    }

    /// Append one row as a JSON message (`log_time = t_ms × 10⁶` ns).
    ///
    /// # Errors
    /// MCAP write errors (I/O underneath) propagate.
    pub fn write_row(&mut self, row: &SnapshotRow) -> mcap::McapResult<()> {
        let header = mcap::records::MessageHeader {
            channel_id: self.channel_id,
            sequence: row.sequence,
            log_time: row.log_time_ns,
            publish_time: row.log_time_ns,
        };
        self.writer
            .write_to_known_channel(&header, row.json_payload().as_bytes())
    }

    /// Write the summary records and flush; call before dropping.
    ///
    /// # Errors
    /// MCAP summary-generation errors propagate.
    pub fn finish(&mut self) -> mcap::McapResult<()> {
        self.writer.finish().map(|_| ())
    }
}
