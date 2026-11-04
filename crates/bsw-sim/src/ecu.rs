//! The simulated wiper ECU: RTE wiring + OS schedule + COM + `IoHwAb` + DEM/NvM
//! around the real `wiper_swc::WiperSwc` application and the `plant-sim` plant.
//!
//! `Ecu::tick()` executes exactly one 10 ms OS base tick (call it in a loop to
//! simulate; all time is the scheduler's virtual time).
//!
//! # Schedule table
//!
//! | Time (ms) | Activation | Runnable |
//! |---|---|---|
//! | boot (construction) | init event | `Wiper_Init` |
//! | 10, 20, 30, … | cyclic 10 ms | `Wiper_Run10ms` |
//! | 100, 200, 300, … | cyclic 100 ms | `Wiper_Run100ms` (diag → DEM/NvM/status frame) |
//!
//! # Data flow of one 10 ms tick
//!
//! 1. OS advances 10 ms; sensor nodes transmit due CAN frames onto the bus.
//! 2. COM receives frames 672/673/674 → RTE data-element ports; stalk alive
//!    supervision may push `valid = false` (fault injection = node offline).
//! 3. Data-received edge on the stalk element → `Wiper_OnStalkChange`.
//! 4. `Wiper_Run10ms(t, Inputs)` → `Outputs` = motor PWM + pump DIO.
//! 5. `IoHwAb` latches outputs; plant integrates 10 × 1 ms RK4; ADC sampled.
//! 6. On 100 ms: `Wiper_Run100ms` snapshot → DEM reports, `NvM` mirror write,
//!    status frame 675 packed and transmitted (cluster display).

#![forbid(unsafe_code)]

use autosar_rte::mode::{IgnitionMode, ModeSwitchPort, ModeValue};
use autosar_rte::port::{ReadStatus, RteReceiver};
use plant_sim::WiperPlant;
use wiper_swc::{DiagStatus, Dtc, Ignition, Outputs, StalkMode, WiperFsm};

use crate::com::{
    self, CanFrame, Com, ComEvent, ComPorts, RainRx, SpeedRx, StalkRx, StatusSignals,
    RAIN_RX_TIMEOUT_MS, SPEED_RX_TIMEOUT_MS, STALK_FRAME_ID,
};
use crate::dem::{Dem, Det};
use crate::iohwab::IoHwAb;
use crate::nvm::{NvmImage, NvmStub};
use crate::sched::{Scheduler, TaskId};
use com::{RAIN_FRAME_ID, SPEED_FRAME_ID, STATUS_FRAME_ID};

/// Tasks of the wiper ECU's OS schedule table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Task {
    /// `Wiper_Run10ms` — 10 ms cyclic control task.
    Run10ms,
    /// `Wiper_Run100ms` — diagnostics snapshot → DEM/NvM/COM status.
    Diag100ms,
}

impl TaskId for Task {}

/// Simulated transmitter nodes on the virtual CAN bus.
///
/// `period_ms == 0` switches a node offline (fault injection for the sensor
/// timeout scenarios of M5).
#[derive(Debug, Clone, Copy)]
pub struct SensorNodes {
    /// `BodyCtrl` stalk/wash button node content (frame 672).
    pub stalk: StalkTx,
    /// `RainSensor` node content (frame 673).
    pub rain: RainTx,
    /// `AbsEcu` output node content (frame 674).
    pub speed: SpeedTx,
    /// Stalk frame transmit period (50 ms nominal; 0 = node offline).
    pub stalk_period_ms: u32,
    /// Rain frame transmit period (100 ms nominal; 0 = node offline).
    pub rain_period_ms: u32,
    /// Speed frame transmit period (100 ms nominal; 0 = node offline).
    pub speed_period_ms: u32,
    /// Alternating counter bit of frame 672 (toggles per transmission).
    stalk_counter: bool,
}

/// `BodyCtrl` content for frame 672 (`sensitivity` is the level 1..=4).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StalkTx {
    /// Selected stalk mode.
    pub mode: StalkMode,
    /// Sensitivity level 1..=4 (S1..S4).
    pub sensitivity: u8,
    /// Auto (rain-sensor) flag of the INT position.
    pub auto: bool,
    /// Momentary wash button state.
    pub wash_btn: bool,
}

/// `RainSensor` content for frame 673.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RainTx {
    /// Rain intensity 0..=100 %.
    pub pct: u8,
    /// Sensor validity flag.
    pub valid: bool,
}

/// `AbsEcu` content for frame 674.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SpeedTx {
    /// Vehicle speed km/h.
    pub kph: u16,
    /// Sender validity flag.
    pub valid: bool,
}

impl Default for SensorNodes {
    fn default() -> Self {
        Self::new()
    }
}

impl SensorNodes {
    /// All nodes online at their nominal periods, stalk OFF/S1, dry glass.
    #[must_use]
    pub fn new() -> Self {
        Self {
            stalk: StalkTx {
                mode: StalkMode::Off,
                sensitivity: 1,
                auto: false,
                wash_btn: false,
            },
            rain: RainTx {
                pct: 0,
                valid: true,
            },
            speed: SpeedTx {
                kph: 0,
                valid: true,
            },
            stalk_period_ms: 50,
            rain_period_ms: 100,
            speed_period_ms: 100,
            stalk_counter: false,
        }
    }

    /// Transmit the frames whose period elapsed; the stalk counter toggles on
    /// each transmission (alive-supervision requirement of the DBC).
    pub fn tick(&mut self, t_ms: u32, bus: &mut CanBus) {
        if self.stalk_period_ms > 0 && t_ms % self.stalk_period_ms == 0 {
            let s = self.stalk;
            let counter = self.stalk_counter;
            bus.write(com::pack_stalk(
                s.mode,
                s.sensitivity,
                s.auto,
                s.wash_btn,
                counter,
            ));
            self.stalk_counter = !self.stalk_counter;
        }
        if self.rain_period_ms > 0 && t_ms % self.rain_period_ms == 0 {
            bus.write(com::pack_rain(self.rain.pct, self.rain.valid));
        }
        if self.speed_period_ms > 0 && t_ms % self.speed_period_ms == 0 {
            bus.write(com::pack_speed(self.speed.kph, self.speed.valid));
        }
    }
}

/// Virtual CAN bus: stores the latest frame per identifier (no arbitration
/// latency in the simulation; see `docs/bsw-sim.md`).
#[derive(Debug)]
pub struct CanBus {
    slots: [Option<CanFrame>; 4],
}

impl Default for CanBus {
    fn default() -> Self {
        Self::new()
    }
}

impl CanBus {
    /// Empty bus.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            slots: [None, None, None, None],
        }
    }

    fn slot_index(id: u16) -> Option<usize> {
        match id {
            com::STALK_FRAME_ID => Some(0),
            com::RAIN_FRAME_ID => Some(1),
            com::SPEED_FRAME_ID => Some(2),
            com::STATUS_FRAME_ID => Some(3),
            _ => None,
        }
    }

    /// Transmit/overwrite the latest frame of its identifier.
    /// Returns `false` for identifiers outside the simulated frame set.
    pub fn write(&mut self, frame: CanFrame) -> bool {
        let Some(idx) = Self::slot_index(frame.id) else {
            return false;
        };
        if let Some(slot) = self.slots.get_mut(idx) {
            *slot = Some(frame);
            return true;
        }
        false
    }

    /// Latest frame on the bus for `id` (published/retained frame — e.g. the
    /// status frame for any number of display consumers).
    #[must_use]
    pub fn read(&self, id: u16) -> Option<CanFrame> {
        let idx = Self::slot_index(id)?;
        self.slots.get(idx).copied().flatten()
    }

    /// Receive-once semantics for RX nodes: returns the frame and clears the
    /// slot so a retained frame is not delivered repeatedly (one delivery per
    /// physical transmission).
    pub fn take(&mut self, id: u16) -> Option<CanFrame> {
        let idx = Self::slot_index(id)?;
        let slot = self.slots.get_mut(idx)?;
        slot.take()
    }

    /// Drop every frame from the bus (total network outage fault injection:
    /// receivers stop seeing *any* frame until nodes transmit again).
    pub fn clear(&mut self) {
        for slot in &mut self.slots {
            *slot = None;
        }
    }
}

/// One complete, deterministically scheduled wiper ECU (application + BSW).
pub struct Ecu {
    sched: Scheduler<Task>,
    ports: ComPorts,
    com: Com,
    swc: wiper_swc::WiperSwc,
    plant: WiperPlant,
    iohwab: IoHwAb,
    dem: Dem,
    det: Det,
    nvm: NvmStub,
    mode: ModeSwitchPort<IgnitionMode>,
    bus: CanBus,
    sensors: SensorNodes,
    t_ms: u32,
    last_outputs: Outputs,
    last_diag: DiagStatus,
    forwarded_stalk: Option<(StalkMode, u8, bool, bool, bool)>,
}

impl Default for Ecu {
    fn default() -> Self {
        Self::new()
    }
}

impl Ecu {
    /// Power on: OS schedule table + `Wiper_Init` (init event), plant at park.
    #[must_use]
    pub fn new() -> Self {
        let mut sched = Scheduler::new(10);
        // Periods are compile-time constants of this module; registration
        // cannot fail (both are exact 10 ms multiples).
        let _ = sched.register(Task::Run10ms, "Wiper_Run10ms", 10);
        let _ = sched.register(Task::Diag100ms, "Wiper_Run100ms", 100);
        let mut swc = wiper_swc::WiperSwc::new();
        swc.Wiper_Init();
        let mut ecu = Self {
            sched,
            ports: ComPorts::new(),
            com: Com::new(),
            swc,
            plant: WiperPlant::new(),
            iohwab: IoHwAb::new(),
            dem: Dem::new(),
            det: Det::new(),
            nvm: NvmStub::new(),
            mode: ModeSwitchPort::new(),
            bus: CanBus::new(),
            sensors: SensorNodes::new(),
            t_ms: 0,
            last_outputs: Outputs {
                motor: wiper_swc::MotorCommand {
                    speed: wiper_swc::MotorSpeed::Off,
                    duty_pct: 0,
                },
                pump_on: false,
            },
            last_diag: DiagStatus::default(),
            forwarded_stalk: None,
        };
        ecu.iohwab.capture_adc(ecu.plant.snapshot());
        ecu
    }

    // ------------------------------------------------------------- wiring I/O

    /// Mode manager side of the ignition mode switch group
    /// (`Rte_Switch_VehicleMode`).
    pub fn set_ignition(&mut self, ignition: Ignition) {
        let mode = match ignition {
            Ignition::Off => IgnitionMode::Off,
            Ignition::Acc => IgnitionMode::Acc,
            Ignition::On => IgnitionMode::On,
        };
        self.mode.Rte_Switch(mode);
    }

    /// Current ignition mode as the SWC sees it (`NotYetAvailable` → OFF).
    #[must_use]
    pub fn ignition(&self) -> Ignition {
        match self.mode.Rte_Mode() {
            ModeValue::Mode(mode) => match mode {
                IgnitionMode::Off => Ignition::Off,
                IgnitionMode::Acc => Ignition::Acc,
                IgnitionMode::On => Ignition::On,
            },
            // Before the first mode declaration the ECU behaves as ignition off.
            ModeValue::NotYetAvailable => Ignition::Off,
        }
    }

    /// Latest transmitted `WiperStatusFrame` (frame 675) if any.
    #[must_use]
    pub fn status_frame(&self) -> Option<CanFrame> {
        self.bus.read(STATUS_FRAME_ID)
    }

    // ------------------------------------------------------------ accessors

    /// Virtual time of the scheduler (ms since power on).
    #[must_use]
    pub const fn t_ms(&self) -> u32 {
        self.t_ms
    }

    /// The application (read only: state/telemetry probes).
    #[must_use]
    pub const fn swc(&self) -> &wiper_swc::WiperSwc {
        &self.swc
    }

    /// Application (mutations for test probes; the schedule still applies).
    pub fn swc_mut(&mut self) -> &mut wiper_swc::WiperSwc {
        &mut self.swc
    }

    /// The physical plant.
    #[must_use]
    pub const fn plant(&self) -> &WiperPlant {
        &self.plant
    }

    /// The physical plant (fault injection, scenario hooks of M5).
    pub fn plant_mut(&mut self) -> &mut WiperPlant {
        &mut self.plant
    }

    /// The virtual CAN bus (mutations for testing/scenario fault injection).
    pub fn bus_mut(&mut self) -> &mut CanBus {
        &mut self.bus
    }

    /// Simulated CAN bus.
    #[must_use]
    pub const fn bus(&self) -> &CanBus {
        &self.bus
    }

    /// Sensor nodes (stalk/rain/speed content and transmit periods).
    #[must_use]
    pub const fn sensors(&self) -> &SensorNodes {
        &self.sensors
    }

    /// Sensor nodes (mutate content / take a node offline).
    pub fn sensors_mut(&mut self) -> &mut SensorNodes {
        &mut self.sensors
    }

    /// DEM store.
    #[must_use]
    pub const fn dem(&self) -> &Dem {
        &self.dem
    }

    /// `NvM` mirror.
    #[must_use]
    pub const fn nvm(&self) -> &NvmStub {
        &self.nvm
    }

    /// DET violations counter of the simulated BSW.
    #[must_use]
    pub const fn det(&self) -> &Det {
        &self.det
    }

    /// Latest `IoHwAb` hardware image.
    #[must_use]
    pub const fn iohwab(&self) -> &IoHwAb {
        &self.iohwab
    }

    /// Latest `Wiper_Run10ms` outputs.
    #[must_use]
    pub const fn last_outputs(&self) -> &Outputs {
        &self.last_outputs
    }

    /// Latest `Wiper_Run100ms` snapshot.
    #[must_use]
    pub const fn last_diag(&self) -> &DiagStatus {
        &self.last_diag
    }

    /// Stalk data element as the SWC path sees it (value + RTE status).
    #[must_use]
    pub fn read_stalk(&self) -> autosar_rte::port::Received<StalkRx> {
        self.read_stalk_at(self.t_ms)
    }

    /// Rain data element as the SWC path sees it (with transport supervision).
    #[must_use]
    pub fn read_rain(&self) -> autosar_rte::port::Received<RainRx> {
        self.read_rain_at(self.t_ms)
    }

    /// Speed data element as the SWC path sees it (with transport supervision).
    #[must_use]
    pub fn read_speed(&self) -> autosar_rte::port::Received<SpeedRx> {
        self.read_speed_at(self.t_ms)
    }

    // ------------------------------------------------------------ scheduling

    /// Execute exactly one 10 ms OS tick (activates the due runnables).
    pub fn tick(&mut self) {
        let due = self.sched.tick();
        let t = self.sched.now_ms();
        self.t_ms = t;

        // 1. Sensor nodes transmit; 2. COM receives into the RTE ports
        //    (receive-once: each physical transmission is consumed exactly
        //    once, matching real bus reception).
        self.sensors.tick(t, &mut self.bus);
        for id in [STALK_FRAME_ID, RAIN_FRAME_ID, SPEED_FRAME_ID] {
            if let Some(frame) = self.bus.take(id) {
                let event = self.com.receive(&self.ports, &frame, t);
                match event {
                    ComEvent::StalkAccepted
                    | ComEvent::RainAccepted
                    | ComEvent::SpeedAccepted
                    | ComEvent::FrameRejected => {}
                    // Protocol anomalies are DET-relevant (test visibility).
                    ComEvent::StalkDuplicateCounter | ComEvent::StalkInvalidMode => {
                        self.det.report();
                    }
                }
            }
        }
        // 3. Stalk alive supervision may declare input loss (fallback push).
        if self.com.poll(&self.ports, t) {
            self.det.report();
        }

        // 4. Data-received edge → `Wiper_OnStalkChange`.
        self.forward_stalk_event();

        // 5. 10 ms control task.
        if due.contains(&Task::Run10ms) {
            let inputs = self.assemble_inputs(t);
            let outputs = self.swc.Wiper_Run10ms(t, &inputs);
            self.iohwab.write_outputs(outputs);
            let snapshot = self.plant.tick_10ms(self.iohwab.motor_command());
            self.iohwab.capture_adc(snapshot);
            self.last_outputs = outputs;
        }

        // 6. 100 ms diagnostic + status task.
        if due.contains(&Task::Diag100ms) {
            self.run_diag_100ms(t);
        }
    }

    fn run_diag_100ms(&mut self, t: u32) {
        let diag = self.swc.Wiper_Run100ms();
        self.last_diag = diag;
        // Mirror active DTC bits into the DEM (one report per known DTC).
        for (i, dtc) in Dtc::ALL.iter().enumerate() {
            let active = diag.dtc_active_bits & (1u16 << i) != 0;
            self.dem.report(*dtc, active, t);
        }
        // NvM job: latch state + functional telemetry for the diagnostic API.
        self.nvm.request_write(NvmImage {
            dtc_latched_bits: diag.dtc_latched_bits,
            wipe_count: self.swc.wipe_count(),
        });
        let _ = self.nvm.process_jobs();
        // Status frame for the cluster display (frame 675).
        let fsm = self.swc.fsm_state();
        let signals = StatusSignals {
            wiper_active: self.last_outputs.motor.speed != wiper_swc::MotorSpeed::Off,
            speed_mode: self.last_outputs.motor.speed,
            pump_on: self.last_outputs.pump_on,
            fsm,
            stall_fault: self.swc.is_active(Dtc::MotorStall),
            int_pause: fsm == WiperFsm::IntPause,
        };
        let _ = self.bus.write(com::pack_status(&signals));
    }

    /// `Wiper_OnStalkChange` is a data-received runnable: forward only on
    /// content change (duplicates do not re-fire the RTE event).
    fn forward_stalk_event(&mut self) {
        let read = self.read_stalk();
        let Some(rx) = read.value else {
            return;
        };
        let tuple = (rx.mode, rx.sensitivity, rx.auto, rx.wash_btn, rx.valid);
        if self.forwarded_stalk != Some(tuple) {
            self.forwarded_stalk = Some(tuple);
            self.swc
                .Wiper_OnStalkChange(rx.mode, rx.sensitivity, rx.auto, rx.wash_btn, rx.valid);
        }
    }

    fn assemble_inputs(&self, t: u32) -> wiper_swc::Inputs {
        let rain = self.read_rain_at(t);
        let (rain_pct, rain_valid) = match rain.value {
            Some(rx) => (rx.pct, rx.valid && matches!(rain.status, ReadStatus::Valid)),
            None => (0, false),
        };
        let speed = self.read_speed_at(t);
        let (speed_kph, speed_valid) = match speed.value {
            Some(rx) => (
                rx.kph,
                rx.valid && matches!(speed.status, ReadStatus::Valid),
            ),
            None => (0, false),
        };
        self.iohwab.swc_inputs(
            self.ignition(),
            rain_pct,
            rain_valid,
            speed_kph,
            speed_valid,
        )
    }

    fn read_rain_at(&self, t: u32) -> autosar_rte::port::Received<RainRx> {
        RteReceiver::new(&self.ports.rain, Some(RAIN_RX_TIMEOUT_MS)).Rte_Read(t)
    }

    fn read_speed_at(&self, t: u32) -> autosar_rte::port::Received<SpeedRx> {
        RteReceiver::new(&self.ports.speed, Some(SPEED_RX_TIMEOUT_MS)).Rte_Read(t)
    }

    fn read_stalk_at(&self, t: u32) -> autosar_rte::port::Received<StalkRx> {
        RteReceiver::new(&self.ports.stalk, None).Rte_Read(t)
    }
}
