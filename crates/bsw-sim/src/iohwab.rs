//! `IoHwAb` — input/output hardware abstraction of the simulated ECU.
//!
//! PWM/DIO outputs (`Wiper_Motor`, `Wiper_Pump`) are latched here and applied to
//! the plant; the plant's feedback (arm angle, arm speed, park switch, motor
//! current, battery voltage) is sampled ADC-style into this module once per task
//! cycle and re-exposed as the SWC's [`Inputs`] fields ([`IoHwAb::swc_inputs`]).
//! The logical/communication inputs (stalk, rain, speed, ignition) do *not*
//! belong here — they arrive over COM/mode ports and are merged by the ECU.

#![forbid(unsafe_code)]

use plant_sim::PlantOutput;
use wiper_swc::{Ignition, Inputs, MotorCommand, Outputs};

/// The latched hardware image: last commanded PWM/DIO outputs and the most
/// recent ADC sample (one per task cycle — sampled at the task boundary).
#[derive(Debug, Clone, Copy)]
pub struct IoHwAb {
    /// Motor PWM profile + duty (frame output, 1:1 passthrough, REQ-WIP-018).
    motor: MotorCommand,
    /// Washer pump DIO.
    pump_on: bool,
    /// ADC sample of the actuator/sensor feedback.
    adc: PlantOutput,
}

impl Default for IoHwAb {
    fn default() -> Self {
        Self::new()
    }
}

impl IoHwAb {
    /// Hardware at rest: PWM off, pump off, plant idle at park.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            motor: MotorCommand {
                speed: wiper_swc::MotorSpeed::Off,
                duty_pct: 0,
            },
            pump_on: false,
            adc: PlantOutput {
                angle_deg: 0,
                arm_speed_dps: 0,
                park_switch: true,
                current_da: 0,
                battery_v01: 138,
            },
        }
    }

    /// `Rte_Write_Wiper_Motor` / `Rte_Write_Wiper_Pump`: latch both outputs.
    pub fn write_outputs(&mut self, outputs: Outputs) {
        self.motor = outputs.motor;
        self.pump_on = outputs.pump_on;
    }

    /// Sample the plant feedback (ADC read at the task boundary).
    pub fn capture_adc(&mut self, snapshot: PlantOutput) {
        self.adc = snapshot;
    }

    /// PWM currently driven into the motor bridge.
    #[must_use]
    pub const fn motor_command(&self) -> MotorCommand {
        self.motor
    }

    /// Washer pump DIO state.
    #[must_use]
    pub const fn pump_on(&self) -> bool {
        self.pump_on
    }

    /// Last sampled plant feedback.
    #[must_use]
    pub const fn adc(&self) -> &PlantOutput {
        &self.adc
    }

    /// Compose the SWC input bundle for one `Wiper_Run10ms` call: ADC-sampled
    /// physical signals plus the logical inputs COM/mode ports provide.
    #[must_use]
    pub const fn swc_inputs(
        &self,
        ignition: Ignition,
        rain_pct: u8,
        rain_valid: bool,
        speed_kph: u16,
        speed_valid: bool,
    ) -> Inputs {
        Inputs {
            ignition,
            rain_pct,
            rain_valid,
            speed_kph,
            speed_valid,
            park_switch: self.adc.park_switch,
            motor_current_dA: self.adc.current_da,
            battery_v0p1: self.adc.battery_v01,
            arm_angle_deg: self.adc.angle_deg,
            arm_speed_dps: self.adc.arm_speed_dps,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use plant_sim::PlantOutput;

    /// verifies: REQ-WIP-018 — the `IoHwAb` maps the motor command 1:1 to the PWM
    /// output; LOW profile carries 70 %, HIGH 100 %, OFF 0 % duty.
    #[test]
    fn pwm_passthrough_keeps_duty_profile() {
        let mut hw = IoHwAb::new();
        assert_eq!(hw.motor_command().duty_pct, 0);
        hw.write_outputs(Outputs {
            motor: MotorCommand {
                speed: wiper_swc::MotorSpeed::Low,
                duty_pct: 70,
            },
            pump_on: false,
        });
        assert_eq!(hw.motor_command().duty_pct, 70);
        hw.write_outputs(Outputs {
            motor: MotorCommand {
                speed: wiper_swc::MotorSpeed::High,
                duty_pct: 100,
            },
            pump_on: true,
        });
        assert_eq!(hw.motor_command().duty_pct, 100);
        assert!(hw.pump_on());
    }

    /// verifies: REQ-WIP-012 (feedback path: ADC sample → SWC current/angle
    /// inputs; `STALL_CURRENT_DA` is evaluated against the sampled current)
    #[test]
    fn adc_sample_flows_into_swc_inputs() {
        let mut hw = IoHwAb::new();
        hw.write_outputs(Outputs {
            motor: MotorCommand {
                speed: wiper_swc::MotorSpeed::Off,
                duty_pct: 0,
            },
            pump_on: false,
        });
        hw.capture_adc(PlantOutput {
            angle_deg: 47,
            arm_speed_dps: 620,
            park_switch: false,
            current_da: 193,
            battery_v01: 138,
        });
        let in_ = hw.swc_inputs(Ignition::On, 40, true, 90, true);
        assert_eq!(in_.motor_current_dA, 193);
        assert_eq!(in_.arm_angle_deg, 47);
        assert_eq!(in_.arm_speed_dps, 620);
        assert!(!in_.park_switch);
        assert_eq!(in_.battery_v0p1, 138);
        assert_eq!(in_.rain_pct, 40);
        assert_eq!(in_.speed_kph, 90);
    }
}
