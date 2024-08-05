use core::time::Duration;
use vexide::devices::smart::motor::MotorError;
use vexide::prelude::*;

use crate::state_machine::{State, Subsystem};

pub struct Lift {
    motor: Motor,
}

impl Lift {
    pub fn new(motor: Motor) -> Self {
        Self { motor }
    }
}

impl Subsystem<Result<Position, MotorError>, f64> for Lift {
    async fn run(&mut self, mut state: impl crate::state_machine::State<Result<Position, MotorError>, f64>) {
        state.init().await;

        while let Some(output) = state.update(self.motor.position()).await {
            let _ = self.motor.set_voltage(output as f64);

            sleep(Duration::from_millis(10)).await;
        }
    }
}

pub struct TeleopArm<'a> {
    controller: &'a Controller,
}

impl<'a> TeleopArm<'a> {
    pub fn new(controller: &'a Controller) -> Self {
        Self { controller }
    }
}

impl<'a> State<Result<Position, MotorError>, f64> for TeleopArm<'a> {
    async fn update(&mut self, _: Result<Position, MotorError>) -> Option<f64> {
        let mut power = 0.0;

        if let Ok(pressed) = self.controller.right_trigger_1.is_pressed() {
            if pressed {
                power += 12.0;
            }
        } else {
            return None;
        }

        if let Ok(pressed) = self.controller.right_trigger_2.is_pressed() {
            if pressed {
                power -= 12.0;
            }
        } else {
            return None;
        }

        Some(power)
    }
}
