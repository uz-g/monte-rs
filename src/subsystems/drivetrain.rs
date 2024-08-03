use core::time::Duration;

use vexide::prelude::*;

use crate::state_machine::*;

/// Example implementation of a drivetrain subsystem.
pub struct Drivetrain {
    left_motor: Motor,
    right_motor: Motor,
}

impl Drivetrain {
    pub fn new(left_motor: Motor, right_motor: Motor) -> Self {
        Self {
            left_motor,
            right_motor,
        }
    }
}

impl Subsystem<(f32, f32)> for Drivetrain {
    async fn run(&mut self, mut state: impl State<(f32, f32)>) {
        while let Some(output) = state.update().await {
            self.left_motor.set_voltage(output.0 as f64);
            self.right_motor.set_voltage(output.1 as f64);

            sleep(Duration::from_millis(10)).await;
        }
    }
}

pub struct TankDrive<'a> {
    pub controller: &'a mut Controller,
}

impl<'a> State<(f32, f32)> for TankDrive<'a> {
    async fn update(&mut self) -> Option<(f32, f32)> {
        Some((
            self.controller.left_stick.y().ok()? * 12.0,
            self.controller.right_stick.y().ok()? * 12.0,
        ))
    }
}
