use alloc::vec::Vec;

use uom::si::{angular_velocity::*, f64::AngularVelocity};
use vexide::prelude::Motor;

pub struct MotorGroup {
    motors: Vec<Motor>,
}

impl MotorGroup {
    pub fn new(motors: Vec<Motor>) -> Self {
        Self { motors }
    }

    pub fn set_voltage(&mut self, voltage: f64) {
        for motor in self.motors.iter_mut() {
            let _ = motor.set_voltage(voltage);
        }
    }

    pub fn set_velocity(&mut self, velocity: AngularVelocity) {
        for motor in self.motors.iter_mut() {
            let _ = motor.set_velocity(velocity.get::<revolution_per_minute>() as i32);
        }
    }

    pub fn position(&self) -> f64 {
        self.motors
            .iter()
            .filter_map(|motor| motor.position().map(|x| x.as_radians()).ok())
            .sum()
    }
}
