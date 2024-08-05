use alloc::vec::Vec;

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

    pub fn position(&self) -> f64 {
        self.motors
            .iter()
            .filter_map(|motor| motor.position().map(|x| x.as_radians()).ok())
            .sum()
    }
}
