use vexide::prelude::{Motor, RotationSensor};
use crate::actuator::motor_group::MotorGroup;

pub trait RotarySensor {
    fn pos(&self) -> f64;
}

impl RotarySensor for RotationSensor {
    fn pos(&self) -> f64 {
        self.angle().unwrap_or_default().as_radians()
    }
}

impl RotarySensor for Motor {
    fn pos(&self) -> f64 {
        self.position().unwrap_or_default().as_radians()
    }
}

impl RotarySensor for MotorGroup {
    fn pos(&self) -> f64 {
        self.position()
    }
}

pub struct TrackingWheel<T: RotarySensor> {
    sensor: T,
    offset: f64,
    diameter: f64,
    gearing: Option<f64>,
    last_length: f64,
}

impl<T: RotarySensor> TrackingWheel<T> {
    pub fn new(sensor: T, diameter: f64, offset: f64, gearing: Option<f64>) -> Self {
        Self {
            sensor,
            diameter,
            offset,
            gearing,
            last_length: 0.0,
        }
    }

    pub fn travel(&self) -> f64 {
        self.sensor.pos() * self.diameter * self.gearing.unwrap_or(1.0)
    }

    pub fn update(&mut self) -> f64 {
        let current_length = self.travel();

        let delta = current_length - self.last_length;

        self.last_length = current_length;

        delta
    }
}
