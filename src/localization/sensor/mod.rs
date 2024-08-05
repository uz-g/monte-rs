use nalgebra::Vector3;
use vexide::prelude::DistanceSensor;

use crate::localization::localization::StateRepresentation;

pub trait Sensor {
    fn p(&self, x: &StateRepresentation) -> Option<f64>;
}

impl Sensor for DistanceSensor {
    fn p(&self, x: &StateRepresentation) -> Option<f64> {
        todo!();
    }
}
