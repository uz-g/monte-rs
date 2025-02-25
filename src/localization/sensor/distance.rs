use nalgebra::{Rotation2, Vector2, Vector3};
use vexide::prelude::DistanceSensor;

use crate::{
    config::WALLS,
    localization::{localization::StateRepresentation, sensor::Sensor},
    utils,
};

pub struct WallDistanceSensor {
    distance: DistanceSensor,
    sensor_pose: Vector3<f64>,
}

impl WallDistanceSensor {
    pub fn new(distance: DistanceSensor, sensor_pose: Vector3<f64>) -> Self {
        Self {
            distance,
            sensor_pose,
        }
    }
}

impl Sensor for WallDistanceSensor {
    fn p(&self, x: &StateRepresentation) -> Option<f64> {
        if let Some(Some(measured)) = self.distance.distance().ok()
            && self.distance.get().ok()? > 50.0
        {
            let measured_meters = measured as f64 / 1000.0;

            // ... rest of the implementation ...

            let std = 0.025 * predicted / self.distance.get().ok()?;

            Some(utils::normal_pdf(measured_meters, predicted, std))
        } else {
            None
        }
    }
}
