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
    fn new(distance: DistanceSensor, sensor_pose: Vector3<f64>) -> Self {
        Self {
            distance,
            sensor_pose,
        }
    }
}

impl Sensor for WallDistanceSensor {
    fn p(&self, x: &StateRepresentation) -> Option<f64> {
        let measured = self.distance.distance().ok()?;

        if measured == 9999 || self.distance.relative_size().ok()? < 50 {
            None
        } else {
            let measured_meters = measured as f64 / 1000.0;

            let sensor_position =
                Rotation2::new(-x.z) * Vector2::new(self.sensor_pose.x, self.sensor_pose.y);
            let orientation = Rotation2::new(self.sensor_pose.z + x.z);

            let predicted = WALLS
                .iter()
                .filter_map(|line| Some(0.0))
                .min_by(|a, b| a.partial_cmp(b).unwrap())?;

            let std = 0.025 * predicted / self.distance.distance_confidence().ok()?;

            Some(utils::normal_pdf(measured_meters, predicted, std))
        }
    }
}
