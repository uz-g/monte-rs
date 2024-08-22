use nalgebra::{Rotation2, Vector2};
use uom::si::{f64::Length, length::meter};
use vexide::prelude::AdiLineTracker;

use crate::{
    config::FIELD_TAPES,
    localization::{localization::StateRepresentation, sensor::Sensor},
};

pub struct LineTrackerSensor {
    line_tracker: AdiLineTracker,
    position: Vector2<f64>,
    line_sensor_threshold: f64,
    distance_threshold: Length,
}

impl LineTrackerSensor {
    pub fn new(
        line_tracker: AdiLineTracker,
        position: Vector2<f64>,
        line_sensor_threshold: f64,
        distance_threshold: Length,
    ) -> Self {
        Self {
            line_tracker,
            position,
            line_sensor_threshold,
            distance_threshold,
        }
    }
}

impl Sensor for LineTrackerSensor {
    fn p(&self, x: &StateRepresentation) -> Option<f64> {
        let measured = self.line_tracker.reflectivity().ok()? > self.line_sensor_threshold;
        let sensor_position = Rotation2::new(-x.z) * self.position + Vector2::new(x.x, x.y);

        let predicted = FIELD_TAPES
            .into_iter()
            .map(|tape| {
                ((tape.0.y - tape.1.y) * sensor_position.y
                    - (tape.0.x - tape.1.x) * sensor_position.x
                    + tape.1.x * tape.0.y
                    - tape.1.y * tape.0.x)
                    / (tape.1 - tape.0).magnitude()
            })
            .min_by(|a, b| a.partial_cmp(b).unwrap())?
            < self.distance_threshold.get::<meter>();

        if measured && predicted {
            Some(0.9)
        } else if measured && !predicted {
            Some(0.1)
        } else if !measured && predicted {
            Some(0.1)
        } else {
            Some(0.9)
        }
    }
}
