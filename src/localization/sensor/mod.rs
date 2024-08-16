use nalgebra::Vector2;

use crate::{localization::localization::StateRepresentation, utils};

mod distance;
mod gps;
mod line_tracker;

pub trait Sensor {
    fn p(&self, x: &StateRepresentation) -> Option<f64>;
}

pub struct DummySensor {
    pub covariance: f64,
    pub mean: Vector2<f64>,
}

impl Sensor for DummySensor {
    fn p(&self, x: &StateRepresentation) -> Option<f64> {
        Option::from(utils::normal_pdf(
            (Vector2::new(x.x, x.y) - self.mean).magnitude(),
            0.0,
            self.covariance,
        ))
    }
}
