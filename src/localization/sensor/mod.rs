use alloc::vec::Vec;
use core::f64::consts::PI;

use nalgebra::{DefaultAllocator, Dim, Matrix, Matrix2, Scalar, Vector, Vector2, Vector3};
use rand_distr::num_traits::Pow;
use vexide::prelude::{AdiLineTracker, DistanceSensor, Float};

use crate::{localization::localization::StateRepresentation, state_machine::State};

fn normal_pdf(x: f64, mu: f64, sigma: f64) -> f64 {
    let exponent = -(x - mu) * (x - mu) / (2.0 * sigma * sigma);
    (1.0 / (sigma * (2.0 * core::f64::consts::PI).sqrt())) * exponent.exp()
}

pub struct WallDistanceSensor {
    distance: DistanceSensor,
    lines: Vec<(Vector2<f64>, Vector2<f64>)>,
    circles: Vec<(Vector2<f64>, f64)>,
    sensor_pose: Vector3<f64>,
}

pub trait Sensor {
    fn p(&self, x: &StateRepresentation) -> Option<f64>;
}

impl Sensor for WallDistanceSensor {
    fn p(&self, x: &StateRepresentation) -> Option<f64> {
        let measured = self.distance.distance().expect("Can't receive distance");

        if measured == 9999 {
            return None;
        }

        let measured_meters = measured as f64 / 1000.0;

        let min_predicted = 0.0;
        let max_predicted = 10.0;

        let predicted = 0.0;

        let std = 1.0
            / self
                .distance
                .distance_confidence()
                .expect("Can't receive confidence");

        Some(normal_pdf(measured_meters, predicted, std))
    }
}

impl Sensor for AdiLineTracker {
    fn p(&self, _x: &StateRepresentation) -> Option<f64> {
        todo!()
    }
}

pub fn normal_pdf_vec2(x: &Vector2<f64>, mean: &Vector2<f64>, covariance: &Matrix2<f64>) -> f64 {
    let n = 2.0; // Dimensionality, since we're working with Vector2

    // Calculate the determinant of sigma
    let det_sigma = covariance.determinant();

    // Calculate the inverse of sigma
    let sigma_inv = covariance.pseudo_inverse(0.0001).expect("Can't invert");

    // Calculate the exponent part: (x - mu)ᵀ Σ⁻¹ (x - mu)
    let diff = x - mean;
    let exponent = -0.5 * diff.transpose() * sigma_inv * diff;

    // Calculate the coefficient
    let coefficient = 1.0 / ((2.0 * PI).powf(n / 2.0) * det_sigma.sqrt());

    // Combine everything
    coefficient * exponent[(0, 0)].exp()
}

pub struct DummySensor {
    pub covariance: f64,
    pub mean: Vector2<f64>,
}

impl Sensor for DummySensor {
    fn p(&self, x: &StateRepresentation) -> Option<f64> {
        Option::from(normal_pdf(
            (Vector2::new(x.x, x.y) - self.mean).magnitude(),
            0.0,
            self.covariance,
        ))
    }
}
