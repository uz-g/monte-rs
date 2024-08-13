use core::f64::consts::PI;

use nalgebra::{DefaultAllocator, Dim, Matrix, Matrix2, Scalar, Vector, Vector2};
use rand_distr::num_traits::Pow;
use vexide::prelude::{AdiLineTracker, DistanceSensor, Float};

use crate::localization::localization::StateRepresentation;

fn normal_pdf(x: f64, mu: f64, sigma: f64) -> f64 {
    let exponent = -(x - mu) * (x - mu) / (2.0 * sigma * sigma);
    (1.0 / (sigma * (2.0 * core::f64::consts::PI).sqrt())) * exponent.exp()
}

pub trait Sensor {
    fn p(&self, x: &StateRepresentation) -> Option<f64>;
}

impl Sensor for DistanceSensor {
    fn p(&self, _x: &StateRepresentation) -> Option<f64> {
        let measured = 0.0;
        let predicted = 0.0;
        let std = 0.0;

        Some(normal_pdf(measured, predicted, std));
        todo!()
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

// pub fn normal_pdf_vec<T, R, C, S>(x: Vector<T, R, S>, mean: Vector<T, R, S>, covariance: Matrix<T, R, C, S>) -> f64
// where T: Scalar + core::ops::SubAssign + core::ops::Sub,
//       R: Dim,
//       C: Dim,
//       S: Default + nalgebra::Storage<T, R>, DefaultAllocator: nalgebra::allocator::Allocator<R> {
//     let left = 1.0/((2.0 * PI).pow(x.size()) * covariance.determinant()).sqrt() * 1.0;
//     let right = (-0.5 * (x - mean).transpose() * covariance);
//
//     todo!();
// }

pub struct DummySensor {
    pub(crate) covariance: Matrix2<f64>,
    pub(crate) mean: Vector2<f64>,
}

impl Sensor for DummySensor {
    fn p(&self, x: &StateRepresentation) -> Option<f64> {
        Option::from(normal_pdf_vec2(
            &Vector2::new(x.x, x.y),
            &self.mean,
            &self.covariance,
        ))
    }
}
