use core::f64::consts::{PI, TAU};

use vexide::core::float::Float;

pub fn normal_pdf(x: f64, mu: f64, sigma: f64) -> f64 {
    let exponent = -(x - mu) * (x - mu) / (2.0 * sigma * sigma);
    (1.0 / (sigma * (2.0 * PI).sqrt())) * exponent.exp()
}

/// Finds the minimum signed distance between 2 angles
///
/// # Examples
/// ```
/// let angle1: f64;
/// let angle2: f64;
///
/// // angle1 - angle2
/// let difference = angle_difference(angle1, angle2);
/// ```
pub fn angle_difference(x: f64, y: f64) -> f64 {
    (x - y + PI) % TAU - PI
}

// pub fn normal_pdf_vec2(x: &Vector2<f64>, mean: &Vector2<f64>, covariance: &Matrix2<f64>) -> f64 {
//     let n = 2.0; // Dimensionality, since we're working with Vector2
//
//     // Calculate the determinant of sigma
//     let det_sigma = covariance.determinant();
//
//     // Calculate the inverse of sigma
//     let sigma_inv = covariance.pseudo_inverse(0.0001).expect("Can't invert");
//
//     // Calculate the exponent part: (x - mu)ᵀ Σ⁻¹ (x - mu)
//     let diff = x - mean;
//     let exponent = -0.5 * diff.transpose() * sigma_inv * diff;
//
//     // Calculate the coefficient
//     let coefficient = 1.0 / ((2.0 * PI).powf(n / 2.0) * det_sigma.sqrt());
//
//     // Combine everything
//     coefficient * exponent[(0, 0)].exp()
// }
