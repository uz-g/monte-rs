use alloc::borrow::ToOwned;
use alloc::boxed::Box;
use alloc::sync::Arc;
use nalgebra::{Rotation2, Vector2};
use rand::{Rng, SeedableRng};
use rand_distr::Normal;
use rand::rngs::SmallRng;
use crate::localization::localization::StateRepresentation;
use crate::localization::predict::Predict;
use crate::sensor::orientation::Orientation;
use crate::sensor::rotary::{RotarySensor, TrackingWheel};
use rand_distr::Distribution;

pub struct TankPoseTracking<T: RotarySensor> {
    left_side: TrackingWheel<T>,
    right_side: TrackingWheel<T>,
    orientation: Box<dyn Orientation>,
    left_delta: f64,
    right_delta: f64,
    heading: Rotation2<f64>,
    rng: Arc<SmallRng>
}

impl<T: RotarySensor> TankPoseTracking<T> {
    fn new(left_side: TrackingWheel<T>,
           right_side: TrackingWheel<T>,
           orientation: impl Orientation + 'static) -> Self {
        Self {
            left_side,
            right_side,
            orientation: Box::new(orientation),
            left_delta: 0.0,
            right_delta: 0.0,
            heading: Rotation2::new(0.0),
            rng: Arc::new(SmallRng::seed_from_u64(0)),
        }
    }
}

impl<T: RotarySensor> Predict for TankPoseTracking<T> {
    fn update(&mut self) {
        self.left_delta = self.left_side.update();
        self.right_delta = self.right_side.update();
        self.heading = self.orientation.get_orientation().unwrap_or_default();
    }

    fn predict(&mut self) -> StateRepresentation {
        let left_noisy = self.rng.sample(Normal::new(self.left_delta, 0.05 * self.left_delta));
        let right_noisy = self.rng.sample(Normal::new(self.right_delta, 0.05 * self.right_delta));

        let mean = left_noisy + right_noisy / 2.0;

        let local = self.heading * Vector2::new(mean, 0.);

        StateRepresentation::new(local.x, local.y, 0.0)
    }
}