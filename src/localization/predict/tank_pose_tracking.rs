use nalgebra::{Rotation2, Vector2};
use rand::{rngs::SmallRng, Rng, SeedableRng};
use rand_distr::Normal;
use uom::si::{
    angle::{degree, radian},
    f64::Angle,
};
use vexide::prelude::InertialSensor;

use crate::{
    localization::localization::StateRepresentation,
    sensor::rotary::{RotarySensor, TrackingWheel},
};

pub struct TankPoseTracking<T: RotarySensor> {
    left_side: TrackingWheel<T>,
    right_side: TrackingWheel<T>,
    orientation: InertialSensor,
    left_delta: f64,
    right_delta: f64,
    heading: Rotation2<f64>,
    rng: SmallRng,
}

impl<T: RotarySensor> TankPoseTracking<T> {
    pub fn new(
        left_side: TrackingWheel<T>,
        right_side: TrackingWheel<T>,
        orientation: InertialSensor,
    ) -> Self {
        Self {
            left_side,
            right_side,
            orientation,
            left_delta: 0.0,
            right_delta: 0.0,
            heading: Rotation2::new(0.0),
            rng: SmallRng::seed_from_u64(0),
        }
    }

    pub async fn update(&mut self) {
        self.left_delta = self.left_side.update().await;
        self.right_delta = self.right_side.update().await;
        self.heading = self.orientation();
    }

    pub fn predict(&mut self) -> StateRepresentation {
        let left_noisy = self
            .rng
            .sample(Normal::new(self.left_delta, 0.05 * self.left_delta).unwrap());
        let right_noisy = self
            .rng
            .sample(Normal::new(self.right_delta, 0.05 * self.right_delta).unwrap());

        let mean = left_noisy + right_noisy / 2.0;

        let local = self.heading * Vector2::new(mean, 0.);

        StateRepresentation::new(local.x, local.y, 0.0)
    }

    pub(crate) fn orientation(&self) -> Rotation2<f64> {
        Rotation2::new(
            Angle::new::<degree>(-self.orientation.heading().unwrap_or_default()).get::<radian>(),
        )
    }
}
