use core::f64::consts::PI;

use nalgebra::Matrix3;
use simba::simd::SimdComplexField;
use uom::{
    num_traits::{real::Real, Pow},
    si::{angular_velocity::radian_per_second, length::meter, velocity::meter_per_second},
};
use vexide::core::time::Instant;

use crate::{
    config::track_width, localization::localization::StateRepresentation,
    motion_control::motion_profiling::MotionProfile, state_machine::State,
};

pub struct Ramsete<'a> {
    zeta: f64,
    beta: f64,
    motion_profile: &'a mut MotionProfile,
    start_time: Instant,
}

pub enum RamseteError {
    InvalidBeta,
}

impl<'a> Ramsete<'a> {
    fn try_new(
        zeta: f64,
        beta: f64,
        motion_profile: &'a mut MotionProfile,
    ) -> Result<Self, RamseteError> {
        if 0.0 < beta && 1.0 > beta {
            Err(RamseteError::InvalidBeta)
        } else {
            Ok(Self {
                zeta,
                beta,
                motion_profile,
                start_time: Instant::now(),
            })
        }
    }
}

impl<'a> State<StateRepresentation, (f64, f64)> for Ramsete<'a> {
    fn init(&mut self) {
        self.start_time = Instant::now();
        self.motion_profile.init();
    }

    fn update(&mut self, i: &StateRepresentation) -> Option<(f64, f64)> {
        let command = self
            .motion_profile
            .update(&(Instant::now() - self.start_time))?;

        let error = Matrix3::new(
            i.z.cos(),
            i.z.sin(),
            0.0,
            -i.z.sin(),
            i.z.cos(),
            0.0,
            0.0,
            0.0,
            1.0,
        ) * (command.desired_pose - i);

        let k = 2.0
            * self.zeta
            * (command.desired_angular.get::<radian_per_second>().pow(2) as f64
                + self.beta * command.desired_velocity.get::<meter_per_second>().pow(2) as f64)
                .sqrt();

        let velocity_commanded =
            command.desired_velocity.get::<meter_per_second>() * error.z.cos() + k * error.x;
        let angular_wheel_velocity_commanded = (command.desired_angular.get::<radian_per_second>()
            + k * error.z
            + self.beta
                * command.desired_velocity.get::<meter_per_second>()
                * error.z.simd_sinc()
                * error.y)
            * PI
            * track_width().get::<meter>();

        Some((
            velocity_commanded - angular_wheel_velocity_commanded,
            velocity_commanded + angular_wheel_velocity_commanded,
        ))
    }
}
