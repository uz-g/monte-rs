use alloc::boxed::Box;
use vexide::{
    prelude::*,
    time::Instant,
};
use simba::scalar::Real;

use motion_profiling::motion_profile::MotionProfile;
use nalgebra::{Matrix3, SimdComplexField};
use uom::{
    num_traits::{real::Real, Pow},
    si::{
        angular_velocity::radian_per_second, f64::AngularVelocity, length::meter,
        velocity::meter_per_second,
    },
};
use vexide::core::time::Instant;

use crate::{
    config::{track_width, wheel_diameter},
    localization::localization::StateRepresentation,
    state_machine::State,
    utils::angle_difference,
};

pub struct Ramsete {
    zeta: f64,
    beta: f64,
    motion_profile: Box<dyn MotionProfile>,
    start_time: Instant,
}

#[derive(Debug)]
pub enum RamseteError {
    InvalidBeta,
}

impl Ramsete {
    pub fn try_new(
        zeta: f64,
        beta: f64,
        motion_profile: Box<dyn MotionProfile>,
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

impl<'a> State<StateRepresentation, (AngularVelocity, AngularVelocity)> for Ramsete {fn update(&mut self, i: &StateRepresentation) -> Option<(AngularVelocity, AngularVelocity)> {
    let command = self.motion_profile.get(Instant::now() - self.start_time)?;

    let error = Matrix3::new(
        Real::cos(i.z),
        Real::sin(i.z),
        0.0,
        -Real::sin(i.z),
        Real::cos(i.z),
        0.0,
        0.0,
        0.0,
        1.0,
    ) * (command.desired_pose - i);

    let k = 2.0
        * self.zeta
        * Real::sqrt((command.desired_angular.get::<radian_per_second>().pow(2) as f64
            + self.beta * command.desired_velocity.get::<meter_per_second>().pow(2) as f64));

        let velocity_commanded =
            command.desired_velocity.get::<meter_per_second>() * error.z.cos() + k * error.x;
        let angular_wheel_velocity_commanded = (command.desired_angular.get::<radian_per_second>()
            + k * angle_difference(error.z, 0.0)
            + self.beta
                * command.desired_velocity.get::<meter_per_second>()
                * error.z.simd_sinc()
                * error.y)
            * track_width().get::<meter>()
            / 2.0;

        Some((
            AngularVelocity::new::<radian_per_second>(
                (velocity_commanded - angular_wheel_velocity_commanded)
                    / (wheel_diameter().get::<meter>() / 2.0),
            ),
            AngularVelocity::new::<radian_per_second>(
                (velocity_commanded + angular_wheel_velocity_commanded)
                    / (wheel_diameter().get::<meter>() / 2.0),
            ),
        ))
    }
}

#[cfg(test)]
mod tests {
    use core::time::Duration;

    use motion_profiling::motion_profile::MotionCommand;

    use super::*;

    struct DummyMotionProfile;

    impl MotionProfile for DummyMotionProfile {
        fn duration(&self) -> Duration {
            Duration::new(0, 0)
        }

        fn get(&mut self, t: Duration) -> Option<MotionCommand> {
            None
        }
    }

    #[test]
    fn build_ramsete() {
        let mut mp = DummyMotionProfile;
        let ramsete = Ramsete::try_new(1.0, 0.5, &mut mp);
        assert!(ramsete.is_ok());
    }
}
