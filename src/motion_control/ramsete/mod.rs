use core::f64::consts::PI;

use motion_profiling::motion_profile::MotionProfile;
use nalgebra::{Matrix3, SimdComplexField};
use uom::{
    num_traits::{real::Real, Pow},
    si::{angular_velocity::radian_per_second, length::meter, velocity::meter_per_second},
};
use vexide::core::time::Instant;

use crate::{
    config::track_width, localization::localization::StateRepresentation, state_machine::State,
};

pub struct Ramsete<'a> {
    zeta: f64,
    beta: f64,
    motion_profile: &'a mut dyn MotionProfile,
    start_time: Instant,
}

pub enum RamseteError {
    InvalidBeta,
}

impl<'a> Ramsete<'a> {
    fn try_new(
        zeta: f64,
        beta: f64,
        motion_profile: &'a mut dyn MotionProfile,
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
    }

    fn update(&mut self, i: &StateRepresentation) -> Option<(f64, f64)> {
        let command = self.motion_profile.get(Instant::now() - self.start_time)?;

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

#[cfg(test)]
mod tests {
    use core::time::Duration;

    use motion_profiling::motion_profile::MotionCommand;

    use super::*;

    struct DummyMotionProfile;

    impl MotionProfile for DummyMotionProfile {
        fn get_duration(&self) -> Duration {
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
