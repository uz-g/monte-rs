use core::time::Duration;

use uom::si::f64::{AngularVelocity, Velocity};

use crate::{localization::localization::StateRepresentation, state_machine::State};

pub struct MotionCommand {
    pub desired_velocity: Velocity,
    pub desired_angular: AngularVelocity,
    pub desired_pose: StateRepresentation,
}

pub type MotionProfile = dyn State<Duration, MotionCommand>;
