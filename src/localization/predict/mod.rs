mod tank_pose_tracking;

use super::localization::StateRepresentation;

pub trait Predict {
    fn update(&mut self);

    fn predict(&self) -> StateRepresentation;
}
