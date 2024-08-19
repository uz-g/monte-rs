use nalgebra::Vector2;
use vexide::devices::smart::GpsSensor;

use crate::{
    config::GPS_ANGLE_DIFF_MAX,
    localization::{localization::StateRepresentation, sensor::Sensor},
    utils::{angle_difference, normal_pdf},
};

const GPS_BAD_BITFLAG: u32 = 0b0000_0100_0000_0000;

impl Sensor for GpsSensor {
    fn p(&self, x: &StateRepresentation) -> Option<f64> {
        let pose = self.pose().ok()?;

        let std = self.error().ok()? * 2.0;

        if angle_difference(pose.1, x.z) > GPS_ANGLE_DIFF_MAX
            || self.status().ok()? & GPS_BAD_BITFLAG != 0
        {
            todo!();
        } else {
            let position = Vector2::new(pose.0.x, pose.0.y);
            let predicted = Vector2::new(x.x, x.y);

            Some(normal_pdf((position - predicted).magnitude(), 0.0, std))
        }
    }
}
