use nalgebra::Rotation2;
use uom::si::{angle::*, f64::Angle};
use vexide::prelude::InertialSensor;

pub trait Orientation {
    fn get_orientation(&self) -> Option<Rotation2<f64>>;

    fn set_orientation(&mut self, rotation: Rotation2<f64>);
}

impl Orientation for InertialSensor {
    fn get_orientation(&self) -> Option<Rotation2<f64>> {
        match self.heading() {
            Ok(t) => Some(Rotation2::new(Angle::new::<degree>(t).get::<radian>())),
            _ => None,
        }
    }

    fn set_orientation(&mut self, rotation: Rotation2<f64>) {
        let _ = self.set_heading(Angle::new::<radian>(rotation.angle()).get::<degree>());
    }
}
