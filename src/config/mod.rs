use core::{f64::consts::PI, time::Duration};
use vexide::prelude::*;
use vexide::{
    prelude::*,
    geometry::Point2,  // If Point2 is available in prelude, remove this line
};
// Remove the duplicate import
// use vexide::devices::geometry::Point2;

use crate::localization::localization::StateRepresentation;

pub const TELEMETRY_ENABLED: bool = false;
pub const NUM_PARTICLES: usize = 100;

pub fn wheel_diameter() -> Length {
    Length::new::<inch>(2.75)
}

pub const DRIVE_RATIO: f64 = 4.0;

pub static LOCALIZATION_MIN_UPDATE_INTERVAL: Duration = Duration::from_millis(5000);

pub fn localization_min_update_distance() -> Length {
    Length::new::<inch>(2.0)
}

pub const FIELD_SIZE: f64 = 3.566414;
pub const FIELD_MAX: f64 = FIELD_SIZE / 2.0;

pub const ANGLE_NOISE: f64 = PI / 20.0;
pub const DRIVE_NOISE: f64 = 0.1;

pub const LINE_SENSOR_THRESHOLD: f64 = 0.2;

pub const FIELD_TAPES: [(Vector2<f64>, Vector2<f64>); 2] = [
    (Vector2::new(0.0, 0.0), Vector2::new(1.0, 0.0)),
    (Vector2::new(0.0, 0.0), Vector2::new(0.0, 1.0)),
];

pub const WALLS: [(Vector2<f64>, Vector2<f64>); 2] = [
    (Vector2::new(0.0, 0.0), Vector2::new(1.0, 0.0)),
    (Vector2::new(0.0, 0.0), Vector2::new(0.0, 1.0)),
];

pub const GPS_ANGLE_DIFF_MAX: f64 = PI / 8.0;

pub const LIFT_RATIO: f64 = 8.0;
pub const INTAKE_RATIO: f64 = 16.5 / 6.0;

pub fn track_width() -> Length {
    Length::new::<inch>(10.0)
}

pub fn distance_threshold() -> Length {
    Length::new::<inch>(1.0)
}

pub fn get_gps_offset() -> Point2<f64> {
    Point2::new(0.2, 0.2)
}

pub fn get_distance_1_offset() -> StateRepresentation {
    StateRepresentation::new(0.0, 0.0, 0.0)
}

pub fn get_distance_2_offset() -> StateRepresentation {
    StateRepresentation::new(0.0, 0.0, 0.0)
}

pub fn get_distance_3_offset() -> StateRepresentation {
    StateRepresentation::new(0.0, 0.0, 0.0)
}

pub fn get_line_1_offset() -> Vector2<f64> {
    Vector2::new(0.0, 0.0)
}
