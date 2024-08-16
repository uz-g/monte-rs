use core::{f64::consts::PI, time::Duration};

use nalgebra::Vector2;
use uom::si::{f64::Length, length::inch};

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

pub const FIELD_MAX: f64 = 1.783;

pub const ANGLE_NOISE: f64 = PI / 20.0;
pub const DRIVE_NOISE: f64 = 0.1;

pub const FIELD_TAPES: [(Vector2<f64>, Vector2<f64>); 2] = [
    (Vector2::new(0.0, 0.0), Vector2::new(1.0, 0.0)),
    (Vector2::new(0.0, 0.0), Vector2::new(0.0, 1.0)),
];

pub const WALLS: [(Vector2<f64>, Vector2<f64>); 2] = [
    (Vector2::new(0.0, 0.0), Vector2::new(1.0, 0.0)),
    (Vector2::new(0.0, 0.0), Vector2::new(0.0, 1.0)),
];
