use core::time::Duration;

use uom::si::{f64::Length, length::inch};

pub const NUM_PARTICLES: usize = 100;

pub fn wheel_diameter() -> Length {
    Length::new::<inch>(2.75)
}

pub const DRIVE_RATIO: f64 = 1.0;

pub static LOCALIZATION_MIN_UPDATE_INTERVAL: Duration = Duration::from_millis(250);

pub fn localization_min_update_distance() -> Length {
    Length::new::<inch>(2.0)
}
