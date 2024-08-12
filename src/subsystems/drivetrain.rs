use alloc::sync::Arc;
use core::{ops::Add, time::Duration};

use uom::si::f64::Length;
use vexide::{
    core::{sync::Mutex, time::Instant},
    prelude::*,
};

use crate::{
    actuator::motor_group::MotorGroup,
    config::{localization_min_update_distance, LOCALIZATION_MIN_UPDATE_INTERVAL, NUM_PARTICLES},
    localization::{
        localization::{particle_filter::ParticleFilter, Localization, StateRepresentation},
        predict::tank_pose_tracking::TankPoseTracking,
    },
    sensor::rotary::TrackingWheel,
    state_machine::*,
};

/// Example implementation of a drivetrain subsystem.
pub struct Drivetrain {
    left_motor: Arc<Mutex<MotorGroup>>,
    right_motor: Arc<Mutex<MotorGroup>>,
    localization: Arc<Mutex<ParticleFilter<NUM_PARTICLES>>>,
    _localization_task: Task<()>,
}

impl Drivetrain {
    pub fn new(
        left_motor: Arc<Mutex<MotorGroup>>,
        right_motor: Arc<Mutex<MotorGroup>>,
        imu: InertialSensor,
        tracking_wheel_diameter: Length,
        drive_ratio: f64,
    ) -> Self {
        let localization = Arc::new(Mutex::new(ParticleFilter::new(
            TankPoseTracking::new(
                TrackingWheel::new(
                    left_motor.clone(),
                    tracking_wheel_diameter,
                    Option::from(drive_ratio),
                ),
                TrackingWheel::new(
                    right_motor.clone(),
                    tracking_wheel_diameter,
                    Option::from(drive_ratio),
                ),
                imu,
            ),
            LOCALIZATION_MIN_UPDATE_INTERVAL,
            localization_min_update_distance(),
        )));
        Self {
            localization: localization.clone(),
            _localization_task: spawn(async move {
                loop {
                    let now = Instant::now();

                    localization.lock().await.update().await;

                    sleep_until(now.add(Duration::from_millis(10)));
                }
            }),
            left_motor,
            right_motor,
        }
    }
}

impl Subsystem<StateRepresentation, (f64, f64)> for Drivetrain {
    async fn run(&mut self, mut state: impl State<StateRepresentation, (f64, f64)>) {
        state.init().await;
        while let Some(output) = state
            .update(self.localization.lock().await.pose_estimate())
            .await
        {
            let now = Instant::now();

            let _ = self.left_motor.lock().await.set_voltage(output.0);
            let _ = self.right_motor.lock().await.set_voltage(output.1);

            sleep_until(now.add(Duration::from_millis(10))).await;
        }
    }
}

pub struct TankDrive<'a> {
    controller: &'a Controller,
}

impl<'a> TankDrive<'a> {
    pub fn new(controller: &'a Controller) -> Self {
        TankDrive { controller }
    }
}

impl<'a> State<StateRepresentation, (f64, f64)> for TankDrive<'a> {
    async fn update(&mut self, _: StateRepresentation) -> Option<(f64, f64)> {
        Some((
            self.controller.left_stick.y().ok()? as f64 * 12.0,
            self.controller.right_stick.y().ok()? as f64 * 12.0,
        ))
    }
}

pub struct VoltageDrive {
    left_voltage: f64,
    right_voltage: f64,
}

impl VoltageDrive {
    pub fn new(left_voltage: f64, right_voltage: f64) -> Self {
        Self {
            left_voltage,
            right_voltage,
        }
    }
}

impl State<StateRepresentation, (f64, f64)> for VoltageDrive {
    async fn update(&mut self, _: StateRepresentation) -> Option<(f64, f64)> {
        Some((self.left_voltage as f64, self.right_voltage as f64))
    }
}
