use alloc::{sync::Arc, vec::Vec};
use core::{f64::consts::TAU, ops::Add, time::Duration};

use nalgebra::{Matrix3, Vector2};
use uom::si::f64::{AngularVelocity, Length};
use vexide::{
    core::{sync::Mutex, time::Instant},
    devices::{smart::GpsSensor, PortError},
    prelude::*,
};

use crate::{
    actuator::{motor_group::MotorGroup, telemetry::Telemetry},
    config::{
        distance_threshold, localization_min_update_distance, ANGLE_NOISE, DRIVE_NOISE, FIELD_MAX,
        LINE_SENSOR_THRESHOLD, LOCALIZATION_MIN_UPDATE_INTERVAL, NUM_PARTICLES, TELEMETRY_ENABLED,
    },
    localization::{
        localization::{particle_filter::ParticleFilter, Localization, StateRepresentation},
        predict::tank_pose_tracking::TankPoseTracking,
        sensor::{distance::WallDistanceSensor, line_tracker::LineTrackerSensor},
    },
    sensor::rotary::TrackingWheel,
    state_machine::*,
};

/// Example implementation of a drivetrain subsystem.
#[allow(dead_code)]
pub struct Drivetrain {
    left_motor: Arc<Mutex<MotorGroup>>,
    right_motor: Arc<Mutex<MotorGroup>>,
    localization: Arc<Mutex<ParticleFilter<NUM_PARTICLES>>>,
    _localization_task: Task<()>,
    telemetry: Telemetry,
}

impl Drivetrain {
    pub async fn new(
        left_motor: Arc<Mutex<MotorGroup>>,
        right_motor: Arc<Mutex<MotorGroup>>,
        imu: InertialSensor,
        tracking_wheel_diameter: Length,
        drive_ratio: f64,
        telemetry: Telemetry,
        distance_sensors: Vec<(DistanceSensor, StateRepresentation)>,
        line_sensors: Vec<(AdiLineTracker, Vector2<f64>)>,
        gps: Result<GpsSensor, PortError>,
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
                DRIVE_NOISE,
                ANGLE_NOISE,
            )
            .await,
            LOCALIZATION_MIN_UPDATE_INTERVAL,
            localization_min_update_distance(),
        )));

        {
            let mut loc_lock = localization.lock().await;

            for (sensor, pose) in distance_sensors {
                loc_lock.add_sensor(WallDistanceSensor::new(sensor, pose));
            }

            for (sensor, pose) in line_sensors {
                loc_lock.add_sensor(LineTrackerSensor::new(
                    sensor,
                    pose,
                    LINE_SENSOR_THRESHOLD,
                    distance_threshold(),
                ));
            }

            if let Ok(gps_ok) = gps {
                loc_lock.add_sensor(gps_ok);
            }

            // loc_lock.add_sensor(DummySensor {
            //     covariance: 0.5,
            //     mean: Default::default(),
            // });

            loc_lock.init_uniform(
                &StateRepresentation::new(-FIELD_MAX, -FIELD_MAX, 0.0),
                &StateRepresentation::new(FIELD_MAX, FIELD_MAX, TAU),
            );
        }

        Self {
            localization: localization.clone(),
            telemetry: telemetry.clone(),
            _localization_task: spawn(async move {
                loop {
                    let now = Instant::now();

                    {
                        let mut loc = localization.lock().await;

                        loc.update().await;

                        if TELEMETRY_ENABLED {
                            telemetry.send_json(loc.get_estimates().to_vec()).await;
                            telemetry.send("\n".as_bytes()).await;
                        }
                    }

                    sleep_until(now.add(Duration::from_millis(10))).await;
                }
            }),
            left_motor,
            right_motor,
        }
    }

    pub async fn init_norm(&mut self, mean: &StateRepresentation, covariance: &Matrix3<f64>) {
        self.localization.lock().await.init_norm(mean, covariance);
    }

    pub async fn run_velocity(
        &mut self,
        mut state: impl State<StateRepresentation, (AngularVelocity, AngularVelocity)>,
    ) {
        state.init();
        loop {
            let position;

            {
                position = self.localization.lock().await.pose_estimate();
            }

            if let Some(output) = state.update(&position) {
                let now = Instant::now();

                self.left_motor.lock().await.set_velocity(output.0);
                self.right_motor.lock().await.set_velocity(output.1);

                sleep_until(now.add(Duration::from_millis(10))).await;
            } else {
                return;
            }
        }
    }

    pub async fn run(&mut self, mut state: impl State<StateRepresentation, (f64, f64)>) {
        state.init();
        loop {
            let position;

            {
                position = self.localization.lock().await.pose_estimate();
            }

            if let Some(output) = state.update(&position) {
                let now = Instant::now();

                // println!("updateD, {:?}", now);

                self.left_motor.lock().await.set_voltage(output.0);
                self.right_motor.lock().await.set_voltage(output.1);

                sleep_until(now.add(Duration::from_millis(10))).await;
            } else {
                return;
            }
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
    fn update(&mut self, _: &StateRepresentation) -> Option<(f64, f64)> {
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
    fn update(&mut self, _: &StateRepresentation) -> Option<(f64, f64)> {
        Some((self.left_voltage as f64, self.right_voltage as f64))
    }
}
