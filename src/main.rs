#![no_main]
#![no_std]
#![feature(future_join)]
#![feature(async_closure)]
#![feature(duration_millis_float)]
#![feature(let_chains)]
extern crate alloc;
extern crate uom;

use alloc::{boxed::Box, ffi::CString, format, sync::Arc, vec};
use core::{future::join, panic::PanicInfo, time::Duration};

use futures::{select_biased, FutureExt};
use motion_profiling::combined_mp::CombinedMP;
use nalgebra::Matrix3;
use subsystems::drivetrain::VoltageDrive;
use uom::si::length::meter;
use vexide::{
    core::sync::Mutex,
    devices::{
        controller::ControllerId,
        screen::{Text, TextSize},
        smart::GpsSensor,
    },
    prelude::*,
};

use crate::{
    actuator::{motor_group::MotorGroup, telemetry::Telemetry},
    config::{
        get_distance_1_offset, get_distance_2_offset, get_distance_3_offset, get_gps_offset,
        get_line_1_offset, track_width, wheel_diameter, DRIVE_RATIO,
    },
    localization::localization::StateRepresentation,
    motion_control::ramsete::Ramsete,
    subsystems::{
        drivetrain::{Drivetrain, TankDrive},
        intake::Intake,
    },
};

mod actuator;
mod config;
mod localization;
mod motion_control;
mod sensor;
mod state_machine;
mod subsystems;
mod utils;

struct Robot {
    drivetrain: Drivetrain,
    intake: Intake,
    controller: Controller,
    _telemetry: Telemetry,
    _telemetry_task: Task<()>,
}

impl Robot {
    async fn new(mut peripherals: Peripherals) -> Self {
        let _telemetry = Telemetry::new(SerialPort::open(peripherals.port_10, 115200));

        let drivetrain = Drivetrain::new(
            Arc::new(Mutex::new(MotorGroup::new(vec![
                Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
                Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
                Motor::new(peripherals.port_3, Gearset::Green, Direction::Forward),
            ]))),
            Arc::new(Mutex::new(MotorGroup::new(vec![
                Motor::new(peripherals.port_4, Gearset::Green, Direction::Forward),
                Motor::new(peripherals.port_5, Gearset::Green, Direction::Forward),
                Motor::new(peripherals.port_6, Gearset::Green, Direction::Forward),
            ]))),
            InertialSensor::new(peripherals.port_8),
            wheel_diameter(),
            DRIVE_RATIO,
            _telemetry.clone(),
            vec![
                (
                    DistanceSensor::new(peripherals.port_12),
                    get_distance_1_offset(),
                ),
                (
                    DistanceSensor::new(peripherals.port_13),
                    get_distance_2_offset(),
                ),
                (
                    DistanceSensor::new(peripherals.port_14),
                    get_distance_3_offset(),
                ),
            ],
            vec![(AdiLineTracker::new(peripherals.adi_a), get_line_1_offset())],
            GpsSensor::new(peripherals.port_11, get_gps_offset(), ((0.0, 0.0), 0.0))
                .expect("GPS not plugged in"),
        )
        .await;

        Self {
            drivetrain,
            intake: Intake::new(
                Motor::new(peripherals.port_15, Gearset::Red, Direction::Forward),
                Motor::new(peripherals.port_16, Gearset::Red, Direction::Forward),
                Motor::new(peripherals.port_17, Gearset::Red, Direction::Forward),
            ),
            controller: peripherals.primary_controller,
            _telemetry: _telemetry.clone(),
            _telemetry_task: spawn(async move {
                let mut text = Text::new_aligned(
                    "test-text",
                    TextSize::Small,
                    (0, 0),
                    Default::default(),
                    Default::default(),
                );
                loop {
                    text.text = CString::new(format!("Current time: {}", 0.0)).unwrap();
                    peripherals.screen.fill(&text, Rgb::WHITE);

                    sleep(Duration::from_millis(50)).await;
                }
            }),
        }
    }
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("auto");

        self.drivetrain
            .init_norm(
                &StateRepresentation::new(0.0, 0.0, 0.0),
                &Matrix3::new(0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0),
            )
            .await;

        let ramsete = Ramsete::try_new(
            0.1,
            0.5,
            Box::new(
                CombinedMP::try_new_2d(
                    serde_json::from_str(include_str!("../bins/paths/test.json")).unwrap(),
                    track_width().get::<meter>(),
                )
                .unwrap(),
            ),
        )
        .unwrap();

        self.drivetrain.run_velocity(ramsete).await;

        {
            let drive_state = self.drivetrain.run(VoltageDrive::new(-12.0, -12.0));

            let _ = select_biased! {
                () = drive_state.fuse() => 1,
                () = sleep(Duration::from_secs(2)).fuse() => 2,
            };
        }

        self.drivetrain.run(VoltageDrive::new(0.0, 0.0)).await;
    }

    async fn driver(&mut self) {
        println!("Drive");
        let drive_state = self.drivetrain.run(TankDrive::new(&self.controller));

        join!(drive_state).await;
    }
}

#[vexide::main(banner = true)]
async fn main(peripherals: Peripherals) {
    let robot = Robot::new(peripherals).await;
    robot.compete().await;
}
