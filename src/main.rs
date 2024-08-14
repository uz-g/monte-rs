#![no_main]
#![no_std]
#![feature(future_join)]
#![feature(async_closure)]
#![feature(duration_millis_float)]
extern crate alloc;

use alloc::{ffi::CString, sync::Arc};
use core::{future::join, time::Duration};

use futures::{select_biased, FutureExt};
use state_machine::Subsystem;
use subsystems::{
    drivetrain::VoltageDrive,
    lift::{Lift, TeleopArm},
};
use vexide::prelude::*;

mod actuator;
mod config;
mod localization;
mod sensor;
mod state_machine;
mod subsystems;

extern crate uom;

use alloc::{format, vec};
use core::ops::Add;

use nalgebra::{Matrix, Matrix2, Matrix3, Vector2};
use vexide::{
    core::{io::stdout, sync::Mutex, time::Instant},
    devices::{
        screen::{Text, TextSize},
        smart::SmartDeviceType,
    },
};

use crate::{
    actuator::{motor_group::MotorGroup, telemetry::Telemetry},
    config::{wheel_diameter, DRIVE_RATIO},
    localization::{localization::StateRepresentation, sensor::DummySensor},
    subsystems::drivetrain::{Drivetrain, TankDrive},
};

struct Robot {
    drivetrain: Drivetrain,
    lift: Lift,
    controller: Controller,
    telemetry: Telemetry,
    _telemetry_task: Task<()>,
}

impl Robot {
    async fn new(mut peripherals: Peripherals) -> Self {
        let telemetry = Telemetry::new(SerialPort::open(peripherals.port_2, 115200));

        let drivetrain = Drivetrain::new(
            Arc::new(Mutex::new(MotorGroup::new(vec![Motor::new(
                peripherals.port_12,
                Gearset::Green,
                Direction::Forward,
            )]))),
            Arc::new(Mutex::new(MotorGroup::new(vec![Motor::new(
                peripherals.port_1,
                Gearset::Green,
                Direction::Forward,
            )]))),
            InertialSensor::new(peripherals.port_5),
            wheel_diameter(),
            DRIVE_RATIO,
            telemetry.clone(),
        )
        .await;

        Self {
            drivetrain,
            lift: Lift::new(Motor::new(
                peripherals.port_3,
                Gearset::Green,
                Direction::Forward,
            )),
            controller: peripherals.primary_controller,
            telemetry: telemetry.clone(),
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
        sleep(Duration::from_secs(2)).await;

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
        let arm_state = self.lift.run(TeleopArm::new(&self.controller));

        join!(arm_state, drive_state).await;
    }
}

#[vexide::main(banner = false)]
async fn main(peripherals: Peripherals) {
    let robot = Robot::new(peripherals).await;
    robot.compete().await;
}
