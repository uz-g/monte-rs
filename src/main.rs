#![no_main]
#![no_std]
#![feature(future_join)]
#![feature(async_closure)]

extern crate alloc;

use alloc::sync::Arc;
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

use alloc::vec;
use core::ops::Add;

use vexide::core::sync::Mutex;

use crate::{
    actuator::{motor_group::MotorGroup, telemetry::Telemetry},
    config::{wheel_diameter, DRIVE_RATIO},
    subsystems::drivetrain::{Drivetrain, TankDrive},
};

struct Robot {
    drivetrain: Drivetrain,
    lift: Lift,
    controller: Controller,
    telemetry: Telemetry,
}

impl Robot {
    fn new(peripherals: Peripherals) -> Self {
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
        );
        Self {
            drivetrain,
            lift: Lift::new(Motor::new(
                peripherals.port_3,
                Gearset::Green,
                Direction::Forward,
            )),
            controller: peripherals.primary_controller,
            telemetry: Telemetry::new(SerialPort::open(peripherals.port_2, 115200)),
        }
    }
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
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
        // let drive_state = self.drivetrain.run(TankDrive::new(&self.controller));
        // let arm_state = self.lift.run(TeleopArm::new(&self.controller));
        loop {
            self.telemetry.send("hello world\n".as_bytes()).await;
            println!("hello world");
            sleep(Duration::from_millis(10)).await;
        }

        // join!(arm_state, drive_state, fn_test()).await;
    }
}

#[vexide::main(banner = true)]
async fn main(peripherals: Peripherals) {
    let robot = Robot::new(peripherals);
    robot.compete().await;
}
