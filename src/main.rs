#![no_main]
#![no_std]
#![feature(future_join)]

extern crate alloc;

use alloc::boxed::Box;
use core::{future::join, time::Duration};

use futures::{select_biased, FutureExt};
use state_machine::Subsystem;
use subsystems::{
    drivetrain::VoltageDrive,
    lift::{Lift, TeleopArm},
};
use vexide::prelude::*;

mod state_machine;
mod subsystems;

use crate::subsystems::drivetrain::{Drivetrain, TankDrive};

struct Robot {
    drivetrain: Drivetrain,
    lift: Lift,
    controller: Controller,
}

impl Robot {
    fn new(peripherals: Peripherals) -> Self {
        Self {
            drivetrain: Drivetrain::new(
                Motor::new(peripherals.port_12, Gearset::Green, Direction::Forward),
                Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
            ),
            lift: Lift::new(Motor::new(
                peripherals.port_2,
                Gearset::Green,
                Direction::Forward,
            )),
            controller: peripherals.primary_controller,
        }
    }
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        {
            let drive_state = self.drivetrain.run(VoltageDrive::new(12.0, 12.0));

            let _ = select_biased! {
                () = drive_state.fuse() => 1,
                () = sleep(Duration::from_secs(2)).fuse() => 2,
            };
        }

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
        let drive_state = self.drivetrain.run(TankDrive::new(&self.controller));
        let arm_state = self.lift.run(TeleopArm::new(&self.controller));

        join!(arm_state, drive_state).await;
    }
}

#[vexide::main(banner = true)]
async fn main(peripherals: Peripherals) {
    let robot = Robot::new(peripherals);
    robot.compete().await;
}
