#![no_main]
#![no_std]
#![feature(future_join)]

extern crate alloc;

use alloc::boxed::Box;
use core::{error::Error, future::join, time::Duration};

use state_machine::Subsystem;
use subsystems::lift::{Lift, TeleopArm};
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
    fn new(peripherals: Peripherals) -> Result<Self, Box<dyn Error>> {
        Ok(Self {
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
        })
    }
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        sleep(Duration::from_secs(2)).await;
    }

    async fn driver(&mut self) {
        loop {
            sleep(Duration::from_millis(10)).await;
        }
        // self.drivetrain.run(TankDrive::new(&self.controller)).await;
        // let arm_state = self.lift.run(TeleopArm::new(&self.controller));

        // join!(arm_state).await;
    }
}

#[vexide::main(banner = true)]
async fn main(peripherals: Peripherals) -> Result<(), Box<dyn Error>> {
    let robot = Robot::new(peripherals)?;
    robot.compete().await;
    Ok(())
}
