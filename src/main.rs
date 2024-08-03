#![no_main]
#![no_std]

extern crate alloc;

use alloc::boxed::Box;
use core::{error::Error, time::Duration};

use state_machine::Subsystem;
use vexide::prelude::*;

mod state_machine;
mod subsystems;

use crate::subsystems::drivetrain::{Drivetrain, TankDrive};

struct Robot {
    drivetrain: Drivetrain,
    controller: Controller,
}

impl Robot {
    fn new(peripherals: Peripherals) -> Result<Self, Box<dyn Error>> {
        Ok(Self {
            drivetrain: Drivetrain::new(
                Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
                Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
            ),
            controller: peripherals.primary_controller,
        })
    }
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        sleep(Duration::from_secs(2)).await;
    }

    async fn driver(&mut self) {
        self.drivetrain
            .run(TankDrive {
                controller: &mut self.controller,
            })
            .await;
    }
}

#[vexide::main(banner = false)]
async fn main(peripherals: Peripherals) -> Result<(), Box<dyn Error>> {
    let robot = Robot::new(peripherals)?;
    robot.compete().await;
    Ok(())
}
