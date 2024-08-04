#![no_main]
#![no_std]
#![feature(future_join)]

extern crate alloc;

use core::{future::join, time::Duration};

use futures::{select_biased, FutureExt};
use localization::localization::ParticleFilter;
use state_machine::Subsystem;
use subsystems::{
    drivetrain::VoltageDrive,
    lift::{Lift, TeleopArm},
};
use vexide::prelude::*;

mod localization;
mod state_machine;
mod subsystems;

use alloc::sync::Arc;

use vexide::core::sync::Mutex;

use crate::subsystems::drivetrain::{Drivetrain, TankDrive};

struct Robot {
    drivetrain: Drivetrain,
    lift: Lift,
    controller: Controller,
    localization: Arc<Mutex<ParticleFilter<100>>>,
    _localization_task: Task<()>,
}

impl Robot {
    fn new(peripherals: Peripherals) -> Self {
        let localization = Arc::new(Mutex::new(ParticleFilter::new()));
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
            localization: localization.clone(),
            _localization_task: spawn(async move {
                loop {
                    localization.lock().await.update().await;

                    sleep(Duration::from_millis(10));
                }
            }),
        }
    }
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        {
            let drive_state =
                self.drivetrain
                    .run(VoltageDrive::new(12.0, 12.0, self.localization.clone()));

            let _ = select_biased! {
                () = drive_state.fuse() => 1,
                () = sleep(Duration::from_secs(2)).fuse() => 2,
            };
        }

        {
            let drive_state =
                self.drivetrain
                    .run(VoltageDrive::new(-12.0, -12.0, self.localization.clone()));

            let _ = select_biased! {
                () = drive_state.fuse() => 1,
                () = sleep(Duration::from_secs(2)).fuse() => 2,
            };
        }

        self.drivetrain
            .run(VoltageDrive::new(0.0, 0.0, self.localization.clone()))
            .await;
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
