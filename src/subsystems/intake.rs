use core::time::Duration;

use uom::si::{angular_velocity::revolution_per_minute, f64::AngularVelocity};
use vexide::prelude::{sleep, Controller, Motor, Position};

use crate::{
    config::{INTAKE_RATIO, LIFT_RATIO},
    state_machine::State,
};

pub struct Intake {
    bottom: Motor,
    top: Motor,
    lift: Motor,
}

pub struct IntakeState {
    bottom_ring: bool,
    top_1_ring: bool,
    top_2_ring: bool,
}

pub struct IntakeCommand {
    bottom_speed: AngularVelocity,
    top_position: f64,
    lift_position: (f64, i32),
}

impl Intake {
    pub fn new(bottom: Motor, top: Motor, lift: Motor) -> Self {
        Self { bottom, top, lift }
    }

    pub async fn run(&mut self, mut state: impl State<(), IntakeCommand>) {
        state.init();

        loop {
            if let Some(command) = state.update(&()) {
                self.lift
                    .set_position_target(
                        Position::from_revolutions(command.lift_position.0 * LIFT_RATIO),
                        command.lift_position.1,
                    )
                    .expect("Failed to move arm");
                self.top
                    .set_position_target(
                        Position::from_revolutions(command.lift_position.0 * INTAKE_RATIO),
                        200,
                    )
                    .expect("Failed to move intake");
                self.bottom
                    .set_velocity(command.bottom_speed.get::<revolution_per_minute>() as i32)
                    .expect("Can't set bottom intake speed");
            } else {
                return;
            }

            sleep(Duration::from_millis(10)).await;
        }
    }
}

pub struct LoadGoal {}

impl LoadGoal {
    pub fn new() -> Self {
        Self {}
    }
}

impl State<(), IntakeCommand> for LoadGoal {
    fn update(&mut self, i: &()) -> Option<IntakeCommand> {
        todo!()
    }
}

pub struct IntakeManual<'a> {
    controller: &'a Controller,
}

impl<'a> State<(), IntakeCommand> for IntakeManual<'a> {
    fn update(&mut self, i: &()) -> Option<IntakeCommand> {
        None
    }
}
