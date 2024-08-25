use core::time::Duration;

use vexide::{
    core::println,
    devices::adi::digital::LogicLevel,
    prelude::{sleep, AdiDigitalOut, Controller},
};

use crate::state_machine::State;

pub struct GoalClamp {
    adi_solenoid: AdiDigitalOut,
}

impl GoalClamp {
    pub fn new(adi_solenoid: AdiDigitalOut) -> Self {
        Self { adi_solenoid }
    }

    pub async fn run(&mut self, mut state: impl State<(), LogicLevel>) {
        state.init();

        loop {
            if let Some(command) = state.update(&()) {
                println!("{:?}", command);
                self.adi_solenoid.set_level(command).unwrap();
            } else {
                return;
            }

            sleep(Duration::from_millis(10)).await;
        }
    }
}

pub struct GoalController<'a> {
    pub controller: &'a Controller,
}

impl<'a> State<(), LogicLevel> for GoalController<'a> {
    fn update(&mut self, _: &()) -> Option<LogicLevel> {
        Some(self.controller.button_a.level().unwrap_or(LogicLevel::High))
    }
}
