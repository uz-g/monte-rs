use core::time::Duration;

use uom::si::{angle::revolution, f64::Angle};
use vexide::prelude::{sleep, Motor, Position};

use crate::state_machine::State;

pub struct Hook {
    motor: Motor,
}

impl Hook {
    pub fn new(motor: Motor) -> Self {
        Self { motor }
    }

    pub async fn run(&mut self, mut state: impl State<(), f64>) {
        state.init();

        loop {
            if let Some(position) = state.update(&()) {
                let _ = self
                    .motor
                    .set_position_target(Position::from_revolutions(position), 200);
            } else {
                return;
            }

            sleep(Duration::from_millis(10)).await;
        }
    }
}

pub struct HookPosition(pub Angle);

impl State<(), f64> for HookPosition {
    fn update(&mut self, _: &()) -> Option<f64> {
        Some(self.0.get::<revolution>())
    }
}
