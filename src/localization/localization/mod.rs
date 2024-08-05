use alloc::vec::Vec;
use core::time::Duration;

use nalgebra::Vector3;
use rand::{distributions::Uniform, prelude::Distribution, rngs::SmallRng, Rng, SeedableRng};
use vexide::prelude::sleep;

pub type StateRepresentation = Vector3<f64>;

use alloc::boxed::Box;

use super::sensor::*;

pub trait Localization {
    fn pose_estimate(&self) -> StateRepresentation;
    fn update(&mut self);
}

pub mod particle_filter;
