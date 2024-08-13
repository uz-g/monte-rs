use nalgebra::Vector3;
use serde::{Serialize, Serializer};

use super::sensor::*;

pub type StateRepresentation = Vector3<f64>;

pub trait Localization {
    fn pose_estimate(&self) -> StateRepresentation;
    async fn update(&mut self);
}

pub mod particle_filter;
