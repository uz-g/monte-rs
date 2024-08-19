use alloc::{string::String, vec::Vec};

use nalgebra::Vector2;
use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize)]
struct Point {
    x: f64,
    y: f64,
}

impl Into<Vector2<f64>> for Point {
    fn into(self) -> Vector2<f64> {
        Vector2::new(self.x, self.y)
    }
}

#[derive(Debug, Serialize, Deserialize)]
struct PathSegment {
    inverted: bool,
    stop_end: bool,
    path: Vec<Point>,
}

#[derive(Debug, Serialize, Deserialize)]
struct Command {
    t: f64,
    name: String,
}

#[derive(Debug, Serialize, Deserialize)]
struct Path {
    start_speed: f64,
    end_speed: f64,
    segments: Vec<PathSegment>,
    commands: Vec<Command>,
}
