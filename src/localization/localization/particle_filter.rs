use alloc::{boxed::Box, sync::Arc, vec::Vec};
use core::time::Duration;

use rand::{distributions::Uniform, rngs::SmallRng, Rng, SeedableRng};
use uom::si::{f64::Length, length::meter};
use vexide::core::{println, sync::Mutex, time::Instant};

use super::{Localization, Sensor, StateRepresentation};
use crate::{
    actuator::motor_group::MotorGroup, localization::predict::tank_pose_tracking::TankPoseTracking,
};

pub struct ParticleFilter<const D: usize> {
    particles: [StateRepresentation; D],
    sensors: Vec<Box<dyn Sensor>>,
    predictor: TankPoseTracking<Arc<Mutex<MotorGroup>>>,
    rng: SmallRng,
    last_update_time: Instant,
    dist_since_update: f64,
    min_update_interval: Duration,
    min_update_distance: Length,
}

impl<const D: usize> ParticleFilter<D> {
    pub fn new(
        predictor: TankPoseTracking<Arc<Mutex<MotorGroup>>>,
        min_update_interval: Duration,
        min_update_distance: Length,
    ) -> Self {
        Self {
            particles: [StateRepresentation::new(0.0, 0.0, 0.0); D],
            sensors: Vec::new(),
            predictor,
            rng: SmallRng::seed_from_u64(0),
            last_update_time: Instant::now(),
            dist_since_update: 0.0,
            min_update_interval,
            min_update_distance,
        }
    }
}

impl<const D: usize> Localization for ParticleFilter<D> {
    fn pose_estimate(&self) -> StateRepresentation {
        self.particles.iter().sum::<StateRepresentation>() / D as f64
    }

    async fn update(&mut self) {
        println!("particle filter");

        self.predictor.update().await;

        let orientation = self.predictor.orientation().angle();

        // Predict step
        for mut particle in self.particles {
            particle += self.predictor.predict();
            particle.z = orientation;
        }

        self.dist_since_update += self.predictor.predict().magnitude();

        if self.dist_since_update < self.min_update_distance.get::<meter>()
            && self.min_update_interval > Instant::now() - self.last_update_time
            || self.sensors.is_empty()
        {
            return;
        }

        // Update step
        let mut weights = [0.0; D];

        // weight particles
        for (i, particle) in self.particles.iter().enumerate() {
            weights[i] = self
                .sensors
                .iter()
                .filter_map(|sensor| sensor.p(&particle))
                .sum();
        }

        // Do resample
        let avg_weight = weights.iter().sum::<f64>() / weights.len() as f64;
        let sample_rand = self.rng.sample(Uniform::new(0.0, &avg_weight));

        let old_particles = self.particles.clone();

        let mut sum = sample_rand;

        for i in 0..weights.len() {
            let mut weight_sum = weights[0];
            let mut particle_index = 0;

            while weight_sum < sum && particle_index < weights.len() {
                particle_index += 1;
                weight_sum += weights[particle_index];
            }

            self.particles[i] = old_particles[particle_index];

            sum += sample_rand;
        }

        self.last_update_time = Instant::now();
    }
}
