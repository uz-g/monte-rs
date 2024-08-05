use alloc::{boxed::Box, sync::Arc, vec::Vec};

use rand::{distributions::Uniform, rngs::SmallRng, Rng, SeedableRng};
use vexide::core::sync::Mutex;

use super::{Localization, Sensor, StateRepresentation};
use crate::{
    actuator::motor_group::MotorGroup, localization::predict::tank_pose_tracking::TankPoseTracking,
};

pub struct ParticleFilter<const D: usize> {
    particles: [StateRepresentation; D],
    sensors: Vec<Box<dyn Sensor>>,
    predictor: TankPoseTracking<Arc<Mutex<MotorGroup>>>,
    rng: SmallRng,
}

impl<const D: usize> ParticleFilter<D> {
    pub fn new(predictor: TankPoseTracking<Arc<Mutex<MotorGroup>>>) -> Self {
        Self {
            particles: [StateRepresentation::new(0.0, 0.0, 0.0); D],
            sensors: Vec::new(),
            predictor,
            rng: SmallRng::seed_from_u64(0),
        }
    }
}

impl<const D: usize> Localization for ParticleFilter<D> {
    fn pose_estimate(&self) -> StateRepresentation {
        self.particles.iter().sum::<StateRepresentation>() / D as f64
    }

    async fn update(&mut self) {
        self.predictor.update().await;

        let orientation = self.predictor.orientation().angle();

        // Predict step
        for mut particle in self.particles {
            particle += self.predictor.predict();
            particle.z = orientation;
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
    }
}
