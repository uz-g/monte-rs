use alloc::{boxed::Box, sync::Arc, vec::Vec};
use core::time::Duration;

use nalgebra::{clamp, Matrix3};
use rand::{
    distributions::{Distribution, Uniform},
    rngs::SmallRng,
    Rng, SeedableRng,
};
use rand_distr::Normal;
use uom::si::{f64::Length, length::meter};
use vexide::core::{println, sync::Mutex, time::Instant};

use super::{Localization, Sensor, StateRepresentation};
use crate::{
    actuator::motor_group::MotorGroup, config::FIELD_MAX,
    localization::predict::tank_pose_tracking::TankPoseTracking, uom::num_traits::Float,
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
        let rng = SmallRng::seed_from_u64(0);

        Self {
            particles: [StateRepresentation::new(0.0, 0.0, 0.0); D],
            sensors: Vec::new(),
            predictor,
            rng,
            last_update_time: Instant::now(),
            dist_since_update: 0.0,
            min_update_interval,
            min_update_distance,
        }
    }

    pub fn add_sensor(&mut self, sensor: impl Sensor + 'static) {
        self.sensors.push(Box::new(sensor));
    }

    pub fn get_estimates(&self) -> [StateRepresentation; D] {
        self.particles.clone()
    }

    pub fn init_norm(&mut self, mean: &StateRepresentation, covariance: &Matrix3<f64>) {
        let normal_dist = Normal::new(0.0, 1.0).expect("Can't create normal dist");

        for particle in self.particles.iter_mut() {
            let new_particle = mean
                + covariance
                    * StateRepresentation::new(
                        normal_dist.sample(&mut self.rng),
                        normal_dist.sample(&mut self.rng),
                        normal_dist.sample(&mut self.rng),
                    );
            particle.x = clamp(new_particle.x, -FIELD_MAX, FIELD_MAX);
            particle.y = clamp(new_particle.y, -FIELD_MAX, FIELD_MAX);
            particle.z = new_particle.z;
        }
    }

    #[allow(dead_code)]
    pub fn init_uniform(&mut self, min: &StateRepresentation, max: &StateRepresentation) {
        assert!(min.x <= max.x, "Min must be less than max");
        assert!(min.y <= max.y, "Min must be less than max");
        assert!(min.z <= max.z, "Min must be less than max");

        let normal_dist_x = Uniform::new(min.x, max.x);
        let normal_dist_y = Uniform::new(min.y, max.y);
        let normal_dist_z = Uniform::new(min.z, max.z);

        for particle in self.particles.iter_mut() {
            particle.x = normal_dist_x.sample(&mut self.rng);
            particle.y = normal_dist_y.sample(&mut self.rng);
            particle.z = normal_dist_z.sample(&mut self.rng);
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
        for particle in self.particles.iter_mut() {
            let prediction = self.predictor.predict();
            particle.x += prediction.x;
            particle.y += prediction.y;
            particle.z = orientation;
        }

        self.dist_since_update += self.predictor.predict().magnitude();

        // Only run if it's been longer than the minimum time or moved more than 2 inches
        if (Instant::now() - self.last_update_time) < self.min_update_interval
            && self.dist_since_update < self.min_update_distance.get::<meter>()
            || self.sensors.is_empty()
        {
            return;
        }

        // println!("Update");

        // Update step
        let mut weights = [0.0; D];

        // weight particles
        for (i, particle) in self.particles.iter().enumerate() {
            weights[i] = self
                .sensors
                .iter()
                .filter_map(|sensor| sensor.p(&particle))
                .sum::<f64>()
                .abs();
        }

        // Calculate average weight and random variable for resample
        let avg_weight = weights.iter().sum::<f64>() / weights.len() as f64;
        if avg_weight <= 0.0 {
            println!("WARNING: Avg weight too low: {}", avg_weight);
            return;
        }
        let sample_rand = self.rng.sample(Uniform::new(0.0, avg_weight));

        // Clone the particles to be memory safe with resample
        let old_particles = self.particles.clone();

        let mut sum = sample_rand;

        // Sample an adequate number of points
        for i in 0..D {
            // Start with a sum and index for the while loop
            let mut weight_sum = weights[0];
            let mut particle_index = 0;

            // Increase until it gets to the right index
            while weight_sum < sum && particle_index < weights.len() {
                particle_index += 1;
                weight_sum += weights[particle_index];
            }

            // Resample the particle
            self.particles[i] = old_particles[particle_index];

            // Increase the sum
            sum += avg_weight;
        }

        self.last_update_time = Instant::now();
        self.dist_since_update = 0.0;
    }
}
