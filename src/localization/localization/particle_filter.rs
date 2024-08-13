use alloc::{boxed::Box, sync::Arc, vec::Vec};
use core::{f64::consts::PI, time::Duration};

use arr_macro::arr;
use nalgebra::{clamp, Matrix3, OMatrix, U3};
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
    localization::predict::tank_pose_tracking::TankPoseTracking, state_machine::State,
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
        let mut rng = SmallRng::seed_from_u64(0);

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

    pub fn init_norm(&mut self, mean: &StateRepresentation, covariance: OMatrix<f64, U3, U3>) {
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

    pub fn init_uniform(&mut self, min: &StateRepresentation, max: &StateRepresentation) {
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
        for mut particle in self.particles {
            particle += self.predictor.predict();
            particle.z = orientation;
        }

        self.dist_since_update += self.predictor.predict().magnitude();

        if self.dist_since_update < self.min_update_distance.get::<meter>()
            && self.min_update_interval < Instant::now() - self.last_update_time
            || self.sensors.is_empty()
        {
            return;
        }

        println!("Update");

        // Update step
        let mut weights = [0.0; D];

        // weight particles
        for (i, particle) in self.particles.iter().enumerate() {
            weights[i] = self
                .sensors
                .iter()
                .filter_map(|sensor| {
                    let value = sensor.p(&particle);
                    if let Some(item) = value {
                        println!("{}", item);
                    } else {
                        println!("NONE");
                    }
                    value
                })
                .sum();
        }

        println!("Weights: {:?}", weights);

        // Do resample
        let avg_weight = weights.iter().sum::<f64>() / weights.len() as f64;
        let sample_rand = self.rng.sample(Uniform::new(0.0, 1.0));

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
