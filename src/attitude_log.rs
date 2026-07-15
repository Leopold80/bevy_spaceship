use crate::attitude_control::{
    ATTITUDE_KP, AttitudeSample, attitude_command, current_scenario, integrate_attitude,
    target_attitude,
};
use bevy::prelude::*;
use std::fs::{File, create_dir_all};
use std::io::Write;

pub const LOG_INTERVAL_SECS: f32 = 0.1;

#[derive(Resource)]
pub struct AttitudeLog {
    path: String,
    file: File,
}

impl AttitudeLog {
    pub fn new(path: &str) -> Self {
        create_dir_all("logs").expect("failed to create logs directory");
        let mut file = File::create(path).expect("failed to create attitude log");
        writeln!(
            file,
            "time_s,qe0,qev_norm,error_angle_rad,omega_norm,omega_x,omega_y,omega_z"
        )
        .expect("failed to write attitude log header");

        Self {
            path: path.to_string(),
            file,
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new(&self.path);
    }

    pub fn write_sample(&mut self, sample: AttitudeSample) {
        writeln!(
            self.file,
            "{:.4},{:.8},{:.8},{:.8},{:.8},{:.8},{:.8},{:.8}",
            sample.time_s,
            sample.qe0,
            sample.qev_norm,
            sample.error_angle_rad,
            sample.omega.length(),
            sample.omega.x,
            sample.omega.y,
            sample.omega.z
        )
        .expect("failed to write attitude log sample");
    }
}

pub fn run_headless_attitude_log(path: &str, duration_secs: f32, dt: f32) {
    let target = target_attitude();
    let mut current = current_scenario().initial;
    let mut log = AttitudeLog::new(path);
    let mut elapsed_secs = 0.0;
    let mut next_log_secs = 0.0;

    while elapsed_secs <= duration_secs {
        let (omega, mut sample) = attitude_command(target, current, ATTITUDE_KP);

        if elapsed_secs >= next_log_secs {
            sample.time_s = elapsed_secs;
            log.write_sample(sample);
            next_log_secs += LOG_INTERVAL_SECS;
        }

        current = integrate_attitude(current, omega, dt);
        elapsed_secs += dt;
    }
}
