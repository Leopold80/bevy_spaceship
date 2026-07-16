//! 例程：调用方在普通 Rust 循环中组合姿态控制器、plant 与轨迹记录器。
//!
//! 控制器和闭环都只存在于本例程；`apollo-mujoco` 库不持有这些概念。

use apollo_core::{JsonlTrajectoryWriter, TelemetryFrame, TrajectoryHeader};
use apollo_mujoco::{ApolloPlantFactory, ApolloState, BodyWrench};
use glam::{DQuat, DVec3, EulerRot};
use std::error::Error;
use std::f64::consts::PI;
use std::fs::{File, create_dir_all};
use std::io::{BufWriter, Write};

const OUTPUT_PATH: &str = "runs/closed_loop_attitude.jsonl";
const CONTROL_TICKS: usize = 1_500;

fn main() -> Result<(), Box<dyn Error>> {
    let factory = ApolloPlantFactory::apollo_touchdown()?;
    let initial_state = challenge_initial_state();
    let mut plant = factory.spawn(initial_state)?;
    let timing = plant.timing();
    let target = target_attitude();
    let mut controller = CascadedAttitudeController::new(target, timing.control_step_seconds());
    let initial_snapshot = plant.snapshot();

    create_dir_all("runs")?;
    let output = BufWriter::new(File::create(OUTPUT_PATH)?);
    let mut writer =
        JsonlTrajectoryWriter::new(output, TrajectoryHeader::apollo(timing, initial_snapshot))?;

    let initial_error = attitude_error_angle(target, initial_state.body_to_world);
    let mut snapshot = initial_snapshot;
    let mut two_second_error = None;
    let mut two_second_rate = None;

    // 这就是完整闭环：读取显式状态，调用用户算法，再显式推进 plant 一步。
    for tick in 0..CONTROL_TICKS {
        let action = controller.update(snapshot.state);
        let step = plant.step(action)?;
        writer.write_frame(&TelemetryFrame::from(step))?;
        snapshot = step.snapshot;

        if tick == 99 {
            two_second_error = Some(attitude_error_angle(target, snapshot.state.body_to_world));
            two_second_rate = Some(snapshot.state.angular_velocity_body_radps.length());
        }
    }
    writer.get_mut().flush()?;

    let final_error = attitude_error_angle(target, snapshot.state.body_to_world);
    let final_rate = snapshot.state.angular_velocity_body_radps.length();
    println!("trajectory={OUTPUT_PATH}");
    println!("control_ticks={}", snapshot.control_tick);
    println!(
        "initial_attitude_error_deg={:.6}",
        initial_error.to_degrees()
    );
    println!("final_attitude_error_deg={:.6}", final_error.to_degrees());
    println!("final_body_rate_radps={final_rate:.9}");

    let two_second_error = two_second_error.expect("2 s checkpoint must exist");
    let two_second_rate = two_second_rate.expect("2 s checkpoint must exist");
    if two_second_error >= 0.10
        || two_second_rate >= 0.50
        || final_error >= 0.05
        || final_error >= initial_error * 0.20
        || final_rate >= 0.05
    {
        return Err(format!(
            "closed-loop acceptance failed: error@2s={two_second_error}, rate@2s={two_second_rate}, final_error={final_error}, final_rate={final_rate}"
        )
        .into());
    }
    Ok(())
}

fn challenge_initial_state() -> ApolloState {
    ApolloState {
        body_to_world: DQuat::from_euler(EulerRot::XYZ, -0.85, 0.55, 1.25),
        angular_velocity_body_radps: DVec3::new(0.55, -0.35, 0.25),
        ..ApolloState::ZERO
    }
}

fn target_attitude() -> DQuat {
    DQuat::from_axis_angle(DVec3::new(0.42, 0.83, -0.37).normalize(), 1.15)
}

fn attitude_error(target: DQuat, current: DQuat) -> DQuat {
    let error = (target.inverse() * current).normalize();
    if error.w < 0.0 { -error } else { error }
}

fn attitude_error_angle(target: DQuat, current: DQuat) -> f64 {
    2.0 * attitude_error(target, current).w.clamp(-1.0, 1.0).acos()
}

#[derive(Clone, Copy)]
struct RatePidGains {
    kp: f64,
    ki: f64,
    kd: f64,
    angular_damping: f64,
    derivative_filter_cutoff_hz: f64,
    derivative_filter_q: f64,
    integral_limit: f64,
    anti_windup_tracking_time_seconds: f64,
    torque_limit_nm: f64,
}

impl Default for RatePidGains {
    fn default() -> Self {
        Self {
            kp: 42_000.0,
            ki: 680.0,
            kd: 5_000.0,
            angular_damping: 14_000.0,
            derivative_filter_cutoff_hz: 5.0,
            derivative_filter_q: 0.62,
            integral_limit: 6.4,
            anti_windup_tracking_time_seconds: 0.25,
            torque_limit_nm: 52_000.0,
        }
    }
}

/// 旧级联控制基线的例程私有迁移版本，不是库 API。
struct CascadedAttitudeController {
    target: DQuat,
    outer_kp: f64,
    maximum_rate_command_radps: f64,
    sample_time_seconds: f64,
    gains: RatePidGains,
    rate_error_integral: DVec3,
    previous_body_rate: Option<DVec3>,
    derivative_filter: Vec3BiquadLowPass,
}

impl CascadedAttitudeController {
    fn new(target: DQuat, sample_time_seconds: f64) -> Self {
        let gains = RatePidGains::default();
        Self {
            target,
            outer_kp: 5.0,
            maximum_rate_command_radps: 1.35,
            sample_time_seconds,
            gains,
            rate_error_integral: DVec3::ZERO,
            previous_body_rate: None,
            derivative_filter: Vec3BiquadLowPass::new(
                sample_time_seconds,
                gains.derivative_filter_cutoff_hz,
                gains.derivative_filter_q,
            ),
        }
    }

    fn update(&mut self, state: ApolloState) -> BodyWrench {
        let error = attitude_error(self.target, state.body_to_world);
        let error_vector = DVec3::new(error.x, error.y, error.z);
        let rate_command = clamp_length(
            -self.outer_kp * error_vector,
            self.maximum_rate_command_radps,
        );
        let body_rate = state.angular_velocity_body_radps;
        let rate_error = rate_command - body_rate;

        BodyWrench {
            force_body_n: DVec3::ZERO,
            torque_about_com_body_nm: self.rate_pid_torque(rate_error, body_rate),
        }
    }

    fn rate_pid_torque(&mut self, rate_error: DVec3, body_rate: DVec3) -> DVec3 {
        let dt = self.sample_time_seconds;
        let raw_derivative = self
            .previous_body_rate
            .map(|previous| (body_rate - previous) / dt)
            .unwrap_or(DVec3::ZERO);
        self.previous_body_rate = Some(body_rate);
        let filtered_derivative = self.derivative_filter.update(raw_derivative);

        let candidate_integral = clamp_length(
            self.rate_error_integral + rate_error * dt,
            self.gains.integral_limit,
        );
        let torque_without_integral = self.gains.kp * rate_error
            - self.gains.kd * filtered_derivative
            - self.gains.angular_damping * body_rate;
        let unconstrained = torque_without_integral + self.gains.ki * candidate_integral;
        let constrained = clamp_length(unconstrained, self.gains.torque_limit_nm);

        let mut corrected_integral = candidate_integral;
        if unconstrained.length_squared() > self.gains.torque_limit_nm * self.gains.torque_limit_nm
            && self.gains.ki.abs() > f64::EPSILON
        {
            let tracking_time = self.gains.anti_windup_tracking_time_seconds.max(dt);
            corrected_integral +=
                (dt / tracking_time) * (constrained - unconstrained) / self.gains.ki;
        }
        self.rate_error_integral = clamp_length(corrected_integral, self.gains.integral_limit);

        clamp_length(
            torque_without_integral + self.gains.ki * self.rate_error_integral,
            self.gains.torque_limit_nm,
        )
    }
}

struct Vec3BiquadLowPass {
    b0: f64,
    b1: f64,
    b2: f64,
    a1: f64,
    a2: f64,
    x1: DVec3,
    x2: DVec3,
    y1: DVec3,
    y2: DVec3,
}

impl Vec3BiquadLowPass {
    fn new(sample_time_seconds: f64, cutoff_hz: f64, q: f64) -> Self {
        let sample_rate_hz = 1.0 / sample_time_seconds.max(1.0e-9);
        let cutoff_hz = cutoff_hz.clamp(0.05, sample_rate_hz * 0.45);
        let q = q.max(0.25);
        let omega = 2.0 * PI * cutoff_hz / sample_rate_hz;
        let alpha = omega.sin() / (2.0 * q);
        let cosine = omega.cos();
        let a0 = 1.0 + alpha;

        Self {
            b0: ((1.0 - cosine) * 0.5) / a0,
            b1: (1.0 - cosine) / a0,
            b2: ((1.0 - cosine) * 0.5) / a0,
            a1: (-2.0 * cosine) / a0,
            a2: (1.0 - alpha) / a0,
            x1: DVec3::ZERO,
            x2: DVec3::ZERO,
            y1: DVec3::ZERO,
            y2: DVec3::ZERO,
        }
    }

    fn update(&mut self, input: DVec3) -> DVec3 {
        let output = self.b0 * input + self.b1 * self.x1 + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2;
        self.x2 = self.x1;
        self.x1 = input;
        self.y2 = self.y1;
        self.y1 = output;
        output
    }
}

fn clamp_length(value: DVec3, maximum: f64) -> DVec3 {
    if value.length_squared() <= maximum * maximum {
        value
    } else {
        value.normalize() * maximum
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn private_controller_opposes_body_rate_at_its_target() {
        let target = DQuat::from_euler(EulerRot::XYZ, 0.2, -0.4, 0.7);
        let mut controller = CascadedAttitudeController::new(target, 0.02);
        let action = controller.update(ApolloState {
            body_to_world: target,
            angular_velocity_body_radps: DVec3::X,
            ..ApolloState::ZERO
        });

        assert!(action.torque_about_com_body_nm.x < 0.0);
        assert!(action.torque_about_com_body_nm.y.abs() < 1.0e-8);
        assert!(action.torque_about_com_body_nm.z.abs() < 1.0e-8);
    }
}
