//! Rust 完整例程：调用方在普通循环中组合姿态控制器、位置控制器、plant 与轨迹记录器。
//!
//! 控制器和闭环都只存在于本例程；`apollo-mujoco` 库不持有这些概念。
//! 建议先阅读 `main()` 中标出的 1～4 步，再按需要替换两个私有控制器。

use apollo_core::{JsonlTrajectoryWriter, TelemetryFrame, TrajectoryHeader};
use apollo_mujoco::{ApolloModelSpec, ApolloPlantFactory, ApolloState, BodyWrench};
use glam::{DQuat, DVec3, EulerRot};
use std::error::Error;
use std::f64::consts::PI;
use std::fs::{File, create_dir_all};
use std::io::{BufWriter, Write};

const OUTPUT_PATH: &str = "runs/closed_loop_attitude.jsonl";
const CONTROL_TICKS: usize = 1_500;

fn main() -> Result<(), Box<dyn Error>> {
    // 1. 工厂只负责共享只读模型；每次 spawn 都创建状态独立的 plant。
    let factory = ApolloPlantFactory::apollo_touchdown()?;
    let model_spec = factory.model_spec();
    let initial_state = challenge_initial_state(model_spec.center_of_mass_body_m);
    let mut plant = factory.spawn(initial_state)?;
    let timing = plant.timing();
    let target = target_attitude();
    let mut attitude_controller =
        CascadedAttitudeController::new(target, timing.control_step_seconds());
    let (target_com_position, _) = center_of_mass_state(initial_state, model_spec);
    let position_controller = ComPositionController::new(target_com_position, model_spec);
    let initial_snapshot = plant.snapshot();

    // 2. 记录器由调用方创建，不是 plant.step() 的隐藏副作用。
    create_dir_all("runs")?;
    let output = BufWriter::new(File::create(OUTPUT_PATH)?);
    let mut writer = JsonlTrajectoryWriter::new(
        output,
        TrajectoryHeader::apollo(timing, initial_snapshot).with_initial_desired_attitude(target),
    )?;

    let initial_error = attitude_error_angle(target, initial_state.body_to_world);
    let mut snapshot = initial_snapshot;
    let mut two_second_error = None;
    let mut two_second_rate = None;

    // 3. 这就是完整闭环：读状态 -> 算动作 -> step 一次 -> 显式记录。
    // 要接入自己的控制律、制导律或策略，通常只需要替换 update() 调用。
    for tick in 0..CONTROL_TICKS {
        let attitude_action = attitude_controller.update(snapshot.state);
        let action = BodyWrench {
            force_body_n: position_controller.update(snapshot.state),
            torque_about_com_body_nm: attitude_action.torque_about_com_body_nm,
        };
        let step = plant.step(action)?;
        writer.write_frame(&TelemetryFrame::from(step).with_desired_attitude(target))?;
        snapshot = step.snapshot;

        if tick == 99 {
            two_second_error = Some(attitude_error_angle(target, snapshot.state.body_to_world));
            two_second_rate = Some(snapshot.state.angular_velocity_body_radps.length());
        }
    }
    writer.get_mut().flush()?;

    // 4. 例程自带验收条件，既便于人工运行，也能作为 Cargo example 测试目标。
    let final_error = attitude_error_angle(target, snapshot.state.body_to_world);
    let final_rate = snapshot.state.angular_velocity_body_radps.length();
    let (final_com_position, final_com_velocity) = center_of_mass_state(snapshot.state, model_spec);
    let final_position_error = final_com_position.distance(target_com_position);
    let final_com_speed = final_com_velocity.length();
    println!("trajectory={OUTPUT_PATH}");
    println!("control_ticks={}", snapshot.control_tick);
    println!(
        "initial_attitude_error_deg={:.6}",
        initial_error.to_degrees()
    );
    println!("final_attitude_error_deg={:.6}", final_error.to_degrees());
    println!("final_body_rate_radps={final_rate:.9}");
    println!("final_com_position_error_m={final_position_error:.9}");
    println!("final_com_speed_mps={final_com_speed:.9}");

    let two_second_error = two_second_error.expect("2 s checkpoint must exist");
    let two_second_rate = two_second_rate.expect("2 s checkpoint must exist");
    if two_second_error >= 0.10
        || two_second_rate >= 0.50
        || final_error >= 0.05
        || final_error >= initial_error * 0.20
        || final_rate >= 0.05
        || final_position_error >= 0.05
        || final_com_speed >= 0.02
    {
        return Err(format!(
            "closed-loop acceptance failed: error@2s={two_second_error}, rate@2s={two_second_rate}, final_error={final_error}, final_rate={final_rate}, final_position_error={final_position_error}, final_com_speed={final_com_speed}"
        )
        .into());
    }
    Ok(())
}

fn challenge_initial_state(center_of_mass_body_m: DVec3) -> ApolloState {
    let body_to_world = DQuat::from_euler(EulerRot::XYZ, -0.85, 0.55, 1.25);
    let angular_velocity_body_radps = DVec3::new(0.55, -0.35, 0.25);
    let center_of_mass_offset_world = body_to_world * center_of_mass_body_m;
    let angular_velocity_world = body_to_world * angular_velocity_body_radps;

    ApolloState {
        body_to_world,
        // 非零角速度下，原点速度为零并不代表质心静止。这里显式抵消
        // omega x r，避免给零重力刚体注入持续的质心平动速度。
        linear_velocity_body_origin_world_mps: -angular_velocity_world
            .cross(center_of_mass_offset_world),
        angular_velocity_body_radps,
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

fn center_of_mass_state(state: ApolloState, model_spec: ApolloModelSpec) -> (DVec3, DVec3) {
    let offset_world = state.body_to_world * model_spec.center_of_mass_body_m;
    let angular_velocity_world = state.body_to_world * state.angular_velocity_body_radps;
    let position_world = state.position_body_origin_world_m + offset_world;
    let velocity_world =
        state.linear_velocity_body_origin_world_mps + angular_velocity_world.cross(offset_world);
    (position_world, velocity_world)
}

/// 例程私有的质心定点环；输出机体系合力，不改变 plant 的接口和职责。
struct ComPositionController {
    target_position_world_m: DVec3,
    model_spec: ApolloModelSpec,
    natural_frequency_radps: f64,
    damping_ratio: f64,
    maximum_acceleration_mps2: f64,
}

impl ComPositionController {
    fn new(target_position_world_m: DVec3, model_spec: ApolloModelSpec) -> Self {
        Self {
            target_position_world_m,
            model_spec,
            natural_frequency_radps: 0.8,
            damping_ratio: 1.0,
            maximum_acceleration_mps2: 1.0,
        }
    }

    fn update(&self, state: ApolloState) -> DVec3 {
        let (position_world, velocity_world) = center_of_mass_state(state, self.model_spec);
        let omega = self.natural_frequency_radps;
        let acceleration_world = omega * omega * (self.target_position_world_m - position_world)
            - 2.0 * self.damping_ratio * omega * velocity_world;
        let force_world = self.model_spec.mass_kg
            * clamp_length(acceleration_world, self.maximum_acceleration_mps2);

        // plant 的动作力在机体系表达，而位置环在世界系中更直观。
        state.body_to_world.inverse() * force_world
    }
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

    #[test]
    fn challenge_state_starts_with_stationary_center_of_mass() {
        let spec = ApolloModelSpec::touchdown();
        let state = challenge_initial_state(spec.center_of_mass_body_m);
        let (_, velocity) = center_of_mass_state(state, spec);

        assert!(velocity.length() < 1.0e-12);
    }

    #[test]
    fn position_controller_pushes_center_of_mass_toward_target() {
        let spec = ApolloModelSpec::touchdown();
        let controller = ComPositionController::new(DVec3::ZERO, spec);
        let state = ApolloState {
            position_body_origin_world_m: DVec3::X,
            ..ApolloState::ZERO
        };

        let force_body = controller.update(state);
        let (position_world, _) = center_of_mass_state(state, spec);
        assert!(force_body.dot(position_world) < 0.0);
        assert!(force_body.cross(position_world).length() < 1.0e-9);
    }
}
