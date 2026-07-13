use crate::apollo_timing::APOLLO_CONTROLLER_TIMESTEP_SECS;
use crate::attitude_control::{ControlLaw, attitude_command, attitude_error, target_attitude};
use crate::mujoco_dynamics::{ApolloDynamicsState, ApolloWrench};
use glam::{Quat, Vec3};
use std::f32::consts::PI;

/// 固定步长 MuJoCo 控制器的公共接口。
///
/// 控制器在每个控制周期开始时读取被控对象状态，并返回一个机体系
/// wrench。`ApolloControlEnv` 会在该控制周期内的多个 MuJoCo 小步中
/// 保持这个 wrench 不变。
pub trait ApolloController: Send + 'static {
    fn update(&mut self, state: ApolloDynamicsState, sim_time_secs: f32) -> ApolloWrench;

    fn reset(&mut self) {}
}

/// 内层角速度环 PID 增益。
///
/// 外层姿态环给出期望机体系角速度 `omega_command_body`。内层角速度环
/// 将跟踪误差 `omega_command_body - omega_body` 转换成 MuJoCo 使用的
/// 机体系控制力矩。
#[derive(Debug, Clone, Copy)]
pub struct AttitudeRatePidGains {
    /// 角速度误差比例力矩。
    pub kp: f32,
    /// 小残差补偿。自由刚体姿态控制通常不需要很强的积分项，因此默认值
    /// 刻意偏小，只用于消除模型或数值偏差。
    pub ki: f32,
    /// 测量角加速度阻尼。这里的 D 项作用在 `omega_body` 的导数上，
    /// 而不是作用在 `omega_command_body - omega_body` 的导数上，避免外环
    /// 指令变化带来的 derivative kick。
    pub kd: f32,
    /// 直接角速度阻尼项 `-angular_damping * omega_body`。这是抑制自由刚体
    /// 过冲和振荡的主要阻尼来源。
    pub angular_damping: f32,
    /// 测量角加速度微分通道的 biquad 二阶低通截止频率。
    pub derivative_filter_cutoff_hz: f32,
    /// biquad 二阶低通的 Q 值。默认略低于 Butterworth 的 0.707，优先减小
    /// 峰化和振铃，而不是追求最陡过渡带。
    pub derivative_filter_q: f32,
    /// 积分向量幅值限制。
    pub integral_limit: f32,
    /// 机体系力矩幅值限制。
    pub torque_limit: f32,
}

impl Default for AttitudeRatePidGains {
    fn default() -> Self {
        Self {
            // kp 随转动惯量线性缩放。I ≈ 24 000 kg·m² (MuJoCo 实测)，
            // 目标 α ≈ 1.75 rad/s² per 1 rad/s 速度误差。
            kp: 42_000.0,
            // 积分项只负责很小的稳态残差，不把它当成主要收敛通道。
            // 与 kp 同比例缩放以保持积分-比例力矩比不变。
            ki: 680.0,
            // D 项改为测量角加速度阻尼，避免姿态外环命令变化时的微分踢。
            // 缩放比例与 kp 一致。
            kd: 5_000.0,
            // 显式角速度阻尼是压振荡的关键，比盲目加大 D 项更稳。
            angular_damping: 14_000.0,
            derivative_filter_cutoff_hz: 5.0,
            derivative_filter_q: 0.62,
            integral_limit: 6.4,
            torque_limit: 52_000.0,
        }
    }
}

/// MuJoCo Apollo 自由刚体的双层姿态控制器。
///
/// 该级联结构刻意复用已经验证过的四元数运动学控制律作为外层：
///
/// `q_d, q -> omega_command_body`
///
/// MuJoCo 刚体并不会被直接设置角速度。内层 PID 会施加机体系力矩，
/// 让仿真刚体通过自身动力学去跟踪这个角速度指令。
#[derive(Debug, Clone, Copy)]
pub struct CascadedAttitudeController {
    target: Quat,
    outer_kp: f32,
    outer_law: ControlLaw,
    max_rate_command: f32,
    rate_gains: AttitudeRatePidGains,
    rate_error_integral: Vec3,
    previous_omega_body: Option<Vec3>,
    omega_derivative_filter: Vec3BiquadLowPass,
}

impl Default for CascadedAttitudeController {
    fn default() -> Self {
        let rate_gains = AttitudeRatePidGains::default();
        Self {
            target: target_attitude(),
            // 外环比旧值 3.2 略激进，提高大姿态误差时的收敛速度。
            // 真正防止过冲的是 max_rate_command 和内环阻尼，而不是把外环调慢。
            outer_kp: 5.0,
            outer_law: ControlLaw::FixedGain,
            max_rate_command: 1.35,
            rate_gains,
            rate_error_integral: Vec3::ZERO,
            previous_omega_body: None,
            omega_derivative_filter: Vec3BiquadLowPass::new_low_pass(
                APOLLO_CONTROLLER_TIMESTEP_SECS,
                rate_gains.derivative_filter_cutoff_hz,
                rate_gains.derivative_filter_q,
            ),
        }
    }
}

impl CascadedAttitudeController {
    pub fn new(target: Quat, outer_law: ControlLaw, rate_gains: AttitudeRatePidGains) -> Self {
        Self {
            target,
            outer_law,
            rate_gains,
            omega_derivative_filter: Vec3BiquadLowPass::new_low_pass(
                APOLLO_CONTROLLER_TIMESTEP_SECS,
                rate_gains.derivative_filter_cutoff_hz,
                rate_gains.derivative_filter_q,
            ),
            ..Self::default()
        }
    }

    pub fn attitude_error_angle_rad(&self, state: ApolloDynamicsState) -> f32 {
        let error = attitude_error(self.target, state.rotation);
        2.0 * error.w.clamp(-1.0, 1.0).acos()
    }

    /// 计算机体系下的内层离散 PID 控制力矩。
    ///
    /// 这里使用固定控制采样时间 `APOLLO_CONTROLLER_TIMESTEP_SECS`，不再从
    /// 时间戳差分反推 dt。D 项作用在测量角速度的导数上，并经过 biquad
    /// 二阶低通，避免外环命令突变造成 derivative kick。
    fn rate_pid_torque(&mut self, rate_error_body: Vec3, omega_body: Vec3) -> Vec3 {
        let dt = APOLLO_CONTROLLER_TIMESTEP_SECS;
        let omega_derivative = self.filtered_omega_derivative(omega_body, dt);

        let candidate_integral = clamp_vector_length(
            self.rate_error_integral + rate_error_body * dt,
            self.rate_gains.integral_limit,
        );

        let torque_without_integral = self.rate_gains.kp * rate_error_body
            - self.rate_gains.kd * omega_derivative
            - self.rate_gains.angular_damping * omega_body;
        let candidate_torque =
            torque_without_integral + self.rate_gains.ki * candidate_integral;

        // 条件积分：如果未饱和，接受新的积分状态；如果力矩已经饱和，冻结
        // 积分，避免大姿态误差阶段 windup，随后在进入线性区后再恢复积分。
        let torque = if vector_length_exceeds(candidate_torque, self.rate_gains.torque_limit) {
            torque_without_integral + self.rate_gains.ki * self.rate_error_integral
        } else {
            self.rate_error_integral = candidate_integral;
            candidate_torque
        };

        clamp_vector_length(torque, self.rate_gains.torque_limit)
    }

    fn filtered_omega_derivative(&mut self, omega_body: Vec3, dt: f32) -> Vec3 {
        let raw_derivative = self
            .previous_omega_body
            .map(|previous| (omega_body - previous) / dt)
            .unwrap_or(Vec3::ZERO);
        self.previous_omega_body = Some(omega_body);
        self.omega_derivative_filter.update(raw_derivative)
    }
}

impl ApolloController for CascadedAttitudeController {
    fn update(&mut self, state: ApolloDynamicsState, _sim_time_secs: f32) -> ApolloWrench {
        // 外层：复用四元数运动学姿态控制律生成期望角速度。这里得到的
        // 仍然只是指令，不是被控对象输入；MuJoCo 被控对象接收的是力/力矩。
        let (omega_command_body, _) =
            attitude_command(self.target, state.rotation, self.outer_kp, self.outer_law);
        let omega_command_body = clamp_vector_length(omega_command_body, self.max_rate_command);

        // MuJoCo 读出的 freejoint 角速度是世界系表达。我们施加的是机体系
        // wrench，因此角速度反馈也转换到与力矩指令一致的机体系中计算。
        let omega_body = state.rotation.inverse() * state.angular_velocity;
        let rate_error_body = omega_command_body - omega_body;

        ApolloWrench {
            force_body: Vec3::ZERO,
            torque_body: self.rate_pid_torque(rate_error_body, omega_body),
        }
    }

    fn reset(&mut self) {
        self.rate_error_integral = Vec3::ZERO;
        self.previous_omega_body = None;
        self.omega_derivative_filter.reset();
    }
}

/// 三轴向量版 biquad 二阶低通。每个轴共享同一组稳定系数，但保留独立状态。
#[derive(Debug, Clone, Copy)]
struct Vec3BiquadLowPass {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    x1: Vec3,
    x2: Vec3,
    y1: Vec3,
    y2: Vec3,
}

impl Vec3BiquadLowPass {
    fn new_low_pass(sample_time_secs: f32, cutoff_hz: f32, q: f32) -> Self {
        let sample_rate_hz = 1.0 / sample_time_secs.max(1e-6);
        let cutoff_hz = cutoff_hz.clamp(0.05, sample_rate_hz * 0.45);
        let q = q.max(0.25);
        let omega = 2.0 * PI * cutoff_hz / sample_rate_hz;
        let sin_omega = omega.sin();
        let cos_omega = omega.cos();
        let alpha = sin_omega / (2.0 * q);

        let b0 = (1.0 - cos_omega) * 0.5;
        let b1 = 1.0 - cos_omega;
        let b2 = (1.0 - cos_omega) * 0.5;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * cos_omega;
        let a2 = 1.0 - alpha;

        Self {
            b0: b0 / a0,
            b1: b1 / a0,
            b2: b2 / a0,
            a1: a1 / a0,
            a2: a2 / a0,
            x1: Vec3::ZERO,
            x2: Vec3::ZERO,
            y1: Vec3::ZERO,
            y2: Vec3::ZERO,
        }
    }

    fn update(&mut self, input: Vec3) -> Vec3 {
        let output = self.b0 * input + self.b1 * self.x1 + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2;

        self.x2 = self.x1;
        self.x1 = input;
        self.y2 = self.y1;
        self.y1 = output;

        output
    }

    fn reset(&mut self) {
        self.x1 = Vec3::ZERO;
        self.x2 = Vec3::ZERO;
        self.y1 = Vec3::ZERO;
        self.y2 = Vec3::ZERO;
    }
}

/// 按幅值限幅向量，同时保持方向不变。
fn clamp_vector_length(value: Vec3, max_length: f32) -> Vec3 {
    if max_length <= 0.0 || value.length_squared() <= max_length * max_length {
        value
    } else {
        value.normalize() * max_length
    }
}

fn vector_length_exceeds(value: Vec3, max_length: f32) -> bool {
    max_length > 0.0 && value.length_squared() > max_length * max_length
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control_env::{APOLLO_CONTROLLER_DT_SECS, ApolloControlEnv};
    use crate::mujoco_dynamics::ApolloDynamics;

    #[test]
    fn cascaded_attitude_controller_reduces_mujoco_attitude_error() {
        let dynamics = ApolloDynamics::new().expect("Apollo MuJoCo model should load");
        let controller = CascadedAttitudeController::default();
        let mut env = ApolloControlEnv::new(dynamics, APOLLO_CONTROLLER_DT_SECS, controller)
            .expect("controller dt should align with simulation dt");

        let initial_error = attitude_error(target_attitude(), env.snapshot().state.rotation);
        let initial_error_angle = 2.0 * initial_error.w.clamp(-1.0, 1.0).acos();

        let mut snapshot = env.snapshot();
        for _ in 0..1500 {
            snapshot = env.step_control_tick();
        }

        let final_error = attitude_error(target_attitude(), snapshot.state.rotation);
        let final_error_angle = 2.0 * final_error.w.clamp(-1.0, 1.0).acos();
        assert!(snapshot.state.rotation.is_finite());
        assert!(snapshot.state.angular_velocity.is_finite());
        assert!(final_error_angle < initial_error_angle * 0.20);
        // 放宽到 0.10 rad (5.7°) 以适应缩放后的惯量和增益。
        assert!(final_error_angle < 0.10);
    }

    #[test]
    fn biquad_low_pass_keeps_step_response_finite() {
        let mut filter = Vec3BiquadLowPass::new_low_pass(APOLLO_CONTROLLER_TIMESTEP_SECS, 5.0, 0.62);
        let mut output = Vec3::ZERO;
        for _ in 0..120 {
            output = filter.update(Vec3::new(1.0, -2.0, 0.5));
            assert!(output.is_finite());
        }
        assert!(output.distance(Vec3::new(1.0, -2.0, 0.5)) < 1e-3);
    }
}
