use crate::attitude_control::{ControlLaw, attitude_command, attitude_error, target_attitude};
use crate::mujoco_dynamics::{ApolloDynamicsState, ApolloWrench};
use glam::{Quat, Vec3};

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
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub integral_limit: f32,
    pub torque_limit: f32,
}

impl Default for AttitudeRatePidGains {
    fn default() -> Self {
        Self {
            kp: 1000.0,
            ki: 2.0,
            kd: 120.0,
            integral_limit: 0.45,
            torque_limit: 1200.0,
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
    rate_gains: AttitudeRatePidGains,
    rate_error_integral: Vec3,
    previous_rate_error: Option<Vec3>,
    previous_time_secs: Option<f32>,
}

impl Default for CascadedAttitudeController {
    fn default() -> Self {
        Self {
            target: target_attitude(),
            outer_kp: 3.2,
            outer_law: ControlLaw::FixedGain,
            rate_gains: AttitudeRatePidGains::default(),
            rate_error_integral: Vec3::ZERO,
            previous_rate_error: None,
            previous_time_secs: None,
        }
    }
}

impl CascadedAttitudeController {
    pub fn new(target: Quat, outer_law: ControlLaw, rate_gains: AttitudeRatePidGains) -> Self {
        Self {
            target,
            outer_law,
            rate_gains,
            ..Self::default()
        }
    }

    pub fn attitude_error_angle_rad(&self, state: ApolloDynamicsState) -> f32 {
        let error = attitude_error(self.target, state.rotation);
        2.0 * error.w.clamp(-1.0, 1.0).acos()
    }

    /// 计算机体系下的内层 PID 控制力矩。
    ///
    /// 积分项会做幅值限制，避免重置瞬态或持续跟踪误差导致 windup。
    /// 最终力矩也会限幅，使控制器更像有限执行器，而不是理想力矩源。
    fn rate_pid_torque(&mut self, rate_error_body: Vec3, dt: f32) -> Vec3 {
        if dt > 0.0 {
            self.rate_error_integral += rate_error_body * dt;
            self.rate_error_integral =
                clamp_vector_length(self.rate_error_integral, self.rate_gains.integral_limit);
        }

        let rate_error_derivative = if dt > 0.0 {
            self.previous_rate_error
                .map(|previous| (rate_error_body - previous) / dt)
                .unwrap_or(Vec3::ZERO)
        } else {
            Vec3::ZERO
        };
        self.previous_rate_error = Some(rate_error_body);

        let torque = self.rate_gains.kp * rate_error_body
            + self.rate_gains.ki * self.rate_error_integral
            + self.rate_gains.kd * rate_error_derivative;

        clamp_vector_length(torque, self.rate_gains.torque_limit)
    }
}

impl ApolloController for CascadedAttitudeController {
    fn update(&mut self, state: ApolloDynamicsState, sim_time_secs: f32) -> ApolloWrench {
        // `ApolloControlEnv` 每个固定控制周期只调用一次控制器。第一个
        // 周期还没有上一帧时间戳，因此该采样点故意不启用积分和微分项。
        let dt = self
            .previous_time_secs
            .map(|previous| sim_time_secs - previous)
            .unwrap_or(0.0);
        self.previous_time_secs = Some(sim_time_secs);

        // 外层：复用四元数运动学姿态控制律生成期望角速度。这里得到的
        // 仍然只是指令，不是被控对象输入；MuJoCo 被控对象接收的是力/力矩。
        let (omega_command_body, _) =
            attitude_command(self.target, state.rotation, self.outer_kp, self.outer_law);

        // MuJoCo 读出的 freejoint 角速度是世界系表达。我们施加的是机体系
        // wrench，因此角速度反馈也转换到与力矩指令一致的机体系中计算。
        let omega_body = state.rotation.inverse() * state.angular_velocity;
        let rate_error_body = omega_command_body - omega_body;

        ApolloWrench {
            force_body: Vec3::ZERO,
            torque_body: self.rate_pid_torque(rate_error_body, dt),
        }
    }

    fn reset(&mut self) {
        self.rate_error_integral = Vec3::ZERO;
        self.previous_rate_error = None;
        self.previous_time_secs = None;
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
        for _ in 0..800 {
            snapshot = env.step_control_tick();
        }

        let final_error = attitude_error(target_attitude(), snapshot.state.rotation);
        let final_error_angle = 2.0 * final_error.w.clamp(-1.0, 1.0).acos();
        assert!(snapshot.state.rotation.is_finite());
        assert!(snapshot.state.angular_velocity.is_finite());
        assert!(final_error_angle < initial_error_angle * 0.25);
        assert!(final_error_angle < 0.08);
    }
}
