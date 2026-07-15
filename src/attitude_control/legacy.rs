//! 旧版缩放四元数控制律，仅作理论和回归对照。
//!
//! 该控制律在精确 180° 姿态误差处因 `q_e0 = 0` 而输出零指令，
//! 因此不再接入可视化演示或 MuJoCo 级联控制器。

use super::{AttitudeSample, attitude_error, attitude_sample};
use glam::{Quat, Vec3};

/// 历史参考实现：`omega_c = -kp * q_e0 * q_ev`。
pub fn scaled_quaternion_command(target: Quat, current: Quat, kp: f32) -> (Vec3, AttitudeSample) {
    let error = attitude_error(target, current);
    let qev = Vec3::new(error.x, error.y, error.z);
    let omega = -kp * error.w * qev;
    (omega, attitude_sample(error, omega))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_half_turn_remains_a_documented_legacy_equilibrium() {
        let target = Quat::IDENTITY;
        let current = Quat::from_rotation_x(std::f32::consts::PI);
        let (omega, sample) = scaled_quaternion_command(target, current, 2.4);

        assert!(sample.error_angle_rad.to_degrees() > 179.99);
        assert!(omega.length() < 1e-6);
    }
}
