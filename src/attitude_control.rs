use glam::{EulerRot, Quat, Vec3};

pub mod legacy;

pub const ATTITUDE_KP: f32 = 2.4;

#[derive(Clone, Copy)]
pub struct AttitudeScenario {
    pub name: &'static str,
    pub initial: Quat,
}

pub struct AttitudeSample {
    pub time_s: f32,
    pub qe0: f32,
    pub qev_norm: f32,
    pub error_angle_rad: f32,
    pub omega: Vec3,
}

pub fn target_attitude() -> Quat {
    let (axis, angle) = desired_axis_angle();
    Quat::from_axis_angle(axis, angle)
}

pub fn desired_axis_angle() -> (Vec3, f32) {
    (Vec3::new(0.42, 0.83, -0.37).normalize(), 1.15)
}

pub fn current_scenario() -> AttitudeScenario {
    scenario_at(0)
}

pub fn scenario_at(index: usize) -> AttitudeScenario {
    attitude_scenarios()[index]
}

fn attitude_scenarios() -> [AttitudeScenario; 3] {
    [
        AttitudeScenario {
            name: "1 Mixed roll-pitch-yaw",
            initial: Quat::from_euler(EulerRot::XYZ, -0.95, 0.72, 1.35).normalize(),
        },
        AttitudeScenario {
            name: "2 Compound error (175.3 deg)",
            initial: Quat::from_axis_angle(Vec3::new(0.7, -0.25, 0.66).normalize(), 2.55)
                * Quat::from_euler(EulerRot::XYZ, 0.45, -0.38, 0.2),
        },
        AttitudeScenario {
            name: "3 Compound error (149.5 deg)",
            initial: Quat::from_axis_angle(Vec3::new(-0.35, 0.9, 0.27).normalize(), 3.05)
                * Quat::from_rotation_z(-0.55),
        },
    ]
}

pub fn attitude_error(target: Quat, current: Quat) -> Quat {
    let error = (target.inverse() * current).normalize();
    if error.w < 0.0 {
        Quat::from_xyzw(-error.x, -error.y, -error.z, -error.w)
    } else {
        error
    }
}

pub fn attitude_command(target: Quat, current: Quat, kp: f32) -> (Vec3, AttitudeSample) {
    let error = attitude_error(target, current);
    let qev = Vec3::new(error.x, error.y, error.z);
    let omega = -kp * qev;
    (omega, attitude_sample(error, omega))
}

fn attitude_sample(error: Quat, omega: Vec3) -> AttitudeSample {
    AttitudeSample {
        time_s: 0.0,
        qe0: error.w,
        qev_norm: Vec3::new(error.x, error.y, error.z).length(),
        error_angle_rad: 2.0 * error.w.clamp(-1.0, 1.0).acos(),
        omega,
    }
}

pub fn integrate_attitude(current: Quat, omega: Vec3, dt: f32) -> Quat {
    (current * Quat::from_scaled_axis(omega * dt)).normalize()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixed_gain_controller_reduces_all_scenario_errors() {
        let target = target_attitude();
        for scenario in attitude_scenarios() {
            let mut current = scenario.initial;
            let (_, initial_sample) = attitude_command(target, current, ATTITUDE_KP);

            for _ in 0..600 {
                let (omega, _) = attitude_command(target, current, ATTITUDE_KP);
                current = integrate_attitude(current, omega, 1.0 / 60.0);
            }

            let (_, final_sample) = attitude_command(target, current, ATTITUDE_KP);
            assert!(initial_sample.qe0 >= 0.0, "{}", scenario.name);
            assert!(final_sample.qe0 >= 0.0, "{}", scenario.name);
            assert!(
                final_sample.error_angle_rad < initial_sample.error_angle_rad,
                "{}",
                scenario.name
            );
            assert!(final_sample.qev_norm < 0.001, "{}", scenario.name);
        }
    }

    #[test]
    fn scenario_names_match_target_relative_error_angles() {
        let target = target_attitude();
        let scenario_two_error = attitude_command(target, scenario_at(1).initial, ATTITUDE_KP)
            .1
            .error_angle_rad
            .to_degrees();
        let scenario_three_error = attitude_command(target, scenario_at(2).initial, ATTITUDE_KP)
            .1
            .error_angle_rad
            .to_degrees();

        assert!((scenario_two_error - 175.3).abs() < 0.1);
        assert!((scenario_three_error - 149.5).abs() < 0.1);
    }
}
