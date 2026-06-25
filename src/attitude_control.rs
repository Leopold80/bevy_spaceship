use glam::{EulerRot, Quat, Vec3};

pub const ATTITUDE_KP: f32 = 2.4;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ControlLaw {
    ScaledQuaternion,
    FixedGain,
}

impl ControlLaw {
    pub fn name(self) -> &'static str {
        match self {
            Self::ScaledQuaternion => "q_e0 q_ev feedback",
            Self::FixedGain => "fixed-gain q_ev feedback",
        }
    }

    pub fn hud_formula(self) -> &'static str {
        match self {
            Self::ScaledQuaternion => "qe=qd^-1*q, qe0>=0, wc=-kp*qe0*qev",
            Self::FixedGain => "qe=qd^-1*q, qe0>=0, wc=-kp*qev",
        }
    }

    pub fn toggled(self) -> Self {
        match self {
            Self::ScaledQuaternion => Self::FixedGain,
            Self::FixedGain => Self::ScaledQuaternion,
        }
    }
}

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
            name: "2 Large compound error",
            initial: Quat::from_axis_angle(Vec3::new(0.7, -0.25, 0.66).normalize(), 2.55)
                * Quat::from_euler(EulerRot::XYZ, 0.45, -0.38, 0.2),
        },
        AttitudeScenario {
            name: "3 Near 180 deg",
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

pub fn attitude_command(
    target: Quat,
    current: Quat,
    kp: f32,
    control_law: ControlLaw,
) -> (Vec3, AttitudeSample) {
    let error = attitude_error(target, current);
    let qev = Vec3::new(error.x, error.y, error.z);
    let omega = match control_law {
        ControlLaw::ScaledQuaternion => -kp * error.w * qev,
        ControlLaw::FixedGain => -kp * qev,
    };
    let sample = AttitudeSample {
        time_s: 0.0,
        qe0: error.w,
        qev_norm: qev.length(),
        error_angle_rad: 2.0 * error.w.clamp(-1.0, 1.0).acos(),
        omega,
    };

    (omega, sample)
}

pub fn integrate_attitude(current: Quat, omega: Vec3, dt: f32) -> Quat {
    (current * Quat::from_scaled_axis(omega * dt)).normalize()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kinematic_controller_reduces_attitude_error() {
        let target = target_attitude();
        let mut current = current_scenario().initial;
        let (_, initial_sample) =
            attitude_command(target, current, ATTITUDE_KP, ControlLaw::ScaledQuaternion);

        for _ in 0..600 {
            let (omega, _) =
                attitude_command(target, current, ATTITUDE_KP, ControlLaw::ScaledQuaternion);
            current = integrate_attitude(current, omega, 1.0 / 60.0);
        }

        let (_, final_sample) =
            attitude_command(target, current, ATTITUDE_KP, ControlLaw::ScaledQuaternion);

        assert!(initial_sample.qe0 >= 0.0);
        assert!(final_sample.qe0 >= 0.0);
        assert!(final_sample.qev_norm < initial_sample.qev_norm);
        assert!(final_sample.error_angle_rad < initial_sample.error_angle_rad);
        assert!(final_sample.qev_norm < 0.001);
    }

    #[test]
    fn fixed_gain_controller_reduces_attitude_error() {
        let target = target_attitude();
        let mut current = current_scenario().initial;
        let (_, initial_sample) =
            attitude_command(target, current, ATTITUDE_KP, ControlLaw::FixedGain);

        for _ in 0..600 {
            let (omega, _) = attitude_command(target, current, ATTITUDE_KP, ControlLaw::FixedGain);
            current = integrate_attitude(current, omega, 1.0 / 60.0);
        }

        let (_, final_sample) =
            attitude_command(target, current, ATTITUDE_KP, ControlLaw::FixedGain);

        assert!(initial_sample.qe0 >= 0.0);
        assert!(final_sample.qe0 >= 0.0);
        assert!(final_sample.qev_norm < initial_sample.qev_norm);
        assert!(final_sample.error_angle_rad < initial_sample.error_angle_rad);
        assert!(final_sample.qev_norm < 0.001);
    }
}
