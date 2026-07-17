//! `apollo-propulsion-demo` 的纯输入状态机。
//!
//! 该模块不持有 plant，也不绕过执行器限制；它只把按键意图整理成下一控制
//! tick 的 16 路门控时间与 DPS 工作挡位，随后仍交给 `ApolloPropulsionPlant` 校验。

use crate::torque_couples::AxisTorqueSet;
pub use apollo_core::{RCS_MINIMUM_PULSE_NS, RCS_THRUSTER_COUNT};

pub const DPS_THRUST_STEP_LBF: f64 = 525.0;
pub const LBF_TO_NEWTON: f64 = 4.448_221_615_260_5;
pub const DPS_GIMBAL_STEP_RAD: f64 = 0.5_f64.to_radians();

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DemoMode {
    SingleRcs,
    TorqueCouple,
    Dps,
}

impl DemoMode {
    pub fn next(self) -> Self {
        match self {
            Self::SingleRcs => Self::TorqueCouple,
            Self::TorqueCouple => Self::Dps,
            Self::Dps => Self::SingleRcs,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DpsControlLimits {
    pub variable_min_thrust_n: f64,
    pub variable_max_thrust_n: f64,
    pub full_thrust_n: f64,
    pub maximum_gimbal_rad: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DemoDpsRequest {
    Off,
    Variable {
        thrust_n: f64,
        gimbal_x_rad: f64,
        gimbal_z_rad: f64,
    },
    FullThrust {
        gimbal_x_rad: f64,
        gimbal_z_rad: f64,
    },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DemoPropulsionRequest {
    pub rcs_on_time_ns: [u64; RCS_THRUSTER_COUNT],
    pub dps: DemoDpsRequest,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PropulsionDemoControls {
    pub mode: DemoMode,
    pub selected_single_thruster: usize,
    pub selected_torque_axis: usize,
    /// 0..=10 为 1050..=6300 lbf 的 525 lbf 步进，11 为 9870 lbf FTP。
    pub dps_thrust_level: u8,
    pub dps_enabled: bool,
    pub gimbal_x_rad: f64,
    pub gimbal_z_rad: f64,
}

impl Default for PropulsionDemoControls {
    fn default() -> Self {
        Self {
            mode: DemoMode::SingleRcs,
            selected_single_thruster: 0,
            selected_torque_axis: 0,
            dps_thrust_level: 0,
            dps_enabled: false,
            gimbal_x_rad: 0.0,
            gimbal_z_rad: 0.0,
        }
    }
}

impl PropulsionDemoControls {
    pub const VARIABLE_LEVEL_COUNT: u8 = 11;
    pub const FULL_THRUST_LEVEL: u8 = Self::VARIABLE_LEVEL_COUNT;

    pub fn cycle_mode(&mut self) {
        self.mode = self.mode.next();
    }

    pub fn select_next(&mut self) {
        match self.mode {
            DemoMode::SingleRcs => {
                self.selected_single_thruster =
                    (self.selected_single_thruster + 1) % RCS_THRUSTER_COUNT;
            }
            DemoMode::TorqueCouple => {
                self.selected_torque_axis = (self.selected_torque_axis + 1) % 6;
            }
            DemoMode::Dps => {}
        }
    }

    pub fn select_previous(&mut self) {
        match self.mode {
            DemoMode::SingleRcs => {
                self.selected_single_thruster = self
                    .selected_single_thruster
                    .checked_sub(1)
                    .unwrap_or(RCS_THRUSTER_COUNT - 1);
            }
            DemoMode::TorqueCouple => {
                self.selected_torque_axis = self.selected_torque_axis.checked_sub(1).unwrap_or(5);
            }
            DemoMode::Dps => {}
        }
    }

    pub fn toggle_dps(&mut self) {
        self.dps_enabled = !self.dps_enabled;
    }

    pub fn step_dps_thrust_up(&mut self) {
        self.dps_thrust_level = self
            .dps_thrust_level
            .saturating_add(1)
            .min(Self::FULL_THRUST_LEVEL);
    }

    pub fn step_dps_thrust_down(&mut self) {
        self.dps_thrust_level = self.dps_thrust_level.saturating_sub(1);
    }

    pub fn adjust_gimbal(&mut self, delta_x_rad: f64, delta_z_rad: f64, maximum_rad: f64) {
        let requested_x = self.gimbal_x_rad + delta_x_rad;
        let requested_z = self.gimbal_z_rad + delta_z_rad;
        let magnitude = requested_x.hypot(requested_z);
        let scale = if magnitude > maximum_rad {
            maximum_rad / magnitude
        } else {
            1.0
        };
        self.gimbal_x_rad = requested_x * scale;
        self.gimbal_z_rad = requested_z * scale;
    }

    pub fn all_off(&mut self) {
        self.dps_enabled = false;
    }

    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn selected_rcs_indices(&self, torque_sets: &[AxisTorqueSet; 6]) -> Vec<usize> {
        match self.mode {
            DemoMode::SingleRcs => vec![self.selected_single_thruster],
            DemoMode::TorqueCouple => torque_sets[self.selected_torque_axis]
                .thruster_indices
                .clone(),
            DemoMode::Dps => Vec::new(),
        }
    }

    pub fn build_request(
        &self,
        torque_sets: &[AxisTorqueSet; 6],
        pulse_requested: bool,
        continuous_requested: bool,
        control_step_ns: u64,
        limits: DpsControlLimits,
    ) -> DemoPropulsionRequest {
        let mut rcs_on_time_ns = [0; RCS_THRUSTER_COUNT];
        let on_time_ns = if continuous_requested {
            control_step_ns
        } else if pulse_requested {
            RCS_MINIMUM_PULSE_NS
        } else {
            0
        };
        if on_time_ns > 0 {
            for index in self.selected_rcs_indices(torque_sets) {
                if let Some(slot) = rcs_on_time_ns.get_mut(index) {
                    *slot = on_time_ns;
                }
            }
        }

        let dps = if !self.dps_enabled {
            DemoDpsRequest::Off
        } else if self.dps_thrust_level == Self::FULL_THRUST_LEVEL {
            DemoDpsRequest::FullThrust {
                gimbal_x_rad: self.gimbal_x_rad,
                gimbal_z_rad: self.gimbal_z_rad,
            }
        } else {
            let step_n = DPS_THRUST_STEP_LBF * LBF_TO_NEWTON;
            let requested =
                limits.variable_min_thrust_n + f64::from(self.dps_thrust_level) * step_n;
            DemoDpsRequest::Variable {
                thrust_n: requested.min(limits.variable_max_thrust_n),
                gimbal_x_rad: self.gimbal_x_rad,
                gimbal_z_rad: self.gimbal_z_rad,
            }
        };
        DemoPropulsionRequest {
            rcs_on_time_ns,
            dps,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::DVec3;

    fn torque_sets() -> [AxisTorqueSet; 6] {
        std::array::from_fn(|index| AxisTorqueSet {
            target_axis_body: [
                DVec3::X,
                DVec3::NEG_X,
                DVec3::Y,
                DVec3::NEG_Y,
                DVec3::Z,
                DVec3::NEG_Z,
            ][index],
            thruster_indices: vec![index, index + 6],
            torque_about_com_body_nm: DVec3::ZERO,
        })
    }

    fn limits() -> DpsControlLimits {
        DpsControlLimits {
            variable_min_thrust_n: 1_050.0 * LBF_TO_NEWTON,
            variable_max_thrust_n: 6_300.0 * LBF_TO_NEWTON,
            full_thrust_n: 9_870.0 * LBF_TO_NEWTON,
            maximum_gimbal_rad: 6.0_f64.to_radians(),
        }
    }

    #[test]
    fn dps_steps_cover_variable_range_then_enter_separate_full_thrust_detent() {
        let mut controls = PropulsionDemoControls {
            dps_enabled: true,
            ..Default::default()
        };
        for level in 0..=PropulsionDemoControls::FULL_THRUST_LEVEL {
            controls.dps_thrust_level = level;
            let request =
                controls.build_request(&torque_sets(), false, false, 20_000_000, limits());
            if level < PropulsionDemoControls::FULL_THRUST_LEVEL {
                let DemoDpsRequest::Variable { thrust_n, .. } = request.dps else {
                    panic!("variable level must not become full thrust");
                };
                let expected = (1_050.0 + f64::from(level) * 525.0) * LBF_TO_NEWTON;
                assert!((thrust_n - expected).abs() < 1.0e-9);
            } else {
                assert!(matches!(request.dps, DemoDpsRequest::FullThrust { .. }));
            }
        }
    }

    #[test]
    fn fourteen_millisecond_pulse_and_full_tick_continuous_are_distinct() {
        let controls = PropulsionDemoControls::default();
        let pulse = controls.build_request(&torque_sets(), true, false, 20_000_000, limits());
        let continuous = controls.build_request(&torque_sets(), false, true, 20_000_000, limits());
        assert_eq!(pulse.rcs_on_time_ns[0], RCS_MINIMUM_PULSE_NS);
        assert_eq!(continuous.rcs_on_time_ns[0], 20_000_000);
        assert_eq!(
            pulse
                .rcs_on_time_ns
                .iter()
                .filter(|value| **value > 0)
                .count(),
            1
        );
    }

    #[test]
    fn gimbal_step_clamps_to_a_circular_six_degree_cone() {
        let mut controls = PropulsionDemoControls::default();
        for _ in 0..20 {
            controls.adjust_gimbal(
                DPS_GIMBAL_STEP_RAD,
                DPS_GIMBAL_STEP_RAD,
                limits().maximum_gimbal_rad,
            );
        }
        assert!(
            (controls.gimbal_x_rad.hypot(controls.gimbal_z_rad) - limits().maximum_gimbal_rad)
                .abs()
                < 1.0e-12
        );
        assert!(controls.gimbal_x_rad > 0.0 && controls.gimbal_z_rad > 0.0);
    }

    #[test]
    fn selection_wraps_in_single_and_torque_modes() {
        let mut controls = PropulsionDemoControls::default();
        controls.select_previous();
        assert_eq!(controls.selected_single_thruster, 15);
        controls.cycle_mode();
        controls.select_previous();
        assert_eq!(controls.selected_torque_axis, 5);
        controls.cycle_mode();
        controls.select_next();
        assert_eq!(controls.mode, DemoMode::Dps);
    }
}
