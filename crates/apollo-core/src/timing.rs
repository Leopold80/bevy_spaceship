use serde::{Deserialize, Serialize};
use std::num::{NonZeroU32, NonZeroU64};

/// Apollo MuJoCo 基线物理步长：2 ms。
pub const APOLLO_PHYSICS_STEP_NS: u64 = 2_000_000;
/// 每个外部控制步中执行的物理子步数。
pub const APOLLO_SUBSTEPS_PER_CONTROL: NonZeroU32 = NonZeroU32::new(10).unwrap();

/// 确定性仿真的时序配置。
///
/// 整数纳秒和整数 tick 是权威时间源；秒数只在边界处派生。
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct SimulationTiming {
    /// 单个物理子步的整数纳秒数。
    pub physics_step_ns: NonZeroU64,
    /// 一次外部 `step` 包含的物理子步数。
    pub substeps_per_control: NonZeroU32,
}

impl SimulationTiming {
    /// 当前 Apollo 基线时序：2 ms 物理步长，每次动作保持 10 个子步。
    pub const APOLLO: Self = Self {
        physics_step_ns: NonZeroU64::new(APOLLO_PHYSICS_STEP_NS).unwrap(),
        substeps_per_control: APOLLO_SUBSTEPS_PER_CONTROL,
    };

    /// 从已由类型保证非零的整数值构造时序。
    pub const fn new(physics_step_ns: NonZeroU64, substeps_per_control: NonZeroU32) -> Self {
        Self {
            physics_step_ns,
            substeps_per_control,
        }
    }

    /// 从原始整数创建时序；任一值为零时返回 `None`。
    pub const fn from_raw(physics_step_ns: u64, substeps_per_control: u32) -> Option<Self> {
        let Some(physics_step_ns) = NonZeroU64::new(physics_step_ns) else {
            return None;
        };
        let Some(substeps_per_control) = NonZeroU32::new(substeps_per_control) else {
            return None;
        };
        Some(Self::new(physics_step_ns, substeps_per_control))
    }

    /// 派生物理子步秒数。
    pub fn physics_step_seconds(self) -> f64 {
        self.physics_step_ns.get() as f64 * 1.0e-9
    }

    /// 派生一次外部控制步的整数纳秒数。
    pub const fn control_step_ns(self) -> u128 {
        self.physics_step_ns.get() as u128 * self.substeps_per_control.get() as u128
    }

    /// 派生一次外部控制步的秒数。
    pub fn control_step_seconds(self) -> f64 {
        self.control_step_ns() as f64 * 1.0e-9
    }

    /// 由物理 tick 派生仿真时间，纳秒。
    pub const fn sim_time_ns(self, physics_tick: u64) -> u128 {
        self.physics_step_ns.get() as u128 * physics_tick as u128
    }

    /// 由物理 tick 派生仿真时间，秒。
    pub fn sim_time_seconds(self, physics_tick: u64) -> f64 {
        self.sim_time_ns(physics_tick) as f64 * 1.0e-9
    }

    /// 将控制 tick 换算为物理 tick，溢出时返回 `None`。
    pub const fn physics_ticks_for_control_ticks(self, control_ticks: u64) -> Option<u64> {
        control_ticks.checked_mul(self.substeps_per_control.get() as u64)
    }
}

impl Default for SimulationTiming {
    fn default() -> Self {
        Self::APOLLO
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn apollo_timing_derives_control_period_from_integer_values() {
        let timing = SimulationTiming::APOLLO;
        assert_eq!(timing.physics_step_ns.get(), 2_000_000);
        assert_eq!(timing.substeps_per_control.get(), 10);
        assert_eq!(timing.control_step_ns(), 20_000_000);
        assert_eq!(timing.physics_ticks_for_control_ticks(7), Some(70));
        assert!((timing.physics_step_seconds() - 0.002).abs() < f64::EPSILON);
        assert!((timing.control_step_seconds() - 0.020).abs() < f64::EPSILON);
        assert!((timing.sim_time_seconds(70) - 0.140).abs() < f64::EPSILON);
    }

    #[test]
    fn raw_constructor_rejects_zero_values() {
        assert_eq!(SimulationTiming::from_raw(0, 10), None);
        assert_eq!(SimulationTiming::from_raw(2_000_000, 0), None);
    }

    #[test]
    fn tick_conversion_reports_overflow() {
        let timing = SimulationTiming::from_raw(1, 2).unwrap();
        assert_eq!(timing.physics_ticks_for_control_ticks(u64::MAX), None);
    }
}
