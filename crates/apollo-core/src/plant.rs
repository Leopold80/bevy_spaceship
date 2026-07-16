use crate::{ApolloState, BodyWrench, SimulationTiming};
use serde::{Deserialize, Serialize};

/// 被控对象在某一确定 tick 上的快照。
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PlantSnapshot {
    /// 当前物理状态。
    pub state: ApolloState,
    /// 自最近一次 reset 以来完成的外部控制步数。
    pub control_tick: u64,
    /// 自最近一次 reset 以来完成的物理子步数。
    pub physics_tick: u64,
}

impl PlantSnapshot {
    /// 从显式初始状态构造零 tick 快照。
    pub const fn initial(state: ApolloState) -> Self {
        Self {
            state,
            control_tick: 0,
            physics_tick: 0,
        }
    }

    /// 由 `physics_tick` 和给定时序派生仿真时间，纳秒。
    pub const fn sim_time_ns(self, timing: SimulationTiming) -> u128 {
        timing.sim_time_ns(self.physics_tick)
    }

    /// 由 `physics_tick` 和给定时序派生仿真时间，秒。
    pub fn sim_time_seconds(self, timing: SimulationTiming) -> f64 {
        timing.sim_time_seconds(self.physics_tick)
    }
}

/// `Plant::step` 的完整结果。
///
/// `requested_action` 与 `applied_action` 分开表达，为以后的执行器限幅或分配器
/// 保留可观测性；当前理想 wrench plant 中两者相同。
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PlantStep {
    /// 动作执行后的快照。
    pub snapshot: PlantSnapshot,
    /// 调用方请求的动作。
    pub requested_action: BodyWrench,
    /// 后端实际应用的动作。
    pub applied_action: BodyWrench,
}

/// 同步、外部动作驱动的 Apollo 被控对象契约。
///
/// 实现不应在这些方法中 sleep、创建窗口或执行隐藏的上层策略。
pub trait Plant {
    /// 后端构建、重置或推进错误。
    type Error;

    /// 返回该实例的固定时序。
    fn timing(&self) -> SimulationTiming;

    /// 重置到调用方给定的显式状态，并将 tick 清零。
    fn reset(&mut self, initial_state: ApolloState) -> Result<PlantSnapshot, Self::Error>;

    /// 返回当前快照，不推进仿真。
    fn snapshot(&self) -> PlantSnapshot;

    /// 提交一个动作，并严格推进一个控制周期。
    fn step(&mut self, action: BodyWrench) -> Result<PlantStep, Self::Error>;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn snapshot_time_is_derived_from_physics_tick() {
        let snapshot = PlantSnapshot {
            state: ApolloState::ZERO,
            control_tick: 3,
            physics_tick: 30,
        };
        assert_eq!(snapshot.sim_time_ns(SimulationTiming::APOLLO), 60_000_000);
        assert!((snapshot.sim_time_seconds(SimulationTiming::APOLLO) - 0.06).abs() < f64::EPSILON);
    }
}
