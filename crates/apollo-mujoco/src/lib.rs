//! MuJoCo 实现的 Apollo 同步被控对象。
//!
//! 公共边界只有显式状态、机体系 wrench、确定性 tick 和 `reset/step`；
//! 控制律、制导律、奖励、运行循环与可视化均由调用方组合。

mod error;
mod mjcf;
mod plant;
mod propulsion;
mod rcs_actuator;

pub use apollo_core::{
    ApolloModelSpec, ApolloPropulsionSpec, ApolloState, AppliedDps, AppliedPropulsion,
    AppliedRcsThruster, BodyWrench, DPS_GIMBAL_RATE_RAD_S, DpsCommand, DpsMode, DpsSpec, Plant,
    PlantSnapshot, PlantStep, PropulsionCommand, PropulsionStep, RCS_THRUSTER_COUNT,
    RCS_THRUSTER_LABELS, RcsCommand, RcsFeedSystem, RcsQuad, RcsThrusterId, RcsThrusterSpec,
    SimulationTiming,
};
pub use error::PlantError;
pub use mjcf::generate_apollo_mjcf;
pub use plant::{ApolloPlant, ApolloPlantFactory};
pub use propulsion::{ApolloPropulsionPlant, ApolloPropulsionPlantFactory};
