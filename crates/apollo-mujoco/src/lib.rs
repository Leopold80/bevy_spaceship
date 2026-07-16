//! MuJoCo 实现的 Apollo 同步被控对象。
//!
//! 公共边界只有显式状态、机体系 wrench、确定性 tick 和 `reset/step`；
//! 控制律、制导律、奖励、运行循环与可视化均由调用方组合。

mod error;
mod mjcf;
mod plant;

pub use apollo_core::{
    ApolloModelSpec, ApolloState, BodyWrench, Plant, PlantSnapshot, PlantStep, SimulationTiming,
};
pub use error::PlantError;
pub use mjcf::generate_apollo_mjcf;
pub use plant::{ApolloPlant, ApolloPlantFactory};
