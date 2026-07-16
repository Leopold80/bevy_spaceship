//! Apollo 被控对象的后端中立公共边界。
//!
//! 本 crate 只定义状态、动作、时序、轨迹与模型规格；它不包含
//! 具体仿真后端、可视化或上层算法。

#![forbid(unsafe_code)]

mod model;
mod plant;
mod state;
mod timing;
mod trajectory;

pub use model::{
    APOLLO_BODY_NAME, APOLLO_FREEJOINT_NAME, APOLLO_IXX_KG_M2, APOLLO_IYY_KG_M2, APOLLO_IZZ_KG_M2,
    APOLLO_TOUCHDOWN_MASS_KG, ApolloCollisionPart, ApolloMassPoint, ApolloMaterial,
    ApolloModelSpec, ApolloShape, ApolloVisualPart, apollo_collision_parts, apollo_mass_points,
    apollo_visual_parts, center_of_mass_body_m, total_physics_mass_kg,
};
pub use plant::{Plant, PlantSnapshot, PlantStep};
pub use state::{
    ApolloState, BodyWrench, UNIT_QUATERNION_NORM_TOLERANCE, ValidationError,
    normalized_quaternion, validate_finite_quaternion, validate_finite_vec3,
    validate_unit_quaternion,
};
pub use timing::{APOLLO_PHYSICS_STEP_NS, APOLLO_SUBSTEPS_PER_CONTROL, SimulationTiming};
pub use trajectory::{
    APOLLO_TELEMETRY_MODEL, AttitudeReference, JsonlTrajectoryWriter, TELEMETRY_FORMAT,
    TELEMETRY_FORMAT_VERSION, TelemetryFrame, TelemetryFrameError, TrajectoryHeader,
    TrajectoryHeaderError, TrajectoryWriteError,
};
