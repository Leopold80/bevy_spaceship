use apollo_core::ValidationError;
use thiserror::Error;

/// MuJoCo Apollo 被控对象的构造、输入和推进错误。
#[derive(Clone, Debug, Error, PartialEq)]
pub enum PlantError {
    #[error("invalid Apollo model specification: {0}")]
    InvalidModelSpec(String),

    #[error("failed to compile Apollo MJCF: {0}")]
    ModelLoad(String),

    #[error("MuJoCo body '{body_name}' was not found")]
    BodyNotFound { body_name: &'static str },

    #[error("unexpected MuJoCo state layout: expected nq=7 and nv=6, got nq={nq} and nv={nv}")]
    UnexpectedModelLayout { nq: usize, nv: usize },

    #[error("failed to allocate independent MuJoCo data: {0}")]
    DataAllocation(String),

    #[error("invalid initial state: {0}")]
    InvalidInitialState(ValidationError),

    #[error("invalid body wrench: {0}")]
    InvalidAction(ValidationError),

    #[error("MuJoCo produced an invalid state: {0}")]
    InvalidSimulationState(ValidationError),

    #[error("simulation tick counter overflow")]
    TickOverflow,
}
