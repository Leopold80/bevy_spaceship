//! `apollo_sim` 的 PyO3 原生薄绑定。
//!
//! Python 领域对象和 NumPy 校验位于纯 Python 层；此处只进行固定顺序的
//! `Vec<f64>` 转换、Rust 输入复验和 MuJoCo 错误映射。

use apollo_core::{
    ApolloModelSpec, ApolloState, BodyWrench, PlantSnapshot, PlantStep, SimulationTiming,
};
use apollo_mujoco::{
    ApolloPlant as RustApolloPlant, ApolloPlantFactory as RustApolloPlantFactory, PlantError,
};
use glam::{DQuat, DVec3};
use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;

type NativeSnapshot = (Vec<f64>, u64, u64);
type NativeStep = (NativeSnapshot, Vec<f64>, Vec<f64>);
type NativeModelSpec = (String, f64, Vec<f64>, Vec<f64>);

#[pyclass(name = "ApolloPlantFactory", unsendable)]
struct PyApolloPlantFactory {
    inner: RustApolloPlantFactory,
}

#[pymethods]
impl PyApolloPlantFactory {
    #[new]
    fn new(physics_step_seconds: f64, substeps_per_control: u32) -> PyResult<Self> {
        let timing = timing_from_python(physics_step_seconds, substeps_per_control)?;
        let inner = RustApolloPlantFactory::new(ApolloModelSpec::touchdown(), timing)
            .map_err(map_plant_error)?;
        Ok(Self { inner })
    }

    fn spawn(&self, initial_state: Vec<f64>) -> PyResult<PyApolloPlant> {
        let state = state_from_vector(&initial_state)?;
        let inner = self.inner.spawn(state).map_err(map_plant_error)?;
        Ok(PyApolloPlant { inner })
    }

    /// 返回位置控制、执行器分配等调用方算法需要的只读模型参数。
    fn model_spec(&self) -> NativeModelSpec {
        let spec = self.inner.model_spec();
        (
            spec.name.to_owned(),
            spec.mass_kg,
            spec.center_of_mass_body_m.to_array().to_vec(),
            spec.diagonal_inertia_body_kg_m2.to_array().to_vec(),
        )
    }
}

#[pyclass(name = "ApolloPlant", unsendable)]
struct PyApolloPlant {
    inner: RustApolloPlant,
}

#[pymethods]
impl PyApolloPlant {
    fn reset(&mut self, state: Vec<f64>) -> PyResult<NativeSnapshot> {
        let state = state_from_vector(&state)?;
        self.inner
            .reset(state)
            .map(snapshot_to_native)
            .map_err(map_plant_error)
    }

    fn snapshot(&self) -> NativeSnapshot {
        snapshot_to_native(self.inner.snapshot())
    }

    fn step(&mut self, action: Vec<f64>) -> PyResult<NativeStep> {
        let action = wrench_from_vector(&action)?;
        self.inner
            .step(action)
            .map(step_to_native)
            .map_err(map_plant_error)
    }

    /// 原生层也公开实际时序，便于诊断绑定是否与 Rust plant 一致。
    fn timing(&self) -> (f64, u32) {
        let timing = self.inner.timing();
        (
            timing.physics_step_seconds(),
            timing.substeps_per_control.get(),
        )
    }
}

fn timing_from_python(
    physics_step_seconds: f64,
    substeps_per_control: u32,
) -> PyResult<SimulationTiming> {
    if !physics_step_seconds.is_finite() || physics_step_seconds <= 0.0 {
        return Err(PyValueError::new_err(
            "physics_step_seconds must be finite and positive",
        ));
    }

    let nanoseconds = physics_step_seconds * 1.0e9;
    let rounded_nanoseconds = nanoseconds.round();
    if rounded_nanoseconds < 1.0
        || rounded_nanoseconds > u64::MAX as f64
        || (nanoseconds - rounded_nanoseconds).abs() > 1.0e-6
    {
        return Err(PyValueError::new_err(
            "physics_step_seconds must be representable as a positive integer number of nanoseconds",
        ));
    }

    SimulationTiming::from_raw(rounded_nanoseconds as u64, substeps_per_control).ok_or_else(|| {
        PyValueError::new_err("substeps_per_control must be a positive 32-bit integer")
    })
}

fn state_from_vector(value: &[f64]) -> PyResult<ApolloState> {
    if value.len() != 13 {
        return Err(PyValueError::new_err(format!(
            "state must contain 13 values, got {}",
            value.len()
        )));
    }

    let state = ApolloState {
        position_body_origin_world_m: DVec3::new(value[0], value[1], value[2]),
        // Python 使用 wxyz；glam 构造函数使用 xyzw。
        body_to_world: DQuat::from_xyzw(value[4], value[5], value[6], value[3]),
        linear_velocity_body_origin_world_mps: DVec3::new(value[7], value[8], value[9]),
        angular_velocity_body_radps: DVec3::new(value[10], value[11], value[12]),
    };
    state
        .validate()
        .map_err(|error| PyValueError::new_err(error.to_string()))?;
    Ok(state)
}

fn wrench_from_vector(value: &[f64]) -> PyResult<BodyWrench> {
    if value.len() != 6 {
        return Err(PyValueError::new_err(format!(
            "body wrench must contain 6 values, got {}",
            value.len()
        )));
    }

    let wrench = BodyWrench {
        force_body_n: DVec3::new(value[0], value[1], value[2]),
        torque_about_com_body_nm: DVec3::new(value[3], value[4], value[5]),
    };
    wrench
        .validate()
        .map_err(|error| PyValueError::new_err(error.to_string()))?;
    Ok(wrench)
}

fn state_to_vector(state: ApolloState) -> Vec<f64> {
    vec![
        state.position_body_origin_world_m.x,
        state.position_body_origin_world_m.y,
        state.position_body_origin_world_m.z,
        state.body_to_world.w,
        state.body_to_world.x,
        state.body_to_world.y,
        state.body_to_world.z,
        state.linear_velocity_body_origin_world_mps.x,
        state.linear_velocity_body_origin_world_mps.y,
        state.linear_velocity_body_origin_world_mps.z,
        state.angular_velocity_body_radps.x,
        state.angular_velocity_body_radps.y,
        state.angular_velocity_body_radps.z,
    ]
}

fn wrench_to_vector(wrench: BodyWrench) -> Vec<f64> {
    vec![
        wrench.force_body_n.x,
        wrench.force_body_n.y,
        wrench.force_body_n.z,
        wrench.torque_about_com_body_nm.x,
        wrench.torque_about_com_body_nm.y,
        wrench.torque_about_com_body_nm.z,
    ]
}

fn snapshot_to_native(snapshot: PlantSnapshot) -> NativeSnapshot {
    (
        state_to_vector(snapshot.state),
        snapshot.control_tick,
        snapshot.physics_tick,
    )
}

fn step_to_native(step: PlantStep) -> NativeStep {
    (
        snapshot_to_native(step.snapshot),
        wrench_to_vector(step.requested_action),
        wrench_to_vector(step.applied_action),
    )
}

fn map_plant_error(error: PlantError) -> PyErr {
    match error {
        PlantError::InvalidModelSpec(_)
        | PlantError::InvalidInitialState(_)
        | PlantError::InvalidAction(_) => PyValueError::new_err(error.to_string()),
        PlantError::ModelLoad(_)
        | PlantError::BodyNotFound { .. }
        | PlantError::UnexpectedModelLayout { .. }
        | PlantError::DataAllocation(_)
        | PlantError::InvalidSimulationState(_)
        | PlantError::TickOverflow => PyRuntimeError::new_err(error.to_string()),
    }
}

#[pymodule]
fn _apollo_native(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<PyApolloPlantFactory>()?;
    module.add_class::<PyApolloPlant>()?;
    Ok(())
}
