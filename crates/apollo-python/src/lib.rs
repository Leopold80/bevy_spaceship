//! `apollo_sim` 的 PyO3 原生薄绑定。
//!
//! Python 领域对象和 NumPy 校验位于纯 Python 层；此处只进行固定顺序的
//! `Vec<f64>` 转换、Rust 输入复验和 MuJoCo 错误映射。

use apollo_core::{
    ApolloModelSpec, ApolloPropulsionSpec, ApolloState, AppliedDps, AppliedPropulsion, BodyWrench,
    DpsCommand, DpsMode, PlantSnapshot, PlantStep, PropulsionCommand, PropulsionStep,
    RCS_THRUSTER_COUNT, RcsCommand, RcsFeedSystem, RcsQuad, SimulationTiming,
};
use apollo_mujoco::{
    ApolloPlant as RustApolloPlant, ApolloPlantFactory as RustApolloPlantFactory,
    ApolloPropulsionPlant as RustApolloPropulsionPlant,
    ApolloPropulsionPlantFactory as RustApolloPropulsionPlantFactory, PlantError,
};
use glam::{DQuat, DVec3};
use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;

type NativeSnapshot = (Vec<f64>, u64, u64);
type NativeStep = (NativeSnapshot, Vec<f64>, Vec<f64>);
type NativeModelSpec = (String, f64, Vec<f64>, Vec<f64>);
type NativeDpsCommand = (String, Option<f64>, f64, f64);
type NativePropulsionCommand = (Vec<u64>, NativeDpsCommand);
type NativeAppliedDps = (String, f64, f64, f64, Vec<f64>);
type NativeAppliedPropulsion = (Vec<u64>, Vec<f64>, NativeAppliedDps, Vec<f64>);
type NativePropulsionStep = (
    NativeSnapshot,
    NativePropulsionCommand,
    NativeAppliedPropulsion,
);
type NativeRcsThrusterSpec = (usize, String, u8, String, Vec<f64>, Vec<f64>, f64, u64);
type NativeDpsSpec = (Vec<f64>, Vec<f64>, f64, f64, f64, f64, f64);
type NativePropulsionSpec = (Vec<NativeRcsThrusterSpec>, NativeDpsSpec);

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
        model_spec_to_native(self.inner.model_spec())
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

#[pyclass(name = "ApolloPropulsionPlantFactory", unsendable)]
struct PyApolloPropulsionPlantFactory {
    inner: RustApolloPropulsionPlantFactory,
}

#[pymethods]
impl PyApolloPropulsionPlantFactory {
    #[new]
    fn new(physics_step_seconds: f64, substeps_per_control: u32) -> PyResult<Self> {
        let timing = timing_from_python(physics_step_seconds, substeps_per_control)?;
        let inner = RustApolloPropulsionPlantFactory::new(
            ApolloModelSpec::touchdown(),
            ApolloPropulsionSpec::apollo11_touchdown(),
            timing,
        )
        .map_err(map_plant_error)?;
        Ok(Self { inner })
    }

    fn spawn(&self, initial_state: Vec<f64>) -> PyResult<PyApolloPropulsionPlant> {
        let state = state_from_vector(&initial_state)?;
        let inner = self.inner.spawn(state).map_err(map_plant_error)?;
        Ok(PyApolloPropulsionPlant { inner })
    }

    fn model_spec(&self) -> NativeModelSpec {
        model_spec_to_native(self.inner.model_spec())
    }

    fn propulsion_spec(&self) -> NativePropulsionSpec {
        propulsion_spec_to_native(self.inner.propulsion_spec())
    }
}

#[pyclass(name = "ApolloPropulsionPlant", unsendable)]
struct PyApolloPropulsionPlant {
    inner: RustApolloPropulsionPlant,
}

#[pymethods]
impl PyApolloPropulsionPlant {
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

    fn step(
        &mut self,
        rcs_on_time_ns: Vec<u64>,
        dps_mode: &str,
        dps_thrust_n: Option<f64>,
        gimbal_x_rad: f64,
        gimbal_z_rad: f64,
    ) -> PyResult<NativePropulsionStep> {
        let command = propulsion_command_from_native_parts(
            &rcs_on_time_ns,
            dps_mode,
            dps_thrust_n,
            gimbal_x_rad,
            gimbal_z_rad,
        )?;
        self.inner
            .step(command)
            .map(propulsion_step_to_native)
            .map_err(map_plant_error)
    }

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

fn model_spec_to_native(spec: ApolloModelSpec) -> NativeModelSpec {
    (
        spec.name.to_owned(),
        spec.mass_kg,
        spec.center_of_mass_body_m.to_array().to_vec(),
        spec.diagonal_inertia_body_kg_m2.to_array().to_vec(),
    )
}

fn propulsion_spec_to_native(spec: ApolloPropulsionSpec) -> NativePropulsionSpec {
    let rcs = spec
        .rcs_thrusters
        .into_iter()
        .map(|thruster| {
            let quad = match thruster.quad {
                RcsQuad::Quad1 => 1,
                RcsQuad::Quad2 => 2,
                RcsQuad::Quad3 => 3,
                RcsQuad::Quad4 => 4,
            };
            let feed_system = match thruster.feed_system {
                RcsFeedSystem::A => "A",
                RcsFeedSystem::B => "B",
            };
            (
                thruster.id.index(),
                thruster.label.to_owned(),
                quad,
                feed_system.to_owned(),
                thruster.position_body_m.to_array().to_vec(),
                thruster.force_direction_body.to_array().to_vec(),
                thruster.steady_thrust_n,
                thruster.minimum_pulse_ns,
            )
        })
        .collect();
    let dps = spec.dps;
    (
        rcs,
        (
            dps.gimbal_pivot_body_m.to_array().to_vec(),
            dps.nominal_force_direction_body.to_array().to_vec(),
            dps.variable_min_thrust_n,
            dps.variable_max_thrust_n,
            dps.full_thrust_n,
            dps.maximum_gimbal_rad,
            dps.gimbal_rate_rad_s,
        ),
    )
}

fn propulsion_command_from_native_parts(
    rcs_on_time_ns: &[u64],
    dps_mode: &str,
    dps_thrust_n: Option<f64>,
    gimbal_x_rad: f64,
    gimbal_z_rad: f64,
) -> PyResult<PropulsionCommand> {
    let on_time_ns: [u64; RCS_THRUSTER_COUNT] = rcs_on_time_ns.try_into().map_err(|_| {
        PyValueError::new_err(format!(
            "RCS command must contain {RCS_THRUSTER_COUNT} on-time values, got {}",
            rcs_on_time_ns.len()
        ))
    })?;
    let dps = match dps_mode {
        "off" => {
            if dps_thrust_n.is_some() || gimbal_x_rad != 0.0 || gimbal_z_rad != 0.0 {
                return Err(PyValueError::new_err(
                    "DPS off mode does not accept thrust or gimbal values",
                ));
            }
            DpsCommand::Off
        }
        "variable" => DpsCommand::Variable {
            thrust_n: dps_thrust_n
                .ok_or_else(|| PyValueError::new_err("DPS variable mode requires thrust_n"))?,
            gimbal_x_rad,
            gimbal_z_rad,
        },
        "full_thrust" => {
            if dps_thrust_n.is_some() {
                return Err(PyValueError::new_err(
                    "DPS full_thrust mode does not accept thrust_n",
                ));
            }
            DpsCommand::FullThrust {
                gimbal_x_rad,
                gimbal_z_rad,
            }
        }
        other => {
            return Err(PyValueError::new_err(format!("unknown DPS mode: {other}")));
        }
    };
    let command = PropulsionCommand {
        rcs: RcsCommand::from_on_times(on_time_ns),
        dps,
    };
    command
        .validate()
        .map_err(|error| PyValueError::new_err(error.to_string()))?;
    Ok(command)
}

fn dps_mode_to_native(mode: DpsMode) -> String {
    match mode {
        DpsMode::Off => "off",
        DpsMode::Variable => "variable",
        DpsMode::FullThrust => "full_thrust",
    }
    .to_owned()
}

fn dps_command_to_native(command: DpsCommand) -> NativeDpsCommand {
    match command {
        DpsCommand::Off => ("off".to_owned(), None, 0.0, 0.0),
        DpsCommand::Variable {
            thrust_n,
            gimbal_x_rad,
            gimbal_z_rad,
        } => (
            "variable".to_owned(),
            Some(thrust_n),
            gimbal_x_rad,
            gimbal_z_rad,
        ),
        DpsCommand::FullThrust {
            gimbal_x_rad,
            gimbal_z_rad,
        } => ("full_thrust".to_owned(), None, gimbal_x_rad, gimbal_z_rad),
    }
}

fn propulsion_command_to_native(command: PropulsionCommand) -> NativePropulsionCommand {
    (
        command.rcs.on_time_ns.to_vec(),
        dps_command_to_native(command.dps),
    )
}

fn applied_dps_to_native(applied: AppliedDps) -> NativeAppliedDps {
    (
        dps_mode_to_native(applied.mode),
        applied.thrust_n,
        applied.gimbal_x_rad,
        applied.gimbal_z_rad,
        applied.force_direction_body.to_array().to_vec(),
    )
}

fn applied_propulsion_to_native(applied: AppliedPropulsion) -> NativeAppliedPropulsion {
    (
        applied
            .rcs
            .iter()
            .map(|thruster| thruster.applied_gate_on_time_ns)
            .collect(),
        applied
            .rcs
            .iter()
            .map(|thruster| thruster.mean_thrust_n)
            .collect(),
        applied_dps_to_native(applied.dps),
        wrench_to_vector(applied.mean_wrench_body),
    )
}

fn propulsion_step_to_native(step: PropulsionStep) -> NativePropulsionStep {
    (
        snapshot_to_native(step.snapshot),
        propulsion_command_to_native(step.requested_command),
        applied_propulsion_to_native(step.applied),
    )
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
        | PlantError::InvalidAction(_)
        | PlantError::InvalidPropulsionSpec(_)
        | PlantError::InvalidPropulsionCommand(_) => PyValueError::new_err(error.to_string()),
        PlantError::ModelLoad(_)
        | PlantError::BodyNotFound { .. }
        | PlantError::UnexpectedModelLayout { .. }
        | PlantError::DataAllocation(_)
        | PlantError::ForceApplication(_)
        | PlantError::InvalidSimulationState(_)
        | PlantError::TickOverflow => PyRuntimeError::new_err(error.to_string()),
    }
}

#[pymodule]
fn _apollo_native(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<PyApolloPlantFactory>()?;
    module.add_class::<PyApolloPlant>()?;
    module.add_class::<PyApolloPropulsionPlantFactory>()?;
    module.add_class::<PyApolloPropulsionPlant>()?;
    Ok(())
}
