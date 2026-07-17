use crate::{PlantError, generate_apollo_mjcf, rcs_actuator::RcsActuatorState};
use apollo_core::{
    ApolloModelSpec, ApolloPropulsionSpec, ApolloState, AppliedDps, AppliedPropulsion,
    AppliedRcsThruster, BodyWrench, DpsCommand, DpsMode, DpsSpec, PlantSnapshot, PropulsionCommand,
    PropulsionStep, RCS_THRUSTER_COUNT, SimulationTiming,
};
use glam::{DQuat, DVec2, DVec3};
use mujoco_rs::prelude::{MjData, MjModel, MjtObj};
use std::sync::Arc;

/// 可复用的 Apollo 11 LM 推进被控对象工厂。
///
/// 本工厂与理想 [`crate::ApolloPlantFactory`] 并列：前者接收 16 路 RCS 与
/// DPS 命令，后者继续接收任意机体系 wrench。两条接口不会隐式互相分配。
#[derive(Clone)]
pub struct ApolloPropulsionPlantFactory {
    model: Arc<MjModel>,
    body_id: usize,
    model_spec: ApolloModelSpec,
    propulsion_spec: ApolloPropulsionSpec,
    timing: SimulationTiming,
}

impl ApolloPropulsionPlantFactory {
    pub fn new(
        model_spec: ApolloModelSpec,
        propulsion_spec: ApolloPropulsionSpec,
        timing: SimulationTiming,
    ) -> Result<Self, PlantError> {
        propulsion_spec
            .validate_for_timing(timing)
            .map_err(|error| PlantError::InvalidPropulsionSpec(error.to_string()))?;

        let xml = generate_apollo_mjcf(model_spec, timing)?;
        let model = Arc::new(
            MjModel::from_xml_string(&xml)
                .map_err(|error| PlantError::ModelLoad(error.to_string()))?,
        );
        let body_id = model
            .name_to_id(MjtObj::mjOBJ_BODY, model_spec.name)
            .ok_or(PlantError::BodyNotFound {
                body_name: model_spec.name,
            })?;

        let nq = model.nq() as usize;
        let nv = model.nv() as usize;
        if nq != 7 || nv != 6 {
            return Err(PlantError::UnexpectedModelLayout { nq, nv });
        }

        Ok(Self {
            model,
            body_id,
            model_spec,
            propulsion_spec,
            timing,
        })
    }

    /// Apollo 11 LM-5 轻载着陆构型，采用 2 ms × 10 的默认时序。
    pub fn apollo11_touchdown() -> Result<Self, PlantError> {
        Self::new(
            ApolloModelSpec::touchdown(),
            ApolloPropulsionSpec::apollo11_touchdown(),
            SimulationTiming::APOLLO,
        )
    }

    pub fn spawn(&self, initial_state: ApolloState) -> Result<ApolloPropulsionPlant, PlantError> {
        initial_state
            .validate()
            .map_err(PlantError::InvalidInitialState)?;
        let data = MjData::try_new(self.model.clone())
            .map_err(|error| PlantError::DataAllocation(error.to_string()))?;
        let mut plant = ApolloPropulsionPlant {
            data,
            body_id: self.body_id,
            model_spec: self.model_spec,
            propulsion_spec: self.propulsion_spec,
            timing: self.timing,
            control_tick: 0,
            physics_tick: 0,
            rcs_actuators: [RcsActuatorState::default(); RCS_THRUSTER_COUNT],
            dps_gimbal_rad: DVec2::ZERO,
        };
        plant.reset(initial_state)?;
        Ok(plant)
    }

    pub fn model_spec(&self) -> ApolloModelSpec {
        self.model_spec
    }

    pub fn propulsion_spec(&self) -> ApolloPropulsionSpec {
        self.propulsion_spec
    }

    pub fn timing(&self) -> SimulationTiming {
        self.timing
    }
}

/// 由 16 个 RCS 点力和一个可摆动 DPS 点力驱动的同步 Apollo 被控对象。
///
/// RCS 阀门瞬态跨控制周期保存；所有喷口都在各自历史站位施力，因此平动与
/// 转动由同一组点力自然产生。DPS 目标命令在一个控制周期内采用零阶
/// 保持，万向节实际位置则在每个物理子步按 GDA 速率限制追踪目标。
pub struct ApolloPropulsionPlant {
    data: MjData<Arc<MjModel>>,
    body_id: usize,
    model_spec: ApolloModelSpec,
    propulsion_spec: ApolloPropulsionSpec,
    timing: SimulationTiming,
    control_tick: u64,
    physics_tick: u64,
    rcs_actuators: [RcsActuatorState; RCS_THRUSTER_COUNT],
    /// DPS 实际万向节位置：x/z 分量，单位 rad。
    dps_gimbal_rad: DVec2,
}

impl ApolloPropulsionPlant {
    pub fn reset(&mut self, initial_state: ApolloState) -> Result<PlantSnapshot, PlantError> {
        initial_state
            .validate()
            .map_err(PlantError::InvalidInitialState)?;

        self.data.reset();
        {
            let qpos = self.data.qpos_mut();
            qpos[0] = initial_state.position_body_origin_world_m.x;
            qpos[1] = initial_state.position_body_origin_world_m.y;
            qpos[2] = initial_state.position_body_origin_world_m.z;
            qpos[3] = initial_state.body_to_world.w;
            qpos[4] = initial_state.body_to_world.x;
            qpos[5] = initial_state.body_to_world.y;
            qpos[6] = initial_state.body_to_world.z;
        }
        {
            let qvel = self.data.qvel_mut();
            qvel[0] = initial_state.linear_velocity_body_origin_world_mps.x;
            qvel[1] = initial_state.linear_velocity_body_origin_world_mps.y;
            qvel[2] = initial_state.linear_velocity_body_origin_world_mps.z;
            qvel[3] = initial_state.angular_velocity_body_radps.x;
            qvel[4] = initial_state.angular_velocity_body_radps.y;
            qvel[5] = initial_state.angular_velocity_body_radps.z;
        }
        self.clear_applied_forces();
        self.data.forward();
        self.control_tick = 0;
        self.physics_tick = 0;
        for actuator in &mut self.rcs_actuators {
            actuator.reset();
        }
        self.dps_gimbal_rad = DVec2::ZERO;
        Ok(self.snapshot())
    }

    /// 推进一个控制周期并返回请求、实际执行结果和新状态。
    pub fn step(&mut self, command: PropulsionCommand) -> Result<PropulsionStep, PlantError> {
        command
            .validate()
            .map_err(|error| PlantError::InvalidPropulsionCommand(error.to_string()))?;
        // 最小脉冲只约束 OFF -> ON 的新点火。上一 tick 在边界仍开启的阀门若继续
        // 1 ms，就是同一次连续点火的 1 ms 末段，不能再次被提升到 14 ms。
        let gate_open_at_start =
            std::array::from_fn(|index| self.rcs_actuators[index].is_gate_open());
        let applied_gate_on_times = command
            .rcs
            .applied_gate_on_times_with_initial_gate_state(
                self.propulsion_spec,
                self.timing,
                gate_open_at_start,
            )
            .map_err(|error| PlantError::InvalidPropulsionCommand(error.to_string()))?;
        // DpsSpec::apply 只解析工作档、推力和经圆锥限制的目标摆角；
        // 有状态 GDA 在后续物理子步内追踪该目标。
        let target_dps = self
            .propulsion_spec
            .dps
            .apply(command.dps)
            .map_err(|error| PlantError::InvalidPropulsionCommand(error.to_string()))?;

        // 在修改执行器或 MuJoCo 状态前完成溢出检查，命令错误保持无副作用。
        let next_control_tick = self
            .control_tick
            .checked_add(1)
            .ok_or(PlantError::TickOverflow)?;
        let substeps = self.timing.substeps_per_control.get();
        let next_physics_tick = self
            .physics_tick
            .checked_add(u64::from(substeps))
            .ok_or(PlantError::TickOverflow)?;

        let physics_step_ns = self.timing.physics_step_ns.get();
        let physics_step_seconds = self.timing.physics_step_seconds();
        let control_step_seconds = self.timing.control_step_seconds();
        let mut rcs_impulse_n_s = [0.0_f64; RCS_THRUSTER_COUNT];
        let mut force_impulse_body_n_s = DVec3::ZERO;
        let mut torque_impulse_body_nm_s = DVec3::ZERO;

        for substep in 0..substeps {
            let interval_start_ns = u64::from(substep) * physics_step_ns;
            let mut qfrc = vec![0.0; self.data.model().nv() as usize];
            let state = self.read_state();
            let body_origin_world = state.position_body_origin_world_m;
            let body_to_world = state.body_to_world;
            let mut force_body_n = DVec3::ZERO;
            let mut torque_body_nm = DVec3::ZERO;

            for index in 0..RCS_THRUSTER_COUNT {
                let thruster = self.propulsion_spec.rcs_thrusters[index];
                let mean_fraction = self.rcs_actuators[index].advance_scheduled_interval(
                    interval_start_ns,
                    physics_step_ns,
                    applied_gate_on_times[index],
                );
                let mean_thrust_n = mean_fraction * thruster.steady_thrust_n;
                rcs_impulse_n_s[index] += mean_thrust_n * physics_step_seconds;
                if mean_thrust_n == 0.0 {
                    continue;
                }

                let point_force_body = thruster.force_direction_body * mean_thrust_n;
                self.apply_point_force(
                    body_origin_world,
                    body_to_world,
                    thruster.position_body_m,
                    point_force_body,
                    &mut qfrc,
                )?;
                force_body_n += point_force_body;
                torque_body_nm += (thruster.position_body_m
                    - self.model_spec.center_of_mass_body_m)
                    .cross(point_force_body);
            }

            // Apollo 11 的两台 GDA 是带制动的电机—丝杠机构。OFF 时不回中，
            // 而是保持最后位置；有推力档时按二维角度矢量模长限速。
            let gimbal_for_force = if target_dps.mode == DpsMode::Off {
                self.dps_gimbal_rad
            } else {
                let start = self.dps_gimbal_rad;
                self.dps_gimbal_rad = slew_gimbal_toward(
                    start,
                    DVec2::new(target_dps.gimbal_x_rad, target_dps.gimbal_z_rad),
                    self.propulsion_spec.dps.gimbal_rate_rad_s * physics_step_seconds,
                );
                // 子步中点位置对线性摆动做二阶准确的力积分。
                (start + self.dps_gimbal_rad) * 0.5
            };
            let substep_dps =
                applied_dps_at_gimbal(self.propulsion_spec.dps, target_dps, gimbal_for_force);

            if substep_dps.thrust_n > 0.0 {
                let point_force_body = substep_dps.force_direction_body * substep_dps.thrust_n;
                self.apply_point_force(
                    body_origin_world,
                    body_to_world,
                    self.propulsion_spec.dps.gimbal_pivot_body_m,
                    point_force_body,
                    &mut qfrc,
                )?;
                force_body_n += point_force_body;
                torque_body_nm += (self.propulsion_spec.dps.gimbal_pivot_body_m
                    - self.model_spec.center_of_mass_body_m)
                    .cross(point_force_body);
            }

            self.data.qfrc_applied_mut().copy_from_slice(&qfrc);
            force_impulse_body_n_s += force_body_n * physics_step_seconds;
            torque_impulse_body_nm_s += torque_body_nm * physics_step_seconds;
            self.data.step();
        }
        self.clear_applied_forces();

        self.control_tick = next_control_tick;
        self.physics_tick = next_physics_tick;
        let snapshot = self.snapshot();
        snapshot
            .state
            .validate()
            .map_err(PlantError::InvalidSimulationState)?;

        let rcs = std::array::from_fn(|index| AppliedRcsThruster {
            applied_gate_on_time_ns: applied_gate_on_times[index],
            mean_thrust_n: rcs_impulse_n_s[index] / control_step_seconds,
        });
        let mean_wrench_body = BodyWrench {
            force_body_n: force_impulse_body_n_s / control_step_seconds,
            torque_about_com_body_nm: torque_impulse_body_nm_s / control_step_seconds,
        };
        debug_assert!(mean_wrench_body.validate().is_ok());
        let applied_dps =
            applied_dps_at_gimbal(self.propulsion_spec.dps, target_dps, self.dps_gimbal_rad);

        Ok(PropulsionStep {
            snapshot,
            requested_command: command,
            applied: AppliedPropulsion {
                rcs,
                dps: applied_dps,
                mean_wrench_body,
            },
        })
    }

    pub fn snapshot(&self) -> PlantSnapshot {
        PlantSnapshot {
            state: self.read_state(),
            control_tick: self.control_tick,
            physics_tick: self.physics_tick,
        }
    }

    pub fn timing(&self) -> SimulationTiming {
        self.timing
    }

    pub fn propulsion_spec(&self) -> ApolloPropulsionSpec {
        self.propulsion_spec
    }

    fn read_state(&self) -> ApolloState {
        let qpos = self.data.qpos();
        let qvel = self.data.qvel();
        ApolloState {
            position_body_origin_world_m: DVec3::new(qpos[0], qpos[1], qpos[2]),
            body_to_world: DQuat::from_xyzw(qpos[4], qpos[5], qpos[6], qpos[3]).normalize(),
            linear_velocity_body_origin_world_mps: DVec3::new(qvel[0], qvel[1], qvel[2]),
            angular_velocity_body_radps: DVec3::new(qvel[3], qvel[4], qvel[5]),
        }
    }

    fn apply_point_force(
        &mut self,
        body_origin_world_m: DVec3,
        body_to_world: DQuat,
        point_body_m: DVec3,
        force_body_n: DVec3,
        qfrc: &mut [f64],
    ) -> Result<(), PlantError> {
        let point_world_m = body_origin_world_m + body_to_world * point_body_m;
        let force_world_n = body_to_world * force_body_n;
        self.data
            .apply_ft(
                &force_world_n.to_array(),
                &[0.0; 3],
                &point_world_m.to_array(),
                self.body_id,
                qfrc,
            )
            .map_err(|error| PlantError::ForceApplication(error.to_string()))
    }

    fn clear_applied_forces(&mut self) {
        self.data.qfrc_applied_mut().fill(0.0);
        self.data.xfrc_applied_mut().fill([0.0; 6]);
    }
}

/// 在一个物理子步内沿目标方向移动，不超过给定的二维角度增量。
fn slew_gimbal_toward(current: DVec2, target: DVec2, maximum_delta_rad: f64) -> DVec2 {
    let delta = target - current;
    let distance = delta.length();
    if distance <= maximum_delta_rad {
        target
    } else {
        current + delta * (maximum_delta_rad / distance)
    }
}

/// 使用已解析的推力/工作档和指定的实际摆角组合一份执行结果。
fn applied_dps_at_gimbal(spec: DpsSpec, target: AppliedDps, gimbal_rad: DVec2) -> AppliedDps {
    // 这里借用 FullThrust 分支的纯几何计算；工作档和推力随后恢复为
    // 预先验证过的 target 值。gimbal_rad 由零点和已限制目标间的线段产生。
    let geometry = spec
        .apply(DpsCommand::FullThrust {
            gimbal_x_rad: gimbal_rad.x,
            gimbal_z_rad: gimbal_rad.y,
        })
        .expect("validated DPS gimbal state must remain valid");
    AppliedDps {
        mode: target.mode,
        thrust_n: target.thrust_n,
        gimbal_x_rad: geometry.gimbal_x_rad,
        gimbal_z_rad: geometry.gimbal_z_rad,
        force_direction_body: geometry.force_direction_body,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use apollo_core::{DpsCommand, RCS_MINIMUM_PULSE_NS, RcsCommand, RcsThrusterId};
    use std::num::{NonZeroU32, NonZeroU64};

    fn factory() -> ApolloPropulsionPlantFactory {
        ApolloPropulsionPlantFactory::apollo11_touchdown()
            .expect("Apollo propulsion model should load")
    }

    fn id(index: u8) -> RcsThrusterId {
        RcsThrusterId::new(index).unwrap()
    }

    #[test]
    fn factory_exposes_apollo11_specs_and_default_timing() {
        let factory = factory();
        assert_eq!(factory.model_spec(), ApolloModelSpec::touchdown());
        assert_eq!(
            factory.propulsion_spec(),
            ApolloPropulsionSpec::apollo11_touchdown()
        );
        assert_eq!(factory.timing(), SimulationTiming::APOLLO);
    }

    #[test]
    fn overlong_pulse_is_rejected_without_advancing_state() {
        let mut plant = factory().spawn(ApolloState::default()).unwrap();
        let before = plant.snapshot();
        let result = plant.step(PropulsionCommand {
            rcs: RcsCommand::single_pulse(id(0), 20_000_001),
            dps: DpsCommand::Off,
        });
        assert!(matches!(
            result,
            Err(PlantError::InvalidPropulsionCommand(_))
        ));
        assert_eq!(plant.snapshot(), before);
    }

    #[test]
    fn all_sixteen_rcs_jets_generate_their_specified_point_force_and_moment() {
        let model_spec = ApolloModelSpec::touchdown();
        let propulsion_spec = ApolloPropulsionSpec::apollo11_touchdown();

        for index in 0..RCS_THRUSTER_COUNT {
            let mut plant = factory().spawn(ApolloState::default()).unwrap();
            let thruster = propulsion_spec.rcs_thrusters[index];
            let step = plant
                .step(PropulsionCommand {
                    rcs: RcsCommand::single_pulse(id(index as u8), 20_000_000),
                    dps: DpsCommand::Off,
                })
                .unwrap();

            let mean_thrust_n = step.applied.rcs[index].mean_thrust_n;
            assert!(mean_thrust_n > 0.0, "{} produced no thrust", thruster.label);
            assert_eq!(step.applied.rcs[index].applied_gate_on_time_ns, 20_000_000);
            let expected_force = thruster.force_direction_body * mean_thrust_n;
            let expected_torque =
                (thruster.position_body_m - model_spec.center_of_mass_body_m).cross(expected_force);
            assert!(
                step.applied
                    .mean_wrench_body
                    .force_body_n
                    .distance(expected_force)
                    < 1.0e-9,
                "{} force mismatch",
                thruster.label
            );
            assert!(
                step.applied
                    .mean_wrench_body
                    .torque_about_com_body_nm
                    .distance(expected_torque)
                    < 1.0e-9,
                "{} moment mismatch",
                thruster.label
            );

            // 用 MuJoCo 推进后的质心速度复核点力确实进入了刚体，而不只是
            // 在返回值中做了代数合成。
            let state = step.snapshot.state;
            let offset_world = state.body_to_world * model_spec.center_of_mass_body_m;
            let angular_velocity_world = state.body_to_world * state.angular_velocity_body_radps;
            let com_velocity_world = state.linear_velocity_body_origin_world_mps
                + angular_velocity_world.cross(offset_world);
            let force_world = state.body_to_world * thruster.force_direction_body;
            assert!(
                com_velocity_world.dot(force_world) > 0.0,
                "{} MuJoCo response opposes its force",
                thruster.label
            );
        }
    }

    #[test]
    fn dps_variable_mode_clamps_thrust_and_applies_gimbal() {
        let mut plant = factory().spawn(ApolloState::default()).unwrap();
        let command = PropulsionCommand {
            rcs: RcsCommand::OFF,
            dps: DpsCommand::Variable {
                thrust_n: 1.0,
                gimbal_x_rad: 2.0_f64.to_radians(),
                gimbal_z_rad: -1.0_f64.to_radians(),
            },
        };
        let step = plant.step(command).unwrap();

        let dps = plant.propulsion_spec().dps;
        assert_eq!(step.applied.dps.thrust_n, dps.variable_min_thrust_n);
        let expected_delta = dps.gimbal_rate_rad_s * plant.timing().control_step_seconds();
        assert!(
            (step
                .applied
                .dps
                .gimbal_x_rad
                .hypot(step.applied.dps.gimbal_z_rad)
                - expected_delta)
                .abs()
                < 1.0e-15
        );
        assert!((expected_delta.to_degrees() - 0.004).abs() < 1.0e-15);
        assert!(step.applied.dps.force_direction_body.x > 0.0);
        assert!(step.applied.dps.force_direction_body.y > 0.0);
        assert!(step.applied.dps.force_direction_body.z < 0.0);
        assert!(step.applied.mean_wrench_body.force_body_n.y > 0.0);

        // 返回方向必须由限速后的实际摆角计算，而不是请求目标。
        let expected_geometry = dps
            .apply(DpsCommand::FullThrust {
                gimbal_x_rad: step.applied.dps.gimbal_x_rad,
                gimbal_z_rad: step.applied.dps.gimbal_z_rad,
            })
            .unwrap();
        assert!(
            step.applied
                .dps
                .force_direction_body
                .distance(expected_geometry.force_direction_body)
                < 1.0e-15
        );
        let target = dps.apply(command.dps).unwrap();
        assert!(
            step.applied
                .dps
                .force_direction_body
                .distance(target.force_direction_body)
                > 1.0e-3
        );
    }

    #[test]
    fn dps_target_is_cone_clamped_before_rate_limiting() {
        let mut plant = factory().spawn(ApolloState::default()).unwrap();
        let step = plant
            .step(PropulsionCommand {
                rcs: RcsCommand::OFF,
                dps: DpsCommand::FullThrust {
                    gimbal_x_rad: 20.0_f64.to_radians(),
                    gimbal_z_rad: 20.0_f64.to_radians(),
                },
            })
            .unwrap();
        let dps = plant.propulsion_spec().dps;
        let magnitude = step
            .applied
            .dps
            .gimbal_x_rad
            .hypot(step.applied.dps.gimbal_z_rad);
        assert!(
            (magnitude - dps.gimbal_rate_rad_s * plant.timing().control_step_seconds()).abs()
                < 1.0e-15
        );
        assert!((step.applied.dps.gimbal_x_rad - step.applied.dps.gimbal_z_rad).abs() < 1.0e-15);
    }

    #[test]
    fn dps_gimbal_does_not_overshoot_a_nearby_target() {
        let mut plant = factory().spawn(ApolloState::default()).unwrap();
        let target_rad = 0.003_f64.to_radians();
        let step = plant
            .step(PropulsionCommand {
                rcs: RcsCommand::OFF,
                dps: DpsCommand::FullThrust {
                    gimbal_x_rad: target_rad,
                    gimbal_z_rad: 0.0,
                },
            })
            .unwrap();
        assert!((step.applied.dps.gimbal_x_rad - target_rad).abs() < 1.0e-15);
        assert_eq!(step.applied.dps.gimbal_z_rad, 0.0);
    }

    #[test]
    fn dps_off_holds_the_last_gimbal_position_without_thrust() {
        let mut plant = factory().spawn(ApolloState::default()).unwrap();
        let driven = plant
            .step(PropulsionCommand {
                rcs: RcsCommand::OFF,
                dps: DpsCommand::FullThrust {
                    gimbal_x_rad: 1.0_f64.to_radians(),
                    gimbal_z_rad: -2.0_f64.to_radians(),
                },
            })
            .unwrap();
        let held = plant.step(PropulsionCommand::OFF).unwrap();

        assert_eq!(held.applied.dps.mode, DpsMode::Off);
        assert_eq!(held.applied.dps.thrust_n, 0.0);
        assert_eq!(
            held.applied.dps.gimbal_x_rad,
            driven.applied.dps.gimbal_x_rad
        );
        assert_eq!(
            held.applied.dps.gimbal_z_rad,
            driven.applied.dps.gimbal_z_rad
        );
        assert_eq!(
            held.applied.dps.force_direction_body,
            driven.applied.dps.force_direction_body
        );
        assert_eq!(held.applied.mean_wrench_body, BodyWrench::ZERO);
    }

    #[test]
    fn reset_recenters_the_dps_gimbal() {
        let mut plant = factory().spawn(ApolloState::default()).unwrap();
        plant
            .step(PropulsionCommand {
                rcs: RcsCommand::OFF,
                dps: DpsCommand::FullThrust {
                    gimbal_x_rad: 1.0_f64.to_radians(),
                    gimbal_z_rad: 0.0,
                },
            })
            .unwrap();
        plant.reset(ApolloState::default()).unwrap();
        let step = plant.step(PropulsionCommand::OFF).unwrap();

        assert_eq!(step.applied.dps.gimbal_x_rad, 0.0);
        assert_eq!(step.applied.dps.gimbal_z_rad, 0.0);
        assert_eq!(
            step.applied.dps.force_direction_body,
            plant.propulsion_spec().dps.nominal_force_direction_body
        );
    }

    #[test]
    fn invalid_command_does_not_advance_the_dps_gimbal() {
        let mut plant = factory().spawn(ApolloState::default()).unwrap();
        let driven = plant
            .step(PropulsionCommand {
                rcs: RcsCommand::OFF,
                dps: DpsCommand::FullThrust {
                    gimbal_x_rad: 1.0_f64.to_radians(),
                    gimbal_z_rad: 0.0,
                },
            })
            .unwrap();
        let before = plant.snapshot();
        let invalid = plant.step(PropulsionCommand {
            rcs: RcsCommand::OFF,
            dps: DpsCommand::FullThrust {
                gimbal_x_rad: f64::NAN,
                gimbal_z_rad: 0.0,
            },
        });
        assert!(matches!(
            invalid,
            Err(PlantError::InvalidPropulsionCommand(_))
        ));
        assert_eq!(plant.snapshot(), before);

        let held = plant.step(PropulsionCommand::OFF).unwrap();
        assert_eq!(
            held.applied.dps.gimbal_x_rad,
            driven.applied.dps.gimbal_x_rad
        );
        assert_eq!(
            held.applied.dps.gimbal_z_rad,
            driven.applied.dps.gimbal_z_rad
        );
    }

    #[test]
    fn reset_removes_rcs_shutdown_tail() {
        let mut plant = factory().spawn(ApolloState::default()).unwrap();
        plant
            .step(PropulsionCommand {
                rcs: RcsCommand::single_pulse(id(0), 20_000_000),
                dps: DpsCommand::Off,
            })
            .unwrap();
        plant.reset(ApolloState::default()).unwrap();
        let step = plant.step(PropulsionCommand::OFF).unwrap();
        assert_eq!(step.applied.rcs[0].mean_thrust_n, 0.0);
        assert_eq!(step.snapshot.control_tick, 1);
    }

    #[test]
    fn continuous_gate_across_tick_boundary_does_not_reapply_minimum_pulse() {
        let mut plant = factory().spawn(ApolloState::default()).unwrap();
        let first = plant
            .step(PropulsionCommand {
                rcs: RcsCommand::single_pulse(id(0), 20_000_000),
                dps: DpsCommand::Off,
            })
            .unwrap();
        let continuation = plant
            .step(PropulsionCommand {
                rcs: RcsCommand::single_pulse(id(0), 1_000_000),
                dps: DpsCommand::Off,
            })
            .unwrap();

        assert_eq!(first.applied.rcs[0].applied_gate_on_time_ns, 20_000_000);
        assert_eq!(
            continuation.applied.rcs[0].applied_gate_on_time_ns,
            1_000_000
        );
        assert_eq!(
            first.applied.rcs[0].applied_gate_on_time_ns
                + continuation.applied.rcs[0].applied_gate_on_time_ns,
            21_000_000
        );
    }

    #[test]
    fn new_pulse_after_a_real_shutdown_still_uses_minimum_pulse() {
        let mut plant = factory().spawn(ApolloState::default()).unwrap();
        plant
            .step(PropulsionCommand {
                rcs: RcsCommand::single_pulse(id(0), 20_000_000),
                dps: DpsCommand::Off,
            })
            .unwrap();
        plant.step(PropulsionCommand::OFF).unwrap();

        let restarted = plant
            .step(PropulsionCommand {
                rcs: RcsCommand::single_pulse(id(0), 1_000_000),
                dps: DpsCommand::Off,
            })
            .unwrap();

        assert_eq!(
            restarted.applied.rcs[0].applied_gate_on_time_ns,
            RCS_MINIMUM_PULSE_NS
        );
    }

    #[test]
    fn identical_command_sequences_are_deterministic() {
        let mut first = factory().spawn(ApolloState::default()).unwrap();
        let mut second = factory().spawn(ApolloState::default()).unwrap();
        let command = PropulsionCommand {
            rcs: RcsCommand::single_pulse(id(3), 14_000_001),
            dps: DpsCommand::FullThrust {
                gimbal_x_rad: 0.01,
                gimbal_z_rad: -0.02,
            },
        };
        for _ in 0..5 {
            assert_eq!(first.step(command).unwrap(), second.step(command).unwrap());
        }
    }

    #[test]
    fn rcs_two_and_one_millisecond_physics_steps_converge_over_one_second() {
        let spec = ApolloPropulsionSpec::apollo11_touchdown();
        let timing_2ms = SimulationTiming::APOLLO;
        let timing_1ms = SimulationTiming::new(
            NonZeroU64::new(1_000_000).unwrap(),
            NonZeroU32::new(20).unwrap(),
        );
        let mut plant_2ms =
            ApolloPropulsionPlantFactory::new(ApolloModelSpec::touchdown(), spec, timing_2ms)
                .unwrap()
                .spawn(ApolloState::default())
                .unwrap();
        let mut plant_1ms =
            ApolloPropulsionPlantFactory::new(ApolloModelSpec::touchdown(), spec, timing_1ms)
                .unwrap()
                .spawn(ApolloState::default())
                .unwrap();

        // 四个 downward-firing 喷口构成对称的 +Y 平移，避免转动误差污染比较。
        let rcs = RcsCommand::hold([id(1), id(5), id(9), id(13)], timing_2ms).unwrap();
        let command = PropulsionCommand {
            rcs,
            dps: DpsCommand::Off,
        };
        for _ in 0..50 {
            plant_2ms.step(command).unwrap();
            plant_1ms.step(command).unwrap();
        }

        let state_2ms = plant_2ms.snapshot().state;
        let state_1ms = plant_1ms.snapshot().state;
        assert!(
            state_2ms
                .position_body_origin_world_m
                .distance(state_1ms.position_body_origin_world_m)
                < 3.0e-4
        );
        assert!(
            state_2ms
                .linear_velocity_body_origin_world_mps
                .distance(state_1ms.linear_velocity_body_origin_world_mps)
                < 1.0e-8
        );
        assert!(
            state_2ms
                .angular_velocity_body_radps
                .distance(state_1ms.angular_velocity_body_radps)
                < 1.0e-10
        );
    }

    #[test]
    fn dps_gimbal_two_and_one_millisecond_steps_converge_over_one_second() {
        let spec = ApolloPropulsionSpec::apollo11_touchdown();
        let timing_2ms = SimulationTiming::APOLLO;
        let timing_1ms = SimulationTiming::new(
            NonZeroU64::new(1_000_000).unwrap(),
            NonZeroU32::new(20).unwrap(),
        );
        let mut plant_2ms =
            ApolloPropulsionPlantFactory::new(ApolloModelSpec::touchdown(), spec, timing_2ms)
                .unwrap()
                .spawn(ApolloState::default())
                .unwrap();
        let mut plant_1ms =
            ApolloPropulsionPlantFactory::new(ApolloModelSpec::touchdown(), spec, timing_1ms)
                .unwrap()
                .spawn(ApolloState::default())
                .unwrap();
        let command = PropulsionCommand {
            rcs: RcsCommand::OFF,
            dps: DpsCommand::Variable {
                thrust_n: spec.dps.variable_min_thrust_n,
                gimbal_x_rad: 0.3_f64.to_radians(),
                gimbal_z_rad: -0.4_f64.to_radians(),
            },
        };

        let mut end_2ms = None;
        let mut end_1ms = None;
        for _ in 0..50 {
            end_2ms = Some(plant_2ms.step(command).unwrap());
            end_1ms = Some(plant_1ms.step(command).unwrap());
        }
        let end_2ms = end_2ms.unwrap();
        let end_1ms = end_1ms.unwrap();
        let gimbal_2ms = DVec2::new(
            end_2ms.applied.dps.gimbal_x_rad,
            end_2ms.applied.dps.gimbal_z_rad,
        );
        let gimbal_1ms = DVec2::new(
            end_1ms.applied.dps.gimbal_x_rad,
            end_1ms.applied.dps.gimbal_z_rad,
        );
        assert!(gimbal_2ms.distance(gimbal_1ms) < 1.0e-14);
        assert!((gimbal_2ms.length().to_degrees() - 0.2).abs() < 1.0e-12);

        let state_2ms = end_2ms.snapshot.state;
        let state_1ms = end_1ms.snapshot.state;
        let position_error = state_2ms
            .position_body_origin_world_m
            .distance(state_1ms.position_body_origin_world_m);
        let velocity_error = state_2ms
            .linear_velocity_body_origin_world_mps
            .distance(state_1ms.linear_velocity_body_origin_world_mps);
        let angular_velocity_error = state_2ms
            .angular_velocity_body_radps
            .distance(state_1ms.angular_velocity_body_radps);
        assert!(position_error < 1.0e-3, "position error {position_error:e}");
        assert!(
            velocity_error < 1.0e-6,
            "linear velocity error {velocity_error:e}"
        );
        assert!(
            angular_velocity_error < 5.0e-7,
            "angular velocity error {angular_velocity_error:e}"
        );
    }
}
