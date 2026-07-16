use crate::{PlantError, generate_apollo_mjcf};
use apollo_core::{
    ApolloModelSpec, ApolloState, BodyWrench, Plant as PlantContract, PlantSnapshot, PlantStep,
    SimulationTiming,
};
use glam::{DQuat, DVec3};
use mujoco_rs::prelude::{MjData, MjModel, MjtObj};
use std::sync::Arc;

/// 可复用的 Apollo MuJoCo 模型工厂。
///
/// 工厂只共享不可变 `MjModel`；每次 [`spawn`](Self::spawn) 都分配独立
/// `MjData`，因此各实例的状态、外力和 tick 不会串扰。
#[derive(Clone)]
pub struct ApolloPlantFactory {
    model: Arc<MjModel>,
    body_id: usize,
    model_spec: ApolloModelSpec,
    timing: SimulationTiming,
}

impl ApolloPlantFactory {
    pub fn new(model_spec: ApolloModelSpec, timing: SimulationTiming) -> Result<Self, PlantError> {
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
            timing,
        })
    }

    /// 当前 Apollo 11 轻载着陆质量属性和 2 ms × 10 默认时序。
    pub fn apollo_touchdown() -> Result<Self, PlantError> {
        Self::new(ApolloModelSpec::touchdown(), SimulationTiming::APOLLO)
    }

    pub fn spawn(&self, initial_state: ApolloState) -> Result<ApolloPlant, PlantError> {
        initial_state
            .validate()
            .map_err(PlantError::InvalidInitialState)?;
        let data = MjData::try_new(self.model.clone())
            .map_err(|error| PlantError::DataAllocation(error.to_string()))?;
        let mut plant = ApolloPlant {
            data,
            body_id: self.body_id,
            timing: self.timing,
            control_tick: 0,
            physics_tick: 0,
        };
        plant.reset(initial_state)?;
        Ok(plant)
    }

    pub fn model_spec(&self) -> ApolloModelSpec {
        self.model_spec
    }

    pub fn timing(&self) -> SimulationTiming {
        self.timing
    }
}

/// 同步、外部 wrench 驱动的单个 Apollo MuJoCo 被控对象。
///
/// 本类型不持有控制器、目标、奖励、线程或 wall-clock 状态。调用者每次
/// 调用 [`step`](Self::step) 时，恰好推进一个配置好的控制周期。
pub struct ApolloPlant {
    data: MjData<Arc<MjModel>>,
    body_id: usize,
    timing: SimulationTiming,
    control_tick: u64,
    physics_tick: u64,
}

impl ApolloPlant {
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
            // MuJoCo freejoint 的四元数顺序为 w, x, y, z。
            qpos[3] = initial_state.body_to_world.w;
            qpos[4] = initial_state.body_to_world.x;
            qpos[5] = initial_state.body_to_world.y;
            qpos[6] = initial_state.body_to_world.z;
        }
        {
            let qvel = self.data.qvel_mut();
            // alignfree=false 时，freejoint 平移 qvel 是机体系原点的世界系速度。
            qvel[0] = initial_state.linear_velocity_body_origin_world_mps.x;
            qvel[1] = initial_state.linear_velocity_body_origin_world_mps.y;
            qvel[2] = initial_state.linear_velocity_body_origin_world_mps.z;
            // MuJoCo freejoint 的旋转 qvel 在机体系表达。
            qvel[3] = initial_state.angular_velocity_body_radps.x;
            qvel[4] = initial_state.angular_velocity_body_radps.y;
            qvel[5] = initial_state.angular_velocity_body_radps.z;
        }
        self.clear_applied_wrench();
        self.data.forward();
        self.control_tick = 0;
        self.physics_tick = 0;
        Ok(self.snapshot())
    }

    pub fn step(&mut self, action: BodyWrench) -> Result<PlantStep, PlantError> {
        action.validate().map_err(PlantError::InvalidAction)?;

        // 在修改 MuJoCo 状态前完成 tick 溢出检查，使错误调用保持无副作用。
        let next_control_tick = self
            .control_tick
            .checked_add(1)
            .ok_or(PlantError::TickOverflow)?;
        let substeps = self.timing.substeps_per_control.get();
        let next_physics_tick = self
            .physics_tick
            .checked_add(u64::from(substeps))
            .ok_or(PlantError::TickOverflow)?;

        // 零阶保持的是机体系 wrench。刚体在控制周期内转动时，每个物理子步
        // 都重新把同一机体系动作旋转到世界系，保持坐标语义正确。
        for _ in 0..substeps {
            self.apply_body_wrench(action);
            self.data.step();
        }
        // 动作只在本次 step 的控制周期内有效。
        self.clear_applied_wrench();

        self.control_tick = next_control_tick;
        self.physics_tick = next_physics_tick;
        let snapshot = self.snapshot();
        snapshot
            .state
            .validate()
            .map_err(PlantError::InvalidSimulationState)?;

        Ok(PlantStep {
            snapshot,
            requested_action: action,
            applied_action: action,
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

    fn apply_body_wrench(&mut self, action: BodyWrench) {
        self.clear_applied_wrench();
        let body_to_world = self.read_state().body_to_world;
        let force_world = body_to_world * action.force_body_n;
        let torque_world = body_to_world * action.torque_about_com_body_nm;
        self.data.xfrc_applied_mut()[self.body_id] = [
            force_world.x,
            force_world.y,
            force_world.z,
            torque_world.x,
            torque_world.y,
            torque_world.z,
        ];
    }

    fn clear_applied_wrench(&mut self) {
        self.data.xfrc_applied_mut().fill([0.0; 6]);
    }
}

impl PlantContract for ApolloPlant {
    type Error = PlantError;

    fn timing(&self) -> SimulationTiming {
        ApolloPlant::timing(self)
    }

    fn reset(&mut self, initial_state: ApolloState) -> Result<PlantSnapshot, Self::Error> {
        ApolloPlant::reset(self, initial_state)
    }

    fn snapshot(&self) -> PlantSnapshot {
        ApolloPlant::snapshot(self)
    }

    fn step(&mut self, action: BodyWrench) -> Result<PlantStep, Self::Error> {
        ApolloPlant::step(self, action)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::EulerRot;

    fn factory() -> ApolloPlantFactory {
        ApolloPlantFactory::apollo_touchdown().expect("Apollo MuJoCo model should load")
    }

    fn challenge_state() -> ApolloState {
        ApolloState {
            position_body_origin_world_m: DVec3::new(1.0, -2.0, 0.5),
            body_to_world: DQuat::from_euler(EulerRot::XYZ, -0.7, 0.4, 1.1),
            linear_velocity_body_origin_world_mps: DVec3::new(-0.2, 0.3, 0.4),
            angular_velocity_body_radps: DVec3::new(0.6, -0.35, 0.15),
        }
    }

    fn assert_state_close(actual: ApolloState, expected: ApolloState, tolerance: f64) {
        assert!(
            actual
                .position_body_origin_world_m
                .distance(expected.position_body_origin_world_m)
                <= tolerance,
            "position differs: {actual:?} vs {expected:?}"
        );
        assert!(
            actual.body_to_world.dot(expected.body_to_world).abs() >= 1.0 - tolerance,
            "attitude differs: {actual:?} vs {expected:?}"
        );
        assert!(
            actual
                .linear_velocity_body_origin_world_mps
                .distance(expected.linear_velocity_body_origin_world_mps)
                <= tolerance,
            "linear velocity differs: {actual:?} vs {expected:?}"
        );
        assert!(
            actual
                .angular_velocity_body_radps
                .distance(expected.angular_velocity_body_radps)
                <= tolerance,
            "angular velocity differs: {actual:?} vs {expected:?}"
        );
    }

    #[test]
    fn factory_uses_requested_timing_and_touchdown_mass_properties() {
        let factory = factory();
        assert_eq!(factory.timing(), SimulationTiming::APOLLO);
        assert!((factory.model.opt().timestep - 0.002).abs() < f64::EPSILON);

        let mass = factory.model.body_mass()[factory.body_id];
        let inertia = factory.model.body_inertia()[factory.body_id];
        let com = factory.model.body_ipos()[factory.body_id];
        let spec = ApolloModelSpec::touchdown();
        assert!((mass - spec.mass_kg).abs() < 1.0e-9);
        assert_eq!(inertia, spec.diagonal_inertia_body_kg_m2.to_array());
        assert!(DVec3::from_array(com).distance(spec.center_of_mass_body_m) < 1.0e-9);
    }

    #[test]
    fn freejoint_state_uses_body_origin_with_nonzero_com_and_spin() {
        let initial = challenge_state();
        assert_ne!(initial.angular_velocity_body_radps, DVec3::ZERO);

        let plant = factory().spawn(initial).unwrap();
        let body_origin_world = DVec3::from_array(plant.data.xpos()[plant.body_id]);
        let com_world = DVec3::from_array(plant.data.xipos()[plant.body_id]);
        let compiled_com_body = DVec3::from_array(plant.data.model().body_ipos()[plant.body_id]);
        let expected_com_world =
            initial.position_body_origin_world_m + initial.body_to_world * compiled_com_body;

        // alignfree=false 后，freejoint qpos 与 xpos 明确指向普通 body frame
        // 的原点；非零 body_ipos 则使惯性系原点（质心）位于另一位置。
        assert!(body_origin_world.distance(initial.position_body_origin_world_m) < 1.0e-12);
        assert!(com_world.distance(expected_com_world) < 1.0e-9);
        assert!(com_world.distance(body_origin_world) > 1.0);

        // MuJoCo freejoint 的平移 qvel 在世界系表达且对应 body origin；
        // 旋转 qvel 在局部机体系表达。非零角速度确保本测试不会把两者混淆。
        let qvel = plant.data.qvel();
        assert_eq!(
            DVec3::new(qvel[0], qvel[1], qvel[2]),
            initial.linear_velocity_body_origin_world_mps
        );
        assert_eq!(
            DVec3::new(qvel[3], qvel[4], qvel[5]),
            initial.angular_velocity_body_radps
        );
    }

    #[test]
    fn spawn_and_reset_preserve_freejoint_frame_conventions() {
        let expected = challenge_state();
        let mut plant = factory().spawn(expected).expect("plant should spawn");
        assert_state_close(plant.snapshot().state, expected, 1.0e-12);

        plant.step(BodyWrench::ZERO).unwrap();
        let reset = plant.reset(expected).unwrap();
        assert_eq!(reset.control_tick, 0);
        assert_eq!(reset.physics_tick, 0);
        assert_state_close(reset.state, expected, 1.0e-12);
    }

    #[test]
    fn one_step_advances_exactly_one_control_period() {
        let mut plant = factory().spawn(ApolloState::ZERO).unwrap();
        let result = plant.step(BodyWrench::ZERO).unwrap();

        assert_eq!(result.snapshot.control_tick, 1);
        assert_eq!(result.snapshot.physics_tick, 10);
        assert_eq!(result.requested_action, BodyWrench::ZERO);
        assert_eq!(result.applied_action, BodyWrench::ZERO);
        assert!((result.snapshot.sim_time_seconds(plant.timing()) - 0.020).abs() < f64::EPSILON);
    }

    #[test]
    fn force_and_torque_change_freejoint_state() {
        let mut plant = factory().spawn(ApolloState::ZERO).unwrap();
        let action = BodyWrench {
            force_body_n: DVec3::new(13_000.0, 0.0, 0.0),
            torque_about_com_body_nm: DVec3::new(0.0, 0.0, 4_000.0),
        };

        for _ in 0..60 {
            plant.step(action).unwrap();
        }
        let state = plant.snapshot().state;
        assert!(state.position_body_origin_world_m.length() > 0.001);
        assert!(state.body_to_world.dot(DQuat::IDENTITY).abs() < 0.99999);
        state.validate().unwrap();
    }

    #[test]
    fn body_frame_wrench_is_rotated_into_world_frame() {
        let initial = ApolloState {
            body_to_world: DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2),
            ..ApolloState::ZERO
        };
        let mut plant = factory().spawn(initial).unwrap();
        let result = plant
            .step(BodyWrench {
                force_body_n: DVec3::X * 10_000.0,
                torque_about_com_body_nm: DVec3::ZERO,
            })
            .unwrap();

        let velocity = result.snapshot.state.linear_velocity_body_origin_world_mps;
        assert!(velocity.y > 0.0);
        assert!(velocity.x.abs() < velocity.y * 1.0e-10);
        assert!(velocity.z.abs() < velocity.y * 1.0e-10);
    }

    #[test]
    fn zero_action_after_force_has_no_residual_wrench() {
        let mut plant = factory().spawn(ApolloState::ZERO).unwrap();
        let forced = plant
            .step(BodyWrench {
                force_body_n: DVec3::new(10_000.0, -2_000.0, 500.0),
                torque_about_com_body_nm: DVec3::ZERO,
            })
            .unwrap()
            .snapshot
            .state;
        let coasted = plant.step(BodyWrench::ZERO).unwrap().snapshot.state;

        assert!(
            coasted
                .linear_velocity_body_origin_world_mps
                .distance(forced.linear_velocity_body_origin_world_mps)
                < 1.0e-12
        );
    }

    #[test]
    fn invalid_inputs_are_rejected_without_advancing() {
        let factory = factory();
        let invalid_state = ApolloState {
            position_body_origin_world_m: DVec3::splat(f64::NAN),
            ..ApolloState::ZERO
        };
        assert!(matches!(
            factory.spawn(invalid_state),
            Err(PlantError::InvalidInitialState(_))
        ));

        let mut plant = factory.spawn(ApolloState::ZERO).unwrap();
        let before = plant.snapshot();
        let invalid_action = BodyWrench {
            torque_about_com_body_nm: DVec3::splat(f64::INFINITY),
            ..BodyWrench::ZERO
        };
        assert!(matches!(
            plant.step(invalid_action),
            Err(PlantError::InvalidAction(_))
        ));
        assert_eq!(plant.snapshot(), before);
    }

    #[test]
    fn identical_instances_are_deterministic() {
        let factory = factory();
        let mut left = factory.spawn(challenge_state()).unwrap();
        let mut right = factory.spawn(challenge_state()).unwrap();
        let actions = [
            BodyWrench::ZERO,
            BodyWrench {
                force_body_n: DVec3::new(120.0, -30.0, 15.0),
                torque_about_com_body_nm: DVec3::new(4.0, 8.0, -2.0),
            },
            BodyWrench {
                force_body_n: DVec3::new(-25.0, 60.0, 7.0),
                torque_about_com_body_nm: DVec3::new(1.0, -3.0, 5.0),
            },
        ];

        for index in 0..90 {
            let action = actions[index % actions.len()];
            assert_eq!(left.step(action).unwrap(), right.step(action).unwrap());
        }
    }

    #[test]
    fn factory_instances_do_not_share_mutable_state() {
        let factory = factory();
        let mut plants: Vec<_> = (0..32)
            .map(|index| {
                factory
                    .spawn(ApolloState {
                        position_body_origin_world_m: DVec3::new(index as f64, 0.0, 0.0),
                        ..ApolloState::ZERO
                    })
                    .unwrap()
            })
            .collect();
        let untouched: Vec<_> = plants.iter().map(ApolloPlant::snapshot).collect();

        plants[0]
            .step(BodyWrench {
                force_body_n: DVec3::Y * 10_000.0,
                torque_about_com_body_nm: DVec3::ZERO,
            })
            .unwrap();

        assert_ne!(plants[0].snapshot(), untouched[0]);
        for (plant, expected) in plants.iter().skip(1).zip(untouched.iter().skip(1)) {
            assert_eq!(plant.snapshot(), *expected);
        }
    }
}
