use crate::apollo_spec::{APOLLO_BODY_NAME, apollo_mjcf_xml};
use glam::{Quat, Vec3};
use mujoco_rs::prelude::*;
use std::sync::Arc;

#[derive(Clone, Copy, Debug, Default)]
pub struct ApolloWrench {
    pub force_body: Vec3,
    pub torque_body: Vec3,
}

#[derive(Clone, Copy, Debug)]
pub struct ApolloDynamicsState {
    /// 世界系位置。
    pub position: Vec3,
    /// 机体坐标系到世界坐标系的姿态旋转。
    pub rotation: Quat,
    /// 世界系线速度。
    pub linear_velocity: Vec3,
    /// 机体坐标系角速度。MuJoCo freejoint 的旋转 `qvel` 原生使用该表达。
    pub angular_velocity: Vec3,
}

pub struct ApolloDynamics {
    model: Arc<MjModel>,
    data: MjData<Arc<MjModel>>,
    body_id: usize,
}

impl ApolloDynamics {
    pub fn new() -> Result<Self, String> {
        let xml = apollo_mjcf_xml();
        let model = Arc::new(MjModel::from_xml_string(&xml).map_err(|err| err.to_string())?);
        let body_id = model
            .name_to_id(MjtObj::mjOBJ_BODY, APOLLO_BODY_NAME)
            .ok_or_else(|| format!("MuJoCo body '{APOLLO_BODY_NAME}' was not found"))?;
        let mut data = MjData::new(model.clone());
        data.forward();

        Ok(Self {
            model,
            data,
            body_id,
        })
    }

    pub fn model(&self) -> &MjModel {
        &self.model
    }

    pub fn simulation_dt_secs(&self) -> f32 {
        self.model.opt().timestep as f32
    }

    pub fn state(&self) -> ApolloDynamicsState {
        let qpos = self.data.qpos();
        let qvel = self.data.qvel();
        ApolloDynamicsState {
            position: Vec3::new(qpos[0] as f32, qpos[1] as f32, qpos[2] as f32),
            rotation: Quat::from_xyzw(
                qpos[4] as f32,
                qpos[5] as f32,
                qpos[6] as f32,
                qpos[3] as f32,
            )
            .normalize(),
            linear_velocity: Vec3::new(qvel[0] as f32, qvel[1] as f32, qvel[2] as f32),
            angular_velocity: Vec3::new(qvel[3] as f32, qvel[4] as f32, qvel[5] as f32),
        }
    }

    pub fn step(&mut self, wrench: ApolloWrench) -> ApolloDynamicsState {
        self.apply_body_wrench(wrench);
        self.data.step();
        self.state()
    }

    pub fn reset(&mut self) {
        self.data.reset();
        self.data.forward();
    }

    /// 重置到指定 freejoint 状态。
    ///
    /// MuJoCo 的 freejoint 将位置、姿态四元数和线速度存为世界系量，
    /// 但将旋转 `qvel` 存为机体局部坐标系量。
    pub fn reset_to_state(&mut self, state: ApolloDynamicsState) {
        self.data.reset();

        let rotation = state.rotation.normalize();
        let qpos = self.data.qpos_mut();
        qpos[0] = state.position.x as f64;
        qpos[1] = state.position.y as f64;
        qpos[2] = state.position.z as f64;
        qpos[3] = rotation.w as f64;
        qpos[4] = rotation.x as f64;
        qpos[5] = rotation.y as f64;
        qpos[6] = rotation.z as f64;

        let qvel = self.data.qvel_mut();
        qvel[0] = state.linear_velocity.x as f64;
        qvel[1] = state.linear_velocity.y as f64;
        qvel[2] = state.linear_velocity.z as f64;
        qvel[3] = state.angular_velocity.x as f64;
        qvel[4] = state.angular_velocity.y as f64;
        qvel[5] = state.angular_velocity.z as f64;

        self.data.forward();
    }

    fn apply_body_wrench(&mut self, wrench: ApolloWrench) {
        self.data.xfrc_applied_mut().fill([0.0; 6]);

        let rotation = self.state().rotation;
        let force_world = rotation * wrench.force_body;
        let torque_world = rotation * wrench.torque_body;
        self.data.xfrc_applied_mut()[self.body_id] = [
            force_world.x as f64,
            force_world.y as f64,
            force_world.z as f64,
            torque_world.x as f64,
            torque_world.y as f64,
            torque_world.z as f64,
        ];
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::apollo_spec::APOLLO_MUJOCO_TIMESTEP_SECS;

    fn is_finite_state(state: ApolloDynamicsState) -> bool {
        state.position.is_finite()
            && state.rotation.is_finite()
            && state.linear_velocity.is_finite()
            && state.angular_velocity.is_finite()
    }

    #[test]
    fn apollo_mujoco_model_uses_fixed_timestep() {
        let dynamics = ApolloDynamics::new().expect("Apollo MuJoCo model should load");

        assert!((dynamics.simulation_dt_secs() - APOLLO_MUJOCO_TIMESTEP_SECS as f32).abs() < 1e-6);
    }

    #[test]
    fn apollo_mujoco_model_steps_with_finite_state() {
        let mut dynamics = ApolloDynamics::new().expect("Apollo MuJoCo model should load");

        let mut state = dynamics.state();
        for _ in 0..120 {
            state = dynamics.step(ApolloWrench::default());
        }

        assert!(is_finite_state(state));
    }

    #[test]
    fn force_and_torque_change_freejoint_state() {
        let mut dynamics = ApolloDynamics::new().expect("Apollo MuJoCo model should load");
        let initial = dynamics.state();

        // Wrench scaled to match real-scale vehicle (~7,300 kg, I ~ 24,000 kg·m²).
        let wrench = ApolloWrench {
            force_body: Vec3::new(13_000.0, 0.0, 0.0),
            torque_body: Vec3::new(0.0, 0.0, 4_000.0),
        };

        let mut final_state = initial;
        for _ in 0..600 {
            final_state = dynamics.step(wrench);
        }

        assert!(final_state.position.distance(initial.position) > 0.001);
        assert!(final_state.rotation.dot(initial.rotation).abs() < 0.99999);
    }

    #[test]
    fn reset_to_state_preserves_freejoint_frame_conventions() {
        let mut dynamics = ApolloDynamics::new().expect("Apollo MuJoCo model should load");
        let expected = ApolloDynamicsState {
            position: Vec3::new(1.0, -2.0, 0.5),
            rotation: Quat::from_euler(glam::EulerRot::XYZ, -0.7, 0.4, 1.1),
            linear_velocity: Vec3::new(-0.2, 0.3, 0.4),
            angular_velocity: Vec3::new(0.6, -0.35, 0.15),
        };

        dynamics.reset_to_state(expected);
        let actual = dynamics.state();

        assert!(actual.position.distance(expected.position) < 1e-6);
        assert!(actual.rotation.dot(expected.rotation).abs() > 1.0 - 1e-6);
        assert!(actual.linear_velocity.distance(expected.linear_velocity) < 1e-6);
        assert!(actual.angular_velocity.distance(expected.angular_velocity) < 1e-6);
    }

    /// 诊断测试：打印 MuJoCo 计算的质量、转动惯量和质心位置。
    /// `cargo test dump_mass_properties -- --nocapture` 查看输出。
    #[test]
    fn dump_mass_properties() {
        let dynamics = ApolloDynamics::new().expect("Apollo MuJoCo model should load");
        let model = dynamics.model();
        let body_id = dynamics.body_id;

        let mass = model.body_mass()[body_id];
        let inertia = &model.body_inertia()[body_id];
        let ixx = inertia[0];
        let iyy = inertia[1];
        let izz = inertia[2];

        // MuJoCo ipos = inertial-frame position relative to body frame (i.e. CoM)
        let ipos = model.body_ipos()[body_id];
        let com_x = ipos[0];
        let com_y = ipos[1];
        let com_z = ipos[2];

        use crate::apollo_spec::{APOLLO_IXX, APOLLO_IYY, APOLLO_IZZ, center_of_mass, total_physics_mass};

        println!();
        println!("===== Apollo Lander Mass Properties (MuJoCo computed) =====");
        println!("Mass:  {:.1} kg  (from parts: {:.1})", mass, total_physics_mass());
        let com = center_of_mass();
        println!("CoM (body frame):  ({:.3}, {:.3}, {:.3})  (from parts: {com_x:.3}, {com_y:.3}, {com_z:.3})",
                 com_x, com_y, com_z, com_x=com.x, com_y=com.y, com_z=com.z);
        println!("Inertia (body diagonal, about CoM):");
        println!("  Ixx = {:10.1}  (Apollo 11 target: {})", ixx, APOLLO_IXX);
        println!("  Iyy = {:10.1}  (Apollo 11 target: {})", iyy, APOLLO_IYY);
        println!("  Izz = {:10.1}  (Apollo 11 target: {})", izz, APOLLO_IZZ);
        println!("============================================================");
        println!();
        println!("Mass comes from apollo_parts(). CoM is mass-weighted from parts.");
        println!("Inertia is set by <inertial diaginertia> using Apollo 11 constants.");
        println!();

        assert!(mass > 1000.0, "mass should be in the ton range");
        assert!(ixx > 1000.0 && iyy > 1000.0 && izz > 1000.0,
                "inertia should be in the thousands");
    }
}
