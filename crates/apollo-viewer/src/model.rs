use apollo_core::{
    ApolloMaterial, ApolloPropulsionSpec, ApolloShape, ApolloVisualPart, apollo_visual_parts,
};
use bevy::asset::RenderAssetUsages;
use bevy::math::primitives::{Cone, Cuboid, Cylinder, Sphere, Torus};
use bevy::mesh::Indices;
use bevy::prelude::*;
use bevy::render::render_resource::PrimitiveTopology;
use std::f32::consts::TAU;

/// RCS 喷口到直线羽流网格尖端的局部偏移。
pub const RCS_PLUME_EXIT_OFFSET_M: f32 = 0.11;
/// 未受导流板影响的诊断羽流长度。
pub const RCS_FREE_PLUME_LENGTH_M: f32 = 0.75;
/// RCS 诊断羽流锥在全强度时的底面半径。
pub const RCS_PLUME_RADIUS_M: f32 = 0.26;
/// Apollo 11 向下喷口直线段：到导流板交汇处即停止，不能继续穿过下降级。
pub const RCS_DEFLECTOR_INTERCEPT_LENGTH_M: f32 = 0.40;
/// 导流板外侧用于说明排气转向的短段长度。
pub const RCS_DEFLECTED_PLUME_LENGTH_M: f32 = 0.58;
/// coal-chute 外向段比自由羽流更窄，只表达受板约束的近场流束。
pub const RCS_DEFLECTED_PLUME_RADIUS_SCALE: f32 = 0.36;
const RCS_DEFLECTOR_CENTER_DOWNSTREAM_M: f32 = 0.53;
const RCS_DEFLECTOR_INBOARD_OFFSET_M: f32 = 0.18;
const RCS_DEFLECTOR_HEIGHT_M: f32 = 0.82;
const RCS_DEFLECTOR_BOW_M: f32 = 0.18;

pub struct LanderMaterials {
    gold: Handle<StandardMaterial>,
    foil: Handle<StandardMaterial>,
    metal: Handle<StandardMaterial>,
    dark: Handle<StandardMaterial>,
    white: Handle<StandardMaterial>,
    rcs_housing: Handle<StandardMaterial>,
    rcs_bell: Handle<StandardMaterial>,
    engine_bell: Handle<StandardMaterial>,
    engine_inner: Handle<StandardMaterial>,
    deflector: Handle<StandardMaterial>,
    hot_plume: Handle<StandardMaterial>,
}

/// RCS 喷管实体；索引与 `apollo-core` 推进规格中的稳定顺序一致。
#[derive(Component, Clone, Copy, Debug, Eq, PartialEq)]
pub struct RcsThrusterVisual {
    pub thruster_index: usize,
}

/// 只有对应的实际执行器状态非零时才显示的 RCS 羽流。
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct RcsPlumeVisual {
    pub thruster_index: usize,
    pub path: RcsPlumePath,
    pub maximum_length_m: f32,
}

/// 直线 RCS 羽流的几何语义。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RcsPlumePath {
    /// 12 个朝上或朝四周排气的喷口，直线段始终远离简化机体。
    Free,
    /// 4 个 D 后缀喷口，直线段只画到 Apollo 11 导流板。
    DeflectorIntercept,
}

/// 导流板后的外向短羽流几何诊断；它不增加第二个力，也不建模导流板推力损失。
/// 位置和方向都在机体系中，显示强度仍读取实际执行结果。
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct RcsDeflectedPlumeVisual {
    pub thruster_index: usize,
    pub source_body_m: Vec3,
    pub plume_direction_body: Vec3,
    pub maximum_length_m: f32,
}

/// 每个喷口自己的正交安装板。它随喷口根实体旋转，局部法向固定为 `+Y`。
#[derive(Component, Clone, Copy, Debug, Eq, PartialEq)]
pub struct RcsMountFlangeVisual {
    pub thruster_index: usize,
}

/// 下降发动机实际点火时显示的羽流。
#[derive(Component, Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct DpsPlumeVisual;

/// DPS 喉部/万向环根实体；交互 demo 按实际摆角旋转整个发动机。
#[derive(Component, Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct DpsEngineVisual;

pub fn create_lander_materials(materials: &mut Assets<StandardMaterial>) -> LanderMaterials {
    LanderMaterials {
        gold: materials.add(StandardMaterial {
            base_color: Color::srgb(0.92, 0.68, 0.22),
            metallic: 0.65,
            perceptual_roughness: 0.34,
            ..default()
        }),
        foil: materials.add(StandardMaterial {
            base_color: Color::srgb(1.0, 0.78, 0.28),
            metallic: 0.9,
            perceptual_roughness: 0.18,
            ..default()
        }),
        metal: materials.add(StandardMaterial {
            base_color: Color::srgb(0.64, 0.67, 0.70),
            metallic: 0.85,
            perceptual_roughness: 0.28,
            ..default()
        }),
        dark: materials.add(StandardMaterial {
            base_color: Color::srgb(0.035, 0.038, 0.045),
            metallic: 0.25,
            perceptual_roughness: 0.45,
            ..default()
        }),
        white: materials.add(StandardMaterial {
            base_color: Color::srgb(0.86, 0.88, 0.84),
            metallic: 0.15,
            perceptual_roughness: 0.55,
            ..default()
        }),
        rcs_housing: materials.add(StandardMaterial {
            base_color: Color::srgb(0.42, 0.44, 0.43),
            metallic: 0.88,
            perceptual_roughness: 0.32,
            ..default()
        }),
        rcs_bell: materials.add(StandardMaterial {
            base_color: Color::srgb(0.22, 0.23, 0.22),
            metallic: 0.92,
            perceptual_roughness: 0.28,
            ..default()
        }),
        engine_bell: materials.add(StandardMaterial {
            base_color: Color::srgb(0.25, 0.27, 0.26),
            metallic: 0.94,
            perceptual_roughness: 0.34,
            ..default()
        }),
        engine_inner: materials.add(StandardMaterial {
            base_color: Color::srgb(0.075, 0.052, 0.040),
            metallic: 0.50,
            perceptual_roughness: 0.72,
            double_sided: true,
            cull_mode: None,
            ..default()
        }),
        deflector: materials.add(StandardMaterial {
            base_color: Color::srgb(0.055, 0.058, 0.052),
            metallic: 0.52,
            perceptual_roughness: 0.76,
            double_sided: true,
            cull_mode: None,
            ..default()
        }),
        hot_plume: materials.add(StandardMaterial {
            base_color: Color::srgba(0.50, 0.72, 1.0, 0.34),
            emissive: LinearRgba::rgb(7.0, 10.0, 18.0),
            emissive_exposure_weight: 0.0,
            alpha_mode: AlphaMode::Add,
            unlit: true,
            double_sided: true,
            cull_mode: None,
            ..default()
        }),
    }
}

pub fn spawn_lander(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &LanderMaterials,
    transform: Transform,
) -> Entity {
    let lander = commands.spawn((transform, Visibility::default())).id();
    let propulsion_spec = ApolloPropulsionSpec::apollo11_touchdown();
    let propulsion_meshes = PropulsionMeshes::create(meshes);

    commands.entity(lander).with_children(|parent| {
        for part in apollo_visual_parts() {
            spawn_apollo_part(parent, meshes, materials, part);
        }
        spawn_propulsion_system(parent, &propulsion_meshes, materials, propulsion_spec);
    });

    lander
}

fn spawn_propulsion_system(
    parent: &mut ChildSpawnerCommands,
    meshes: &PropulsionMeshes,
    materials: &LanderMaterials,
    spec: ApolloPropulsionSpec,
) {
    for (index, thruster) in spec.rcs_thrusters.iter().enumerate() {
        debug_assert_eq!(thruster.id.index(), index);
        spawn_rcs_nozzle(
            parent,
            meshes,
            materials,
            dvec3(thruster.position_body_m),
            dvec3(thruster.force_direction_body),
            index,
        );
    }

    // Core 规格保证稳定顺序中每四台构成一个 quad。壳体中心取四个独立
    // Data Book 作用点的平均值；D 后缀喷口的受力方向为 +Y、羽流向下。
    for quad in spec.rcs_thrusters.chunks_exact(4) {
        let center = quad
            .iter()
            .map(|thruster| thruster.position_body_m)
            .sum::<glam::DVec3>()
            / quad.len() as f64;
        let downward_firing = quad
            .iter()
            .max_by(|left, right| {
                left.force_direction_body
                    .y
                    .total_cmp(&right.force_direction_body.y)
            })
            .expect("validated RCS quad contains four thrusters");
        let center_body_m = dvec3(center);
        let down_thruster_body_m = dvec3(downward_firing.position_body_m);
        let radial = Vec3::new(center_body_m.x, 0.0, center_body_m.z).normalize();
        spawn_rcs_quad_structure(
            parent,
            meshes,
            materials,
            center_body_m,
            down_thruster_body_m,
        );
        spawn_rcs_deflected_plume(
            parent,
            meshes,
            materials,
            down_thruster_body_m,
            radial,
            downward_firing.id.index(),
        );
    }

    spawn_dps_engine(
        parent,
        meshes,
        materials,
        dvec3(spec.dps.gimbal_pivot_body_m),
        dvec3(spec.dps.nominal_force_direction_body),
    );
}

#[derive(Clone)]
struct PropulsionMeshes {
    rcs_chamber: Handle<Mesh>,
    rcs_mount_pad: Handle<Mesh>,
    rcs_mount_flange: Handle<Mesh>,
    rcs_bell_outer: Handle<Mesh>,
    rcs_bell_inner: Handle<Mesh>,
    rcs_exit_ring: Handle<Mesh>,
    rcs_plume: Handle<Mesh>,
    quad_housing: Handle<Mesh>,
    deflector: Handle<Mesh>,
    detail_strut: Handle<Mesh>,
    dps_chamber: Handle<Mesh>,
    dps_bell_outer: Handle<Mesh>,
    dps_bell_inner: Handle<Mesh>,
    dps_exit_ring: Handle<Mesh>,
    dps_mount_ring: Handle<Mesh>,
    dps_heatshield_ring: Handle<Mesh>,
    dps_throat_shadow: Handle<Mesh>,
    dps_plume: Handle<Mesh>,
}

impl PropulsionMeshes {
    fn create(meshes: &mut Assets<Mesh>) -> Self {
        Self {
            rcs_chamber: meshes.add(Cylinder::new(0.042, 0.14).mesh().resolution(18)),
            // 安装板的薄轴是局部 Y；圆法兰也以局部 Y 为轴，因此两者表面
            // 都严格垂直于喷管/羽流轴，而不是借用 quad 壳体的斜面。
            rcs_mount_pad: meshes.add(Cuboid::new(0.20, 0.026, 0.20)),
            rcs_mount_flange: meshes.add(Cylinder::new(0.086, 0.036).mesh().resolution(24)),
            rcs_bell_outer: meshes.add(frustum_side_mesh(0.036, 0.080, 0.17, 24, false)),
            rcs_bell_inner: meshes.add(frustum_side_mesh(0.029, 0.069, 0.15, 24, true)),
            rcs_exit_ring: meshes.add(Torus::new(0.067, 0.083).mesh().major_resolution(24)),
            rcs_plume: meshes.add(
                Cone::new(RCS_PLUME_RADIUS_M, RCS_FREE_PLUME_LENGTH_M)
                    .mesh()
                    .resolution(24),
            ),
            quad_housing: meshes.add(Cuboid::new(0.30, 0.34, 0.30)),
            deflector: meshes.add(curved_deflector_mesh(
                0.68,
                RCS_DEFLECTOR_HEIGHT_M,
                RCS_DEFLECTOR_BOW_M,
            )),
            detail_strut: meshes.add(Cylinder::new(0.026, 1.0).mesh().resolution(12)),
            dps_chamber: meshes.add(Cylinder::new(0.30, 0.40).mesh().resolution(32)),
            dps_bell_outer: meshes.add(frustum_side_mesh(0.29, 0.80, 1.75, 48, false)),
            dps_bell_inner: meshes.add(frustum_side_mesh(0.235, 0.73, 1.66, 48, true)),
            dps_exit_ring: meshes.add(Torus::new(0.73, 0.82).mesh().major_resolution(48)),
            dps_mount_ring: meshes.add(Torus::new(0.64, 0.79).mesh().major_resolution(40)),
            dps_heatshield_ring: meshes.add(Torus::new(0.48, 0.61).mesh().major_resolution(40)),
            dps_throat_shadow: meshes.add(Cylinder::new(0.18, 0.045).mesh().resolution(36)),
            dps_plume: meshes.add(Cone::new(0.78, 2.4).mesh().resolution(40)),
        }
    }
}

fn spawn_rcs_nozzle(
    parent: &mut ChildSpawnerCommands,
    meshes: &PropulsionMeshes,
    materials: &LanderMaterials,
    position_body_m: Vec3,
    force_direction_body: Vec3,
    thruster_index: usize,
) {
    let force_direction_body = force_direction_body.normalize();
    let plume_path = if force_direction_body.dot(Vec3::Y) > 0.999 {
        RcsPlumePath::DeflectorIntercept
    } else {
        RcsPlumePath::Free
    };
    let maximum_plume_length_m = match plume_path {
        RcsPlumePath::Free => RCS_FREE_PLUME_LENGTH_M,
        RcsPlumePath::DeflectorIntercept => RCS_DEFLECTOR_INTERCEPT_LENGTH_M,
    };
    let plume_geometry_scale = maximum_plume_length_m / RCS_FREE_PLUME_LENGTH_M;
    parent
        .spawn((
            Transform::from_translation(position_body_m)
                .with_rotation(Quat::from_rotation_arc(Vec3::Y, force_direction_body)),
            Visibility::Inherited,
            RcsThrusterVisual { thruster_index },
        ))
        .with_children(|thruster| {
            thruster.spawn((
                Mesh3d(meshes.rcs_mount_pad.clone()),
                MeshMaterial3d(materials.metal.clone()),
                Transform::from_xyz(0.0, 0.19, 0.0),
                RcsMountFlangeVisual { thruster_index },
            ));
            thruster.spawn((
                Mesh3d(meshes.rcs_mount_flange.clone()),
                MeshMaterial3d(materials.rcs_housing.clone()),
                Transform::from_xyz(0.0, 0.16, 0.0),
            ));
            thruster.spawn((
                Mesh3d(meshes.rcs_chamber.clone()),
                MeshMaterial3d(materials.rcs_housing.clone()),
                Transform::from_xyz(0.0, 0.10, 0.0),
            ));
            thruster.spawn((
                Mesh3d(meshes.rcs_bell_outer.clone()),
                MeshMaterial3d(materials.rcs_bell.clone()),
                Transform::from_xyz(0.0, -0.015, 0.0),
            ));
            thruster.spawn((
                Mesh3d(meshes.rcs_bell_inner.clone()),
                MeshMaterial3d(materials.engine_inner.clone()),
                Transform::from_xyz(0.0, -0.025, 0.0),
            ));
            thruster.spawn((
                Mesh3d(meshes.rcs_exit_ring.clone()),
                MeshMaterial3d(materials.rcs_bell.clone()),
                Transform::from_xyz(0.0, -0.11, 0.0),
            ));
            // Cone 的尖端位于局部 +Y，正好贴住喷口；排气沿受力反方向延伸。
            thruster.spawn((
                Mesh3d(meshes.rcs_plume.clone()),
                MeshMaterial3d(materials.hot_plume.clone()),
                Transform::from_xyz(
                    0.0,
                    -RCS_PLUME_EXIT_OFFSET_M - maximum_plume_length_m * 0.5,
                    0.0,
                )
                // D 喷口截短时三轴同比缩放，保持母锥 0.26/0.75 的半锥角；
                // 只缩 Y 会让锥体虚假变宽并提前穿入导流板。
                .with_scale(match plume_path {
                    RcsPlumePath::Free => Vec3::ONE,
                    RcsPlumePath::DeflectorIntercept => Vec3::splat(plume_geometry_scale),
                }),
                Visibility::Hidden,
                RcsPlumeVisual {
                    thruster_index,
                    path: plume_path,
                    maximum_length_m: maximum_plume_length_m,
                },
            ));
        });
}

fn spawn_rcs_quad_structure(
    parent: &mut ChildSpawnerCommands,
    meshes: &PropulsionMeshes,
    materials: &LanderMaterials,
    center_body_m: Vec3,
    down_thruster_body_m: Vec3,
) {
    let radial = Vec3::new(center_body_m.x, 0.0, center_body_m.z).normalize();
    let tangent = Vec3::Y.cross(radial).normalize();
    let housing_rotation = Quat::from_rotation_arc(Vec3::Z, radial);

    parent.spawn((
        Mesh3d(meshes.quad_housing.clone()),
        MeshMaterial3d(materials.rcs_housing.clone()),
        Transform::from_translation(center_body_m).with_rotation(housing_rotation),
    ));

    // 两根外伸桁架把四联装与上升级连接；端点由共享喷管规格反算，
    // 不再复制旧模型 landing-gear 的 2.04 m 半径。
    for side in [-1.0_f32, 1.0] {
        let anchor =
            radial * 1.30 + Vec3::Y * (center_body_m.y + side * 0.13) + tangent * side * 0.11;
        let housing = center_body_m - radial * 0.18 + tangent * side * 0.13;
        spawn_unit_cylinder_between(
            parent,
            meshes.detail_strut.clone(),
            materials.metal.clone(),
            anchor,
            housing,
        );
    }

    let deflector_center = rcs_deflector_center(down_thruster_body_m, radial);
    parent.spawn((
        Mesh3d(meshes.deflector.clone()),
        MeshMaterial3d(materials.deflector.clone()),
        Transform::from_translation(deflector_center).with_rotation(housing_rotation),
    ));

    // 导流板属于下降级；支撑桁架从下降级肩部伸到板后，而不是挂在喷管上。
    for side in [-1.0_f32, 1.0] {
        let stage_anchor = radial * 1.48 + Vec3::Y * 2.14 + tangent * side * 0.18;
        let plate_anchor =
            deflector_center + Vec3::Y * 0.31 - radial * 0.035 + tangent * side * 0.25;
        spawn_unit_cylinder_between(
            parent,
            meshes.detail_strut.clone(),
            materials.metal.clone(),
            stage_anchor,
            plate_anchor,
        );
    }
}

fn rcs_deflector_center(down_thruster_body_m: Vec3, radial: Vec3) -> Vec3 {
    // 板必须位于喷流轴与下降级之间（径向内侧）；曲面随后朝外弯，形成
    // Apollo 11 的 coal-chute 通道。放到轴外侧会失去遮挡机体的作用。
    down_thruster_body_m
        - Vec3::Y * RCS_DEFLECTOR_CENTER_DOWNSTREAM_M
        - radial * RCS_DEFLECTOR_INBOARD_OFFSET_M
}

fn rcs_deflected_plume_geometry(down_thruster_body_m: Vec3, radial: Vec3) -> (Vec3, Vec3) {
    // 直线锥的内侧底缘在导流板上；转向段从撞击位置沿板面向外离开。
    let source = down_thruster_body_m
        - Vec3::Y * (RCS_PLUME_EXIT_OFFSET_M + RCS_DEFLECTOR_INTERCEPT_LENGTH_M)
        - radial * 0.13;
    // 短段沿 coal-chute 水平离开下降级；远场下弯/扩散未建模，也不能用
    // 一个对称锥在尚未获得径向净空前向下扩张进下降级。
    let direction = radial;
    (source, direction)
}

fn spawn_rcs_deflected_plume(
    parent: &mut ChildSpawnerCommands,
    meshes: &PropulsionMeshes,
    materials: &LanderMaterials,
    down_thruster_body_m: Vec3,
    radial: Vec3,
    thruster_index: usize,
) {
    let (source, plume_direction) = rcs_deflected_plume_geometry(down_thruster_body_m, radial);
    let length = RCS_DEFLECTED_PLUME_LENGTH_M;
    parent.spawn((
        Mesh3d(meshes.rcs_plume.clone()),
        MeshMaterial3d(materials.hot_plume.clone()),
        Transform::from_translation(source + plume_direction * (length * 0.5))
            .with_rotation(Quat::from_rotation_arc(Vec3::Y, -plume_direction))
            .with_scale(Vec3::splat(
                length / RCS_FREE_PLUME_LENGTH_M * RCS_DEFLECTED_PLUME_RADIUS_SCALE,
            )),
        Visibility::Hidden,
        RcsDeflectedPlumeVisual {
            thruster_index,
            source_body_m: source,
            plume_direction_body: plume_direction,
            maximum_length_m: length,
        },
    ));
}

fn spawn_dps_engine(
    parent: &mut ChildSpawnerCommands,
    meshes: &PropulsionMeshes,
    materials: &LanderMaterials,
    throat_body_m: Vec3,
    force_direction_body: Vec3,
) {
    parent
        .spawn((
            Transform::from_translation(throat_body_m).with_rotation(Quat::from_rotation_arc(
                Vec3::Y,
                force_direction_body.normalize(),
            )),
            Visibility::Inherited,
            DpsEngineVisual,
        ))
        .with_children(|engine| {
            engine.spawn((
                Mesh3d(meshes.dps_mount_ring.clone()),
                MeshMaterial3d(materials.metal.clone()),
                Transform::from_xyz(0.0, 0.02, 0.0),
            ));
            engine.spawn((
                Mesh3d(meshes.dps_chamber.clone()),
                MeshMaterial3d(materials.rcs_housing.clone()),
                Transform::from_xyz(0.0, 0.21, 0.0),
            ));
            engine.spawn((
                Mesh3d(meshes.dps_bell_outer.clone()),
                MeshMaterial3d(materials.engine_bell.clone()),
                Transform::from_xyz(0.0, -0.875, 0.0),
            ));
            engine.spawn((
                Mesh3d(meshes.dps_bell_inner.clone()),
                MeshMaterial3d(materials.engine_inner.clone()),
                Transform::from_xyz(0.0, -0.92, 0.0),
            ));
            engine.spawn((
                Mesh3d(meshes.dps_exit_ring.clone()),
                MeshMaterial3d(materials.engine_bell.clone()),
                Transform::from_xyz(0.0, -1.75, 0.0),
            ));
            // 当前简化下降级是带封闭底盖的圆柱；在真实发动机穿出热盾的位置
            // 增加可见套环与喉部暗面，使底视图仍能辨认安装界面和内壁深度。
            engine.spawn((
                Mesh3d(meshes.dps_heatshield_ring.clone()),
                MeshMaterial3d(materials.metal.clone()),
                Transform::from_xyz(0.0, -1.04, 0.0),
            ));
            engine.spawn((
                Mesh3d(meshes.dps_throat_shadow.clone()),
                MeshMaterial3d(materials.engine_inner.clone()),
                Transform::from_xyz(0.0, -1.08, 0.0),
            ));

            // 八根短撑杆显式表现喉部平面的万向安装环。
            for index in 0..8 {
                let angle = index as f32 * TAU / 8.0;
                let radial = Vec3::new(angle.cos(), 0.0, angle.sin());
                spawn_unit_cylinder_between(
                    engine,
                    meshes.detail_strut.clone(),
                    materials.metal.clone(),
                    radial * 0.70 + Vec3::Y * 0.07,
                    radial * 0.29 + Vec3::Y * 0.32,
                );
            }

            engine.spawn((
                Mesh3d(meshes.dps_plume.clone()),
                MeshMaterial3d(materials.hot_plume.clone()),
                Transform::from_xyz(0.0, -2.95, 0.0),
                Visibility::Hidden,
                DpsPlumeVisual,
            ));
        });
}

fn spawn_apollo_part(
    parent: &mut ChildSpawnerCommands,
    meshes: &mut Assets<Mesh>,
    materials: &LanderMaterials,
    part: ApolloVisualPart,
) {
    let material = apollo_material(materials, part.material);
    let transform = part_transform(part);

    match part.shape {
        ApolloShape::Cuboid { size_m } => {
            parent.spawn((
                Mesh3d(meshes.add(Cuboid::from_size(dvec3(size_m)))),
                MeshMaterial3d(material),
                transform,
            ));
        }
        ApolloShape::Cylinder {
            radius_m,
            height_m,
            resolution,
        } => {
            parent.spawn((
                Mesh3d(
                    meshes.add(
                        Cylinder::new(radius_m as f32, height_m as f32)
                            .mesh()
                            .resolution(resolution),
                    ),
                ),
                MeshMaterial3d(material),
                transform,
            ));
        }
        ApolloShape::Sphere { radius_m } => {
            parent.spawn((
                Mesh3d(meshes.add(Sphere::new(radius_m as f32).mesh().uv(16, 8))),
                MeshMaterial3d(material),
                transform,
            ));
        }
        ApolloShape::Strut {
            start_body_m,
            end_body_m,
            radius_m,
            resolution,
        } => {
            let start = dvec3(start_body_m);
            let end = dvec3(end_body_m);
            spawn_cylinder_between(
                parent,
                meshes.add(
                    Cylinder::new(radius_m as f32, start.distance(end))
                        .mesh()
                        .resolution(resolution),
                ),
                material,
                start,
                end,
            );
        }
    }
}

fn part_transform(part: ApolloVisualPart) -> Transform {
    Transform::from_translation(dvec3(part.translation_body_m))
        .with_rotation(dquat(part.rotation_part_to_body))
        .with_scale(dvec3(part.scale))
}

fn apollo_material(
    materials: &LanderMaterials,
    material: ApolloMaterial,
) -> Handle<StandardMaterial> {
    match material {
        ApolloMaterial::Gold => materials.gold.clone(),
        ApolloMaterial::Foil => materials.foil.clone(),
        ApolloMaterial::Metal => materials.metal.clone(),
        ApolloMaterial::Dark => materials.dark.clone(),
        ApolloMaterial::White => materials.white.clone(),
    }
}

fn spawn_cylinder_between(
    parent: &mut ChildSpawnerCommands,
    mesh: Handle<Mesh>,
    material: Handle<StandardMaterial>,
    start: Vec3,
    end: Vec3,
) {
    let delta = end - start;
    let rotation = Quat::from_rotation_arc(Vec3::Y, delta.normalize());
    parent.spawn((
        Mesh3d(mesh),
        MeshMaterial3d(material),
        Transform::from_translation((start + end) * 0.5).with_rotation(rotation),
    ));
}

/// 将局部 Y 长度为 1 m 的圆柱缩放到两个端点之间。
fn spawn_unit_cylinder_between(
    parent: &mut ChildSpawnerCommands,
    mesh: Handle<Mesh>,
    material: Handle<StandardMaterial>,
    start: Vec3,
    end: Vec3,
) {
    let delta = end - start;
    let length = delta.length();
    if length <= f32::EPSILON {
        return;
    }
    parent.spawn((
        Mesh3d(mesh),
        MeshMaterial3d(material),
        Transform::from_translation((start + end) * 0.5)
            .with_rotation(Quat::from_rotation_arc(Vec3::Y, delta / length))
            .with_scale(Vec3::new(1.0, length, 1.0)),
    ));
}

pub fn dvec3(value: glam::DVec3) -> Vec3 {
    Vec3::new(value.x as f32, value.y as f32, value.z as f32)
}

pub fn dquat(value: glam::DQuat) -> Quat {
    Quat::from_xyzw(
        value.x as f32,
        value.y as f32,
        value.z as f32,
        value.w as f32,
    )
    .normalize()
}

/// 生成一张没有端盖的圆台侧壁。
///
/// 局部 `+Y` 指向喉部，`-Y` 指向喷口；`inward` 用于生成可从喷口内部
/// 看到的内壁。将内、外两张网格分开，避免普通封闭圆台的底盖挡住喉部。
fn frustum_side_mesh(
    throat_radius: f32,
    exit_radius: f32,
    height: f32,
    resolution: u32,
    inward: bool,
) -> Mesh {
    assert!(throat_radius > 0.0 && exit_radius > throat_radius);
    assert!(height > 0.0 && resolution >= 3);

    let resolution = resolution as usize;
    let radial_slope = (exit_radius - throat_radius) / height;
    let mut positions = Vec::with_capacity((resolution + 1) * 2);
    let mut normals = Vec::with_capacity((resolution + 1) * 2);
    let mut uvs = Vec::with_capacity((resolution + 1) * 2);

    for index in 0..=resolution {
        let fraction = index as f32 / resolution as f32;
        let angle = fraction * TAU;
        let (sin, cos) = angle.sin_cos();
        let mut normal = Vec3::new(cos, radial_slope, sin).normalize();
        if inward {
            normal = -normal;
        }
        positions.push([exit_radius * cos, -height * 0.5, exit_radius * sin]);
        positions.push([throat_radius * cos, height * 0.5, throat_radius * sin]);
        normals.push(normal.to_array());
        normals.push(normal.to_array());
        uvs.push([fraction, 1.0]);
        uvs.push([fraction, 0.0]);
    }

    let mut indices = Vec::with_capacity(resolution * 6);
    for index in 0..resolution {
        let bottom = (index * 2) as u32;
        let top = bottom + 1;
        let next_bottom = bottom + 2;
        let next_top = bottom + 3;
        if inward {
            indices.extend_from_slice(&[bottom, next_top, top, bottom, next_bottom, next_top]);
        } else {
            indices.extend_from_slice(&[bottom, top, next_top, bottom, next_top, next_bottom]);
        }
    }

    Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::MAIN_WORLD | RenderAssetUsages::RENDER_WORLD,
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
    .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, uvs)
    .with_inserted_indices(Indices::U32(indices))
}

/// Apollo 11 新增的 RCS 羽流导流板用一张轻微弯曲的双面板表达。
/// 真实硬件由薄层 Inconel/Nickel 叠层并通过桁架固定在下降级上。
fn curved_deflector_mesh(width: f32, height: f32, bow_depth: f32) -> Mesh {
    const ROWS: usize = 7;
    let mut positions = Vec::with_capacity(ROWS * 2);
    let mut normals = Vec::with_capacity(ROWS * 2);
    let mut uvs = Vec::with_capacity(ROWS * 2);
    for row in 0..ROWS {
        let fraction = row as f32 / (ROWS - 1) as f32;
        let y = -height * 0.5 + fraction * height;
        let z = bow_depth * (1.0 - fraction).powi(2);
        let normal = Vec3::new(0.0, 2.0 * bow_depth * (1.0 - fraction), height).normalize();
        for (side, x) in [-width * 0.5, width * 0.5].into_iter().enumerate() {
            positions.push([x, y, z]);
            normals.push(normal.to_array());
            uvs.push([side as f32, 1.0 - fraction]);
        }
    }

    let mut indices = Vec::with_capacity((ROWS - 1) * 6);
    for row in 0..ROWS - 1 {
        let lower_left = (row * 2) as u32;
        let lower_right = lower_left + 1;
        let upper_left = lower_left + 2;
        let upper_right = lower_left + 3;
        indices.extend_from_slice(&[
            lower_left,
            lower_right,
            upper_left,
            lower_right,
            upper_right,
            upper_left,
        ]);
    }

    Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::MAIN_WORLD | RenderAssetUsages::RENDER_WORLD,
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
    .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, uvs)
    .with_inserted_indices(Indices::U32(indices))
}

#[cfg(test)]
mod tests {
    use super::*;
    use apollo_core::{ApolloShape, ApolloState, ApolloVisualPart, apollo_visual_parts};

    #[test]
    fn f64_pose_conversion_preserves_identity_and_axes() {
        assert_eq!(
            dvec3(glam::DVec3::new(1.0, -2.0, 3.0)),
            Vec3::new(1.0, -2.0, 3.0)
        );
        assert!(dquat(ApolloState::ZERO.body_to_world).dot(Quat::IDENTITY) > 1.0 - 1.0e-6);
    }

    #[test]
    fn open_bell_mesh_keeps_inner_and_outer_surfaces_well_formed() {
        let outer = frustum_side_mesh(0.28, 0.94, 1.18, 32, false);
        let inner = frustum_side_mesh(0.24, 0.88, 1.12, 32, true);
        assert_eq!(outer.count_vertices(), 66);
        assert_eq!(inner.count_vertices(), 66);
        assert_eq!(outer.primitive_topology(), PrimitiveTopology::TriangleList);
    }

    #[test]
    fn apollo_11_deflector_mesh_has_curvature_and_two_columns() {
        let mesh = curved_deflector_mesh(0.66, 0.82, 0.18);
        assert_eq!(mesh.count_vertices(), 14);
        let Some(bevy::mesh::VertexAttributeValues::Float32x3(positions)) =
            mesh.attribute(Mesh::ATTRIBUTE_POSITION)
        else {
            panic!("deflector positions must be Float32x3");
        };
        assert!(positions.first().unwrap()[2] > positions.last().unwrap()[2]);
    }

    #[test]
    fn every_rcs_nozzle_axis_is_normal_to_its_independent_mounting_pad() {
        let spec = ApolloPropulsionSpec::apollo11_touchdown();
        for thruster in spec.rcs_thrusters {
            let force_direction = dvec3(thruster.force_direction_body).normalize();
            let nozzle_to_body = Quat::from_rotation_arc(Vec3::Y, force_direction);
            let pad_normal_body = nozzle_to_body * Vec3::Y;
            let plume_axis_body = -force_direction;
            assert!(
                pad_normal_body.dot(plume_axis_body).abs() > 1.0 - 1.0e-6,
                "{} nozzle axis is not normal to its mounting pad",
                thruster.label
            );

            // 安装板在喷管后方（受力方向），喷口/羽流则从板面沿反方向伸出。
            let pad_center = dvec3(thruster.position_body_m) + pad_normal_body * 0.19;
            let pad_to_nozzle = dvec3(thruster.position_body_m) - pad_center;
            assert!(pad_to_nozzle.normalize().dot(plume_axis_body) > 1.0 - 1.0e-6);
        }
    }

    #[test]
    fn horizontal_nozzles_use_body_cardinal_axes_not_quad_tangents() {
        let spec = ApolloPropulsionSpec::apollo11_touchdown();
        let mut checked = 0;
        for quad in spec.rcs_thrusters.chunks_exact(4) {
            let center = quad
                .iter()
                .map(|thruster| thruster.position_body_m)
                .sum::<glam::DVec3>()
                / quad.len() as f64;
            let radial = glam::DVec3::new(center.x, 0.0, center.z).normalize();
            let tangent = glam::DVec3::Y.cross(radial).normalize();
            for thruster in quad {
                let plume = -thruster.force_direction_body;
                if plume.y.abs() > 1.0e-12 {
                    continue;
                }
                checked += 1;
                let x_cardinal = (plume.x.abs() - 1.0).abs() < 1.0e-12 && plume.z.abs() < 1.0e-12;
                let z_cardinal = (plume.z.abs() - 1.0).abs() < 1.0e-12 && plume.x.abs() < 1.0e-12;
                assert!(
                    x_cardinal || z_cardinal,
                    "{} horizontal plume must lie on body +/-X or +/-Z",
                    thruster.label
                );
                assert!(
                    (plume.dot(radial).abs() - std::f64::consts::FRAC_1_SQRT_2).abs() < 1.0e-12
                );
                assert!(
                    (plume.dot(tangent).abs() - std::f64::consts::FRAC_1_SQRT_2).abs() < 1.0e-12
                );
            }
        }
        assert_eq!(checked, 8);
    }

    #[test]
    fn twelve_free_rcs_plume_rays_are_outward_and_clear_the_simplified_lander() {
        let spec = ApolloPropulsionSpec::apollo11_touchdown();
        let parts = apollo_visual_parts();
        let mut checked = 0;
        for thruster in spec.rcs_thrusters {
            let plume_direction = -thruster.force_direction_body;
            if plume_direction.dot(glam::DVec3::NEG_Y) > 0.999 {
                continue;
            }
            checked += 1;
            let horizontal =
                glam::DVec3::new(thruster.position_body_m.x, 0.0, thruster.position_body_m.z)
                    .normalize();
            assert!(
                plume_direction.y > 0.999 || plume_direction.dot(horizontal) > 0.0,
                "{} plume is not directed upward or radially outward",
                thruster.label
            );
            let start =
                thruster.position_body_m + plume_direction * f64::from(RCS_PLUME_EXIT_OFFSET_M);
            assert_segment_clear_of_visual_parts(
                start,
                plume_direction,
                3.0,
                &parts,
                thruster.label,
            );
            assert_conical_plume_clear_of_visual_parts(
                start,
                plume_direction,
                f64::from(RCS_FREE_PLUME_LENGTH_M),
                f64::from(RCS_PLUME_RADIUS_M),
                &parts,
                thruster.label,
            );
        }
        assert_eq!(checked, 12);
    }

    #[test]
    fn four_down_plumes_stop_at_deflectors_then_turn_outward_clear_of_body() {
        let spec = ApolloPropulsionSpec::apollo11_touchdown();
        let parts = apollo_visual_parts();
        let mut checked = 0;
        for quad in spec.rcs_thrusters.chunks_exact(4) {
            let center = quad
                .iter()
                .map(|thruster| thruster.position_body_m)
                .sum::<glam::DVec3>()
                / quad.len() as f64;
            let down = quad
                .iter()
                .find(|thruster| thruster.force_direction_body.dot(glam::DVec3::Y) > 0.999)
                .expect("each quad must have one downward plume jet");
            checked += 1;
            let radial = Vec3::new(center.x as f32, 0.0, center.z as f32).normalize();
            let down_position = dvec3(down.position_body_m);
            let straight_start = down_position - Vec3::Y * RCS_PLUME_EXIT_OFFSET_M;
            let straight_end = straight_start - Vec3::Y * RCS_DEFLECTOR_INTERCEPT_LENGTH_M;
            let deflector_center = rcs_deflector_center(down_position, radial);

            // 导流板位于轴线径向内侧，母锥三轴等比截短到 0.40 m；
            // 解析的分片板面首次交汇正好落在直线段末端，之后不再画直尾焰。
            assert!(
                deflector_center.dot(radial) < down_position.dot(radial),
                "{} deflector must sit inboard of the plume axis",
                down.label
            );
            let cutoff = RCS_DEFLECTOR_INTERCEPT_LENGTH_M;
            let cone_radius_at_cutoff = RCS_PLUME_RADIUS_M * cutoff / RCS_FREE_PLUME_LENGTH_M;
            let board_offset_at_cutoff =
                deflector_piecewise_radial_offset_at_distance(cutoff).abs();
            assert!((cone_radius_at_cutoff - board_offset_at_cutoff).abs() < 0.003);
            let before = cutoff - 0.01;
            assert!(
                RCS_PLUME_RADIUS_M * before / RCS_FREE_PLUME_LENGTH_M
                    < deflector_piecewise_radial_offset_at_distance(before).abs()
            );
            let after = cutoff + 0.01;
            assert!(
                RCS_PLUME_RADIUS_M * after / RCS_FREE_PLUME_LENGTH_M
                    > deflector_piecewise_radial_offset_at_distance(after).abs()
            );
            assert_segment_clear_of_visual_parts(
                straight_start.as_dvec3(),
                -glam::DVec3::Y,
                f64::from(RCS_DEFLECTOR_INTERCEPT_LENGTH_M),
                &parts,
                down.label,
            );
            assert_conical_plume_clear_of_visual_parts(
                straight_start.as_dvec3(),
                -glam::DVec3::Y,
                f64::from(RCS_DEFLECTOR_INTERCEPT_LENGTH_M),
                f64::from(cone_radius_at_cutoff),
                &parts,
                down.label,
            );

            let (deflected_source, deflected_direction) =
                rcs_deflected_plume_geometry(down_position, radial);
            assert!((deflected_source - straight_end).dot(radial) < -0.12);
            assert!(deflected_direction.dot(radial) > 0.99);
            assert!(deflected_direction.y.abs() < 1.0e-6);
            assert_segment_clear_of_visual_parts(
                deflected_source.as_dvec3(),
                deflected_direction.as_dvec3(),
                f64::from(RCS_DEFLECTED_PLUME_LENGTH_M),
                &parts,
                down.label,
            );
            assert_conical_plume_clear_of_visual_parts(
                deflected_source.as_dvec3(),
                deflected_direction.as_dvec3(),
                f64::from(RCS_DEFLECTED_PLUME_LENGTH_M),
                f64::from(
                    RCS_PLUME_RADIUS_M * RCS_DEFLECTED_PLUME_LENGTH_M / RCS_FREE_PLUME_LENGTH_M
                        * RCS_DEFLECTED_PLUME_RADIUS_SCALE,
                ),
                &parts,
                down.label,
            );
        }
        assert_eq!(checked, 4);
    }

    #[test]
    fn lander_spawns_all_shared_spec_thrusters_and_applied_plume_markers() {
        fn setup(
            mut commands: Commands,
            mut meshes: ResMut<Assets<Mesh>>,
            mut materials: ResMut<Assets<StandardMaterial>>,
        ) {
            let materials = create_lander_materials(&mut materials);
            spawn_lander(&mut commands, &mut meshes, &materials, Transform::IDENTITY);
        }

        let mut app = App::new();
        app.init_resource::<Assets<Mesh>>()
            .init_resource::<Assets<StandardMaterial>>()
            .add_systems(Startup, setup);
        app.update();

        let mut thrusters = app
            .world_mut()
            .query::<&RcsThrusterVisual>()
            .iter(app.world())
            .map(|visual| visual.thruster_index)
            .collect::<Vec<_>>();
        thrusters.sort_unstable();
        assert_eq!(thrusters, (0..16).collect::<Vec<_>>());
        assert_eq!(
            app.world_mut()
                .query::<&RcsPlumeVisual>()
                .iter(app.world())
                .count(),
            16
        );
        assert_eq!(
            app.world_mut()
                .query::<&RcsMountFlangeVisual>()
                .iter(app.world())
                .count(),
            16
        );
        assert_eq!(
            app.world_mut()
                .query::<&RcsDeflectedPlumeVisual>()
                .iter(app.world())
                .count(),
            4
        );
        assert_eq!(
            app.world_mut()
                .query::<&DpsEngineVisual>()
                .iter(app.world())
                .count(),
            1
        );
        assert_eq!(
            app.world_mut()
                .query::<&DpsPlumeVisual>()
                .iter(app.world())
                .count(),
            1
        );
    }

    fn assert_segment_clear_of_visual_parts(
        start: glam::DVec3,
        direction: glam::DVec3,
        length_m: f64,
        parts: &[ApolloVisualPart],
        label: &str,
    ) {
        const SAMPLES: usize = 400;
        for sample in 0..=SAMPLES {
            let distance = length_m * sample as f64 / SAMPLES as f64;
            let point = start + direction * distance;
            if let Some(part) = parts
                .iter()
                .find(|part| point_inside_visual_part(point, **part))
            {
                panic!(
                    "{label} diagnostic plume intersects simplified part '{}' at {point:?}",
                    part.name
                );
            }
        }
    }

    fn assert_conical_plume_clear_of_visual_parts(
        tip_body_m: glam::DVec3,
        direction_body: glam::DVec3,
        length_m: f64,
        base_radius_m: f64,
        parts: &[ApolloVisualPart],
        label: &str,
    ) {
        const AXIAL_SAMPLES: usize = 160;
        const ANGULAR_SAMPLES: usize = 24;
        const RADIAL_FRACTIONS: [f64; 5] = [0.0, 0.25, 0.5, 0.75, 1.0];
        let direction = direction_body.normalize();
        let reference = if direction.dot(glam::DVec3::Y).abs() < 0.9 {
            glam::DVec3::Y
        } else {
            glam::DVec3::X
        };
        let first_normal = direction.cross(reference).normalize();
        let second_normal = direction.cross(first_normal).normalize();

        for axial_sample in 0..=AXIAL_SAMPLES {
            let axial_fraction = axial_sample as f64 / AXIAL_SAMPLES as f64;
            let center = tip_body_m + direction * (length_m * axial_fraction);
            let section_radius = base_radius_m * axial_fraction;
            for radial_fraction in RADIAL_FRACTIONS {
                if radial_fraction == 0.0 {
                    assert_point_clear(center, parts, label);
                    continue;
                }
                for angular_sample in 0..ANGULAR_SAMPLES {
                    let angle =
                        angular_sample as f64 * std::f64::consts::TAU / ANGULAR_SAMPLES as f64;
                    let offset = (first_normal * angle.cos() + second_normal * angle.sin())
                        * (section_radius * radial_fraction);
                    assert_point_clear(center + offset, parts, label);
                }
            }
        }
    }

    fn assert_point_clear(point: glam::DVec3, parts: &[ApolloVisualPart], label: &str) {
        if let Some(part) = parts
            .iter()
            .find(|part| point_inside_visual_part(point, **part))
        {
            panic!(
                "{label} diagnostic plume volume intersects simplified part '{}' at {point:?}",
                part.name
            );
        }
    }

    fn deflector_piecewise_radial_offset_at_distance(distance_from_exit_m: f32) -> f32 {
        const ROWS: usize = 7;
        let top_distance = RCS_DEFLECTOR_CENTER_DOWNSTREAM_M
            - RCS_DEFLECTOR_HEIGHT_M * 0.5
            - RCS_PLUME_EXIT_OFFSET_M;
        let fraction_from_top =
            ((distance_from_exit_m - top_distance) / RCS_DEFLECTOR_HEIGHT_M).clamp(0.0, 1.0);
        let row_position = fraction_from_top * (ROWS - 1) as f32;
        let lower = row_position.floor() as usize;
        let upper = (lower + 1).min(ROWS - 1);
        let mix = row_position - lower as f32;
        let offset_at_row = |row_from_top: usize| {
            let fraction_from_top = row_from_top as f32 / (ROWS - 1) as f32;
            -RCS_DEFLECTOR_INBOARD_OFFSET_M + RCS_DEFLECTOR_BOW_M * fraction_from_top.powi(2)
        };
        offset_at_row(lower) * (1.0 - mix) + offset_at_row(upper) * mix
    }

    fn point_inside_visual_part(point_body_m: glam::DVec3, part: ApolloVisualPart) -> bool {
        if let ApolloShape::Strut {
            start_body_m,
            end_body_m,
            radius_m,
            ..
        } = part.shape
        {
            let segment = end_body_m - start_body_m;
            let fraction = ((point_body_m - start_body_m).dot(segment) / segment.length_squared())
                .clamp(0.0, 1.0);
            let closest = start_body_m + segment * fraction;
            return point_body_m.distance_squared(closest)
                <= (radius_m * part.scale.max_element()).powi(2);
        }

        let local =
            part.rotation_part_to_body.conjugate() * (point_body_m - part.translation_body_m);
        let unscaled = local / part.scale;
        match part.shape {
            ApolloShape::Cuboid { size_m } => {
                let half = size_m * 0.5;
                unscaled.x.abs() <= half.x
                    && unscaled.y.abs() <= half.y
                    && unscaled.z.abs() <= half.z
            }
            ApolloShape::Cylinder {
                radius_m, height_m, ..
            } => {
                unscaled.x * unscaled.x + unscaled.z * unscaled.z <= radius_m * radius_m
                    && unscaled.y.abs() <= height_m * 0.5
            }
            ApolloShape::Sphere { radius_m } => unscaled.length_squared() <= radius_m * radius_m,
            ApolloShape::Strut { .. } => unreachable!("struts handled above"),
        }
    }
}
