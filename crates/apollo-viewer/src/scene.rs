use bevy::light::NotShadowCaster;
use bevy::math::primitives::{Cone, Cylinder, Sphere};
use bevy::prelude::*;
use std::f32::consts::PI;

/// 固定放在画面右侧的参考坐标系中心；它不随 plant 平移。
pub const REFERENCE_FRAME_CENTER: Vec3 = Vec3::new(7.4, 4.7, -2.9);
/// 参考坐标轴长度。
pub const REFERENCE_FRAME_AXIS_LENGTH: f32 = 3.6;

/// 期望姿态使用不透明粗线，当前姿态使用半透明细线。
pub struct ReferenceFrameMaterials {
    desired_axis_x: Handle<StandardMaterial>,
    desired_axis_y: Handle<StandardMaterial>,
    desired_axis_z: Handle<StandardMaterial>,
    current_axis_x: Handle<StandardMaterial>,
    current_axis_y: Handle<StandardMaterial>,
    current_axis_z: Handle<StandardMaterial>,
    origin: Handle<StandardMaterial>,
}

pub fn default_window(title: &str) -> WindowPlugin {
    WindowPlugin {
        primary_window: Some(Window {
            title: title.into(),
            resolution: (1280, 800).into(),
            ..default()
        }),
        ..default()
    }
}

pub fn insert_default_lighting(app: &mut App) {
    app.insert_resource(GlobalAmbientLight {
        color: Color::srgb(0.45, 0.5, 0.65),
        brightness: 900.0,
        affects_lightmapped_meshes: true,
    });
}

pub fn spawn_camera_and_light(commands: &mut Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(11.6, 6.8, 17.6).looking_at(Vec3::new(0.4, 2.4, 0.0), Vec3::Y),
    ));

    commands.spawn((
        DirectionalLight {
            illuminance: 22_000.0,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -PI / 4.0, PI / 5.0, 0.0)),
    ));
}

pub fn create_star_material(materials: &mut Assets<StandardMaterial>) -> Handle<StandardMaterial> {
    materials.add(StandardMaterial {
        base_color: Color::WHITE,
        emissive: LinearRgba::rgb(2.8, 2.8, 3.4),
        ..default()
    })
}

pub fn create_reference_frame_materials(
    materials: &mut Assets<StandardMaterial>,
) -> ReferenceFrameMaterials {
    ReferenceFrameMaterials {
        desired_axis_x: materials.add(StandardMaterial {
            base_color: Color::srgb(0.95, 0.12, 0.10),
            emissive: LinearRgba::rgb(0.35, 0.02, 0.02),
            unlit: true,
            ..default()
        }),
        desired_axis_y: materials.add(StandardMaterial {
            base_color: Color::srgb(0.10, 0.85, 0.24),
            emissive: LinearRgba::rgb(0.02, 0.28, 0.05),
            unlit: true,
            ..default()
        }),
        desired_axis_z: materials.add(StandardMaterial {
            base_color: Color::srgb(0.18, 0.36, 1.00),
            emissive: LinearRgba::rgb(0.04, 0.08, 0.35),
            unlit: true,
            ..default()
        }),
        current_axis_x: materials.add(StandardMaterial {
            base_color: Color::srgba(1.00, 0.48, 0.42, 0.45),
            emissive: LinearRgba::rgb(0.22, 0.04, 0.03),
            unlit: true,
            alpha_mode: AlphaMode::Blend,
            ..default()
        }),
        current_axis_y: materials.add(StandardMaterial {
            base_color: Color::srgba(0.50, 1.00, 0.58, 0.45),
            emissive: LinearRgba::rgb(0.04, 0.20, 0.04),
            unlit: true,
            alpha_mode: AlphaMode::Blend,
            ..default()
        }),
        current_axis_z: materials.add(StandardMaterial {
            base_color: Color::srgba(0.56, 0.72, 1.00, 0.45),
            emissive: LinearRgba::rgb(0.04, 0.06, 0.22),
            unlit: true,
            alpha_mode: AlphaMode::Blend,
            ..default()
        }),
        origin: materials.add(StandardMaterial {
            base_color: Color::srgb(0.92, 0.96, 1.00),
            emissive: LinearRgba::rgb(0.28, 0.30, 0.34),
            unlit: true,
            ..default()
        }),
    }
}

pub fn spawn_stars(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    material: Handle<StandardMaterial>,
) {
    let star_mesh = meshes.add(Sphere::new(0.05).mesh().uv(8, 4));
    for index in 0..80 {
        let value = index as f32;
        let x = (value * 12.9898).sin() * 20.0;
        let y = 5.6 + (value * 78.233).sin().abs() * 9.0;
        let z = -14.0 - (value * 37.719).cos().abs() * 14.0;
        commands.spawn((
            Mesh3d(star_mesh.clone()),
            MeshMaterial3d(material.clone()),
            Transform::from_xyz(x, y, z),
        ));
    }
}

/// 生成参考系图例；标签保持纯 ASCII，兼容 Bevy 默认字体。
pub fn spawn_attitude_frame_legend(commands: &mut Commands, desired_available: bool) {
    let legend = if desired_available {
        "DESIRED solid / CURRENT transparent"
    } else {
        "DESIRED unavailable / CURRENT transparent"
    };
    spawn_axis_label(
        commands,
        legend,
        REFERENCE_FRAME_CENTER + Vec3::Y * 0.92,
        Color::srgb(0.92, 0.96, 1.0),
        Visibility::Inherited,
    );
}

/// 生成期望姿态三轴的 X/Y/Z 标签；返回顺序固定为 X、Y、Z。
pub fn spawn_desired_attitude_axis_labels(
    commands: &mut Commands,
    desired_body_to_world: Quat,
    visibility: Visibility,
) -> [Entity; 3] {
    [
        spawn_axis_label(
            commands,
            "X",
            desired_attitude_axis_label_position(desired_body_to_world, Vec3::X),
            Color::srgb(1.0, 0.42, 0.36),
            visibility,
        ),
        spawn_axis_label(
            commands,
            "Y",
            desired_attitude_axis_label_position(desired_body_to_world, Vec3::Y),
            Color::srgb(0.42, 1.0, 0.52),
            visibility,
        ),
        spawn_axis_label(
            commands,
            "Z",
            desired_attitude_axis_label_position(desired_body_to_world, Vec3::Z),
            Color::srgb(0.52, 0.68, 1.0),
            visibility,
        ),
    ]
}

/// 计算一个期望姿态轴标签在叠加坐标系中的世界位置。
pub fn desired_attitude_axis_label_position(desired_body_to_world: Quat, local_axis: Vec3) -> Vec3 {
    REFERENCE_FRAME_CENTER
        + desired_body_to_world * local_axis * (REFERENCE_FRAME_AXIS_LENGTH + 0.32)
}

/// 生成可由调用方同步的粗实线期望姿态坐标系，并返回根实体。
pub fn spawn_desired_attitude_frame(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &ReferenceFrameMaterials,
    desired_body_to_world: Quat,
    visibility: Visibility,
) -> Entity {
    spawn_attitude_frame(
        commands,
        meshes,
        desired_body_to_world,
        visibility,
        AttitudeFrameAppearance {
            axis_materials: [
                materials.desired_axis_x.clone(),
                materials.desired_axis_y.clone(),
                materials.desired_axis_z.clone(),
            ],
            origin_material: materials.origin.clone(),
            axis_style: AxisStyle::desired_frame(),
            origin_radius: 0.065,
        },
    )
}

/// 生成可由调用方同步的细半透明当前姿态坐标系，并返回根实体。
pub fn spawn_current_attitude_frame(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &ReferenceFrameMaterials,
    body_to_world: Quat,
) -> Entity {
    spawn_attitude_frame(
        commands,
        meshes,
        body_to_world,
        Visibility::Inherited,
        AttitudeFrameAppearance {
            axis_materials: [
                materials.current_axis_x.clone(),
                materials.current_axis_y.clone(),
                materials.current_axis_z.clone(),
            ],
            origin_material: materials.origin.clone(),
            axis_style: AxisStyle::current_frame(),
            origin_radius: 0.045,
        },
    )
}

struct AttitudeFrameAppearance {
    axis_materials: [Handle<StandardMaterial>; 3],
    origin_material: Handle<StandardMaterial>,
    axis_style: AxisStyle,
    origin_radius: f32,
}

fn spawn_attitude_frame(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    body_to_world: Quat,
    visibility: Visibility,
    appearance: AttitudeFrameAppearance,
) -> Entity {
    let root = commands
        .spawn((
            Transform::from_translation(REFERENCE_FRAME_CENTER).with_rotation(body_to_world),
            visibility,
        ))
        .id();

    commands.entity(root).with_children(|parent| {
        parent.spawn((
            Mesh3d(meshes.add(Sphere::new(appearance.origin_radius).mesh().uv(12, 6))),
            MeshMaterial3d(appearance.origin_material),
            Transform::default(),
            NotShadowCaster,
        ));
        for (axis, material) in [Vec3::X, Vec3::Y, Vec3::Z]
            .into_iter()
            .zip(appearance.axis_materials)
        {
            spawn_local_arrow(
                parent,
                meshes,
                material,
                Vec3::ZERO,
                axis * REFERENCE_FRAME_AXIS_LENGTH,
                appearance.axis_style,
            );
        }
    });

    root
}

#[derive(Clone, Copy)]
struct AxisStyle {
    shaft_radius: f32,
    head_radius: f32,
    head_length: f32,
}

impl AxisStyle {
    fn desired_frame() -> Self {
        Self {
            shaft_radius: 0.018,
            head_radius: 0.075,
            head_length: 0.28,
        }
    }

    fn current_frame() -> Self {
        Self {
            shaft_radius: 0.011,
            head_radius: 0.055,
            head_length: 0.22,
        }
    }
}

fn spawn_local_arrow(
    parent: &mut ChildSpawnerCommands,
    meshes: &mut Assets<Mesh>,
    material: Handle<StandardMaterial>,
    start: Vec3,
    end: Vec3,
    style: AxisStyle,
) {
    let delta = end - start;
    let length = delta.length();
    if length <= style.head_length {
        return;
    }

    let direction = delta / length;
    let shaft_end = end - direction * style.head_length;
    parent.spawn((
        Mesh3d(meshes.add(Cylinder::new(style.shaft_radius, 1.0).mesh().resolution(14))),
        MeshMaterial3d(material.clone()),
        Transform::from_translation((start + shaft_end) * 0.5)
            .with_rotation(Quat::from_rotation_arc(Vec3::Y, direction))
            .with_scale(Vec3::new(1.0, (shaft_end - start).length(), 1.0)),
        NotShadowCaster,
    ));
    parent.spawn((
        Mesh3d(
            meshes.add(
                Cone::new(style.head_radius, style.head_length)
                    .mesh()
                    .resolution(18),
            ),
        ),
        MeshMaterial3d(material),
        Transform::from_translation(shaft_end + direction * style.head_length * 0.5)
            .with_rotation(Quat::from_rotation_arc(Vec3::Y, direction)),
        NotShadowCaster,
    ));
}

fn spawn_axis_label(
    commands: &mut Commands,
    text: &str,
    position: Vec3,
    color: Color,
    visibility: Visibility,
) -> Entity {
    commands
        .spawn((
            Text2d::new(text),
            TextFont::from_font_size(22.0),
            TextColor(color),
            Transform::from_translation(position),
            visibility,
        ))
        .id()
}
