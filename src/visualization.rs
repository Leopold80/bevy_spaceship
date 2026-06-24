use bevy::light::NotShadowCaster;
use bevy::math::primitives::{Cone, Cylinder, Sphere};
use bevy::prelude::*;
use std::f32::consts::PI;

pub const TARGET_FRAME_CENTER: Vec3 = Vec3::new(3.7, 2.35, -1.45);
pub const TARGET_FRAME_AXIS_LENGTH: f32 = 1.8;

pub struct ReferenceFrameMaterials {
    pub target_axis_x: Handle<StandardMaterial>,
    pub target_axis_y: Handle<StandardMaterial>,
    pub target_axis_z: Handle<StandardMaterial>,
    pub current_axis_x: Handle<StandardMaterial>,
    pub current_axis_y: Handle<StandardMaterial>,
    pub current_axis_z: Handle<StandardMaterial>,
    pub origin: Handle<StandardMaterial>,
}

pub fn spawn_default_camera_and_light(commands: &mut Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(5.2, 3.4, 7.0).looking_at(Vec3::new(0.0, 0.7, 0.0), Vec3::Y),
    ));

    commands.spawn((
        DirectionalLight {
            illuminance: 18_000.0,
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
        target_axis_x: materials.add(StandardMaterial {
            base_color: Color::srgb(0.95, 0.12, 0.1),
            emissive: LinearRgba::rgb(0.35, 0.02, 0.02),
            unlit: true,
            ..default()
        }),
        target_axis_y: materials.add(StandardMaterial {
            base_color: Color::srgb(0.1, 0.85, 0.24),
            emissive: LinearRgba::rgb(0.02, 0.28, 0.05),
            unlit: true,
            ..default()
        }),
        target_axis_z: materials.add(StandardMaterial {
            base_color: Color::srgb(0.18, 0.36, 1.0),
            emissive: LinearRgba::rgb(0.04, 0.08, 0.35),
            unlit: true,
            ..default()
        }),
        current_axis_x: materials.add(StandardMaterial {
            base_color: Color::srgba(1.0, 0.48, 0.42, 0.45),
            emissive: LinearRgba::rgb(0.22, 0.04, 0.03),
            unlit: true,
            alpha_mode: AlphaMode::Blend,
            ..default()
        }),
        current_axis_y: materials.add(StandardMaterial {
            base_color: Color::srgba(0.5, 1.0, 0.58, 0.45),
            emissive: LinearRgba::rgb(0.04, 0.2, 0.04),
            unlit: true,
            alpha_mode: AlphaMode::Blend,
            ..default()
        }),
        current_axis_z: materials.add(StandardMaterial {
            base_color: Color::srgba(0.56, 0.72, 1.0, 0.45),
            emissive: LinearRgba::rgb(0.04, 0.06, 0.22),
            unlit: true,
            alpha_mode: AlphaMode::Blend,
            ..default()
        }),
        origin: materials.add(StandardMaterial {
            base_color: Color::srgb(0.92, 0.96, 1.0),
            emissive: LinearRgba::rgb(0.28, 0.3, 0.34),
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
    let star_mesh = meshes.add(Sphere::new(0.025).mesh().uv(8, 4));
    for i in 0..80 {
        let fi = i as f32;
        let x = (fi * 12.9898).sin() * 10.0;
        let y = 2.8 + (fi * 78.233).sin().abs() * 4.5;
        let z = -7.0 - (fi * 37.719).cos().abs() * 7.0;
        commands.spawn((
            Mesh3d(star_mesh.clone()),
            MeshMaterial3d(material.clone()),
            Transform::from_xyz(x, y, z),
        ));
    }
}

pub fn spawn_target_frame(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &ReferenceFrameMaterials,
    target: Quat,
) {
    let origin = TARGET_FRAME_CENTER;

    spawn_reference_sphere(commands, meshes, materials.origin.clone(), origin, 0.065);
    spawn_axis_label(
        commands,
        "target q_d + current q",
        origin + Vec3::Y * 0.92,
        Color::srgb(0.92, 0.96, 1.0),
    );

    spawn_arrow(
        commands,
        meshes,
        materials.target_axis_x.clone(),
        origin,
        origin + target * Vec3::X * TARGET_FRAME_AXIS_LENGTH,
        AxisStyle::target_frame(),
    );
    spawn_axis_label(
        commands,
        "X",
        origin + target * Vec3::X * (TARGET_FRAME_AXIS_LENGTH + 0.32),
        Color::srgb(1.0, 0.42, 0.36),
    );

    spawn_arrow(
        commands,
        meshes,
        materials.target_axis_y.clone(),
        origin,
        origin + target * Vec3::Y * TARGET_FRAME_AXIS_LENGTH,
        AxisStyle::target_frame(),
    );
    spawn_axis_label(
        commands,
        "Y",
        origin + target * Vec3::Y * (TARGET_FRAME_AXIS_LENGTH + 0.32),
        Color::srgb(0.42, 1.0, 0.52),
    );

    spawn_arrow(
        commands,
        meshes,
        materials.target_axis_z.clone(),
        origin,
        origin + target * Vec3::Z * TARGET_FRAME_AXIS_LENGTH,
        AxisStyle::target_frame(),
    );
    spawn_axis_label(
        commands,
        "Z",
        origin + target * Vec3::Z * (TARGET_FRAME_AXIS_LENGTH + 0.32),
        Color::srgb(0.52, 0.68, 1.0),
    );
}

pub fn spawn_current_frame(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &ReferenceFrameMaterials,
    current: Quat,
) -> Entity {
    let root = commands
        .spawn((
            Transform::from_translation(TARGET_FRAME_CENTER).with_rotation(current),
            Visibility::default(),
        ))
        .id();

    commands.entity(root).with_children(|parent| {
        parent.spawn((
            Mesh3d(meshes.add(Sphere::new(0.045).mesh().uv(12, 6))),
            MeshMaterial3d(materials.origin.clone()),
            Transform::default(),
            NotShadowCaster,
        ));
        spawn_local_arrow(
            parent,
            meshes,
            materials.current_axis_x.clone(),
            Vec3::ZERO,
            Vec3::X * TARGET_FRAME_AXIS_LENGTH,
            AxisStyle::current_frame(),
        );
        spawn_local_arrow(
            parent,
            meshes,
            materials.current_axis_y.clone(),
            Vec3::ZERO,
            Vec3::Y * TARGET_FRAME_AXIS_LENGTH,
            AxisStyle::current_frame(),
        );
        spawn_local_arrow(
            parent,
            meshes,
            materials.current_axis_z.clone(),
            Vec3::ZERO,
            Vec3::Z * TARGET_FRAME_AXIS_LENGTH,
            AxisStyle::current_frame(),
        );
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
    fn target_frame() -> Self {
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

fn spawn_arrow(
    commands: &mut Commands,
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

    let dir = delta / length;
    let shaft_end = end - dir * style.head_length;
    spawn_world_cylinder_between(
        commands,
        meshes.add(Cylinder::new(style.shaft_radius, 1.0).mesh().resolution(14)),
        material.clone(),
        start,
        shaft_end,
    );

    commands.spawn((
        Mesh3d(
            meshes.add(
                Cone::new(style.head_radius, style.head_length)
                    .mesh()
                    .resolution(18),
            ),
        ),
        MeshMaterial3d(material),
        Transform::from_translation(shaft_end + dir * style.head_length * 0.5)
            .with_rotation(Quat::from_rotation_arc(Vec3::Y, dir)),
        NotShadowCaster,
    ));
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

    let dir = delta / length;
    let shaft_end = end - dir * style.head_length;
    let shaft_midpoint = (start + shaft_end) * 0.5;

    parent.spawn((
        Mesh3d(meshes.add(Cylinder::new(style.shaft_radius, 1.0).mesh().resolution(14))),
        MeshMaterial3d(material.clone()),
        Transform::from_translation(shaft_midpoint)
            .with_rotation(Quat::from_rotation_arc(Vec3::Y, dir))
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
        Transform::from_translation(shaft_end + dir * style.head_length * 0.5)
            .with_rotation(Quat::from_rotation_arc(Vec3::Y, dir)),
        NotShadowCaster,
    ));
}

fn spawn_world_cylinder_between(
    commands: &mut Commands,
    mesh: Handle<Mesh>,
    material: Handle<StandardMaterial>,
    start: Vec3,
    end: Vec3,
) {
    let midpoint = (start + end) * 0.5;
    let delta = end - start;
    let rotation = Quat::from_rotation_arc(Vec3::Y, delta.normalize());

    commands.spawn((
        Mesh3d(mesh),
        MeshMaterial3d(material),
        Transform::from_translation(midpoint)
            .with_rotation(rotation)
            .with_scale(Vec3::new(1.0, delta.length(), 1.0)),
        NotShadowCaster,
    ));
}

fn spawn_axis_label(commands: &mut Commands, text: &str, position: Vec3, color: Color) {
    commands.spawn((
        Text2d::new(text),
        TextFont::from_font_size(22.0),
        TextColor(color),
        Transform::from_translation(position),
    ));
}

fn spawn_reference_sphere(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    material: Handle<StandardMaterial>,
    position: Vec3,
    radius: f32,
) {
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(radius).mesh().uv(12, 6))),
        MeshMaterial3d(material),
        Transform::from_translation(position),
        NotShadowCaster,
    ));
}
