use bevy::math::primitives::Sphere;
use bevy::prelude::*;
use std::f32::consts::PI;

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
