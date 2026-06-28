use bevy::prelude::*;
use bevy_spacecraft::spacecraft_model::{
    create_lander_materials, create_starship_materials, spawn_lander, spawn_starship,
};
use bevy_spacecraft::visualization::{create_star_material, spawn_stars};
use std::f32::consts::PI;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Bevy Spacecraft Model Viewer".into(),
                resolution: (1280, 800).into(),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(GlobalAmbientLight {
            color: Color::srgb(0.45, 0.5, 0.65),
            brightness: 900.0,
            affects_lightmapped_meshes: true,
        })
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    spawn_viewer_camera_and_light(&mut commands);

    let lander_materials = create_lander_materials(&mut materials);
    let starship_materials = create_starship_materials(&mut materials);
    let star = create_star_material(&mut materials);

    spawn_stars(&mut commands, &mut meshes, star);
    spawn_lander(
        &mut commands,
        &mut meshes,
        &lander_materials,
        Transform::from_xyz(-2.15, 0.65, 0.0).with_scale(Vec3::splat(0.82)),
    );
    spawn_starship(
        &mut commands,
        &mut meshes,
        &starship_materials,
        Transform::from_xyz(2.0, 0.15, 0.0),
    );
}

fn spawn_viewer_camera_and_light(commands: &mut Commands) {
    commands.spawn((
        Camera3d::default(),
        // Scaled 2× to match the rescaled lander geometry.
        Transform::from_xyz(11.6, 6.8, 17.6).looking_at(Vec3::new(0.4, 3.1, 0.0), Vec3::Y),
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
