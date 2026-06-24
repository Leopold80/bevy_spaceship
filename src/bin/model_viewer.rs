use bevy::prelude::*;
use bevy_spacecraft::spacecraft_model::{create_lander_materials, spawn_lander};
use bevy_spacecraft::visualization::{
    create_star_material, spawn_default_camera_and_light, spawn_stars,
};

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
    spawn_default_camera_and_light(&mut commands);

    let lander_materials = create_lander_materials(&mut materials);
    let star = create_star_material(&mut materials);

    spawn_stars(&mut commands, &mut meshes, star);
    spawn_lander(
        &mut commands,
        &mut meshes,
        &lander_materials,
        Transform::from_xyz(0.0, 0.85, 0.0),
    );
}
