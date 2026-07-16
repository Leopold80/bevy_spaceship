use apollo_viewer::model::{create_lander_materials, spawn_lander};
use apollo_viewer::scene::{
    create_star_material, default_window, insert_default_lighting, spawn_camera_and_light,
    spawn_stars,
};
use bevy::prelude::*;

fn main() {
    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(default_window("Apollo Model Viewer")));
    insert_default_lighting(&mut app);
    app.add_systems(Startup, setup).run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    spawn_camera_and_light(&mut commands);
    let lander_materials = create_lander_materials(&mut materials);
    let star = create_star_material(&mut materials);
    spawn_stars(&mut commands, &mut meshes, star);
    spawn_lander(
        &mut commands,
        &mut meshes,
        &lander_materials,
        Transform::from_xyz(0.0, 0.65, 0.0).with_scale(Vec3::splat(0.82)),
    );
}
