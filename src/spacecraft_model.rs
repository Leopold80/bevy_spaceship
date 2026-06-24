use bevy::math::primitives::{Cuboid, Cylinder, Sphere};
use bevy::prelude::*;
use std::f32::consts::{FRAC_PI_2, PI};

pub struct LanderMaterials {
    pub gold: Handle<StandardMaterial>,
    pub foil: Handle<StandardMaterial>,
    pub metal: Handle<StandardMaterial>,
    pub dark: Handle<StandardMaterial>,
    pub white: Handle<StandardMaterial>,
}

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
    }
}

pub fn spawn_lander(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &LanderMaterials,
    transform: Transform,
) -> Entity {
    let lander = commands.spawn((transform, Visibility::default())).id();

    commands.entity(lander).with_children(|parent| {
        spawn_body(
            parent,
            meshes,
            materials.metal.clone(),
            materials.gold.clone(),
            materials.dark.clone(),
            materials.white.clone(),
        );
        spawn_landing_gear(
            parent,
            meshes,
            materials.metal.clone(),
            materials.foil.clone(),
            materials.dark.clone(),
        );
        spawn_antennas(
            parent,
            meshes,
            materials.metal.clone(),
            materials.dark.clone(),
        );
    });

    lander
}

fn spawn_body(
    parent: &mut ChildSpawnerCommands,
    meshes: &mut Assets<Mesh>,
    metal: Handle<StandardMaterial>,
    gold: Handle<StandardMaterial>,
    dark: Handle<StandardMaterial>,
    white: Handle<StandardMaterial>,
) {
    parent.spawn((
        Mesh3d(meshes.add(Cylinder::new(1.25, 1.25).mesh().resolution(8))),
        MeshMaterial3d(metal.clone()),
        Transform::from_xyz(0.0, 0.62, 0.0)
            .with_rotation(Quat::from_rotation_y(PI / 8.0))
            .with_scale(Vec3::new(1.18, 0.82, 1.0)),
    ));

    parent.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.82, 0.72).mesh().resolution(8))),
        MeshMaterial3d(gold.clone()),
        Transform::from_xyz(0.0, 1.5, 0.0)
            .with_rotation(Quat::from_rotation_y(PI / 8.0))
            .with_scale(Vec3::new(1.0, 0.8, 0.92)),
    ));

    parent.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.38, 0.28).mesh().resolution(24))),
        MeshMaterial3d(white),
        Transform::from_xyz(0.0, 2.05, 0.0),
    ));

    parent.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.74, 0.52, 0.08))),
        MeshMaterial3d(dark.clone()),
        Transform::from_xyz(0.0, 0.78, 1.03),
    ));

    parent.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.34, 0.42, 0.09))),
        MeshMaterial3d(dark),
        Transform::from_xyz(0.48, 1.26, 0.78).with_rotation(Quat::from_rotation_y(-0.32)),
    ));

    parent.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.32, 0.86, 0.12))),
        MeshMaterial3d(gold.clone()),
        Transform::from_xyz(-0.95, 0.55, 0.08).with_rotation(Quat::from_rotation_z(0.28)),
    ));

    parent.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.32, 0.86, 0.12))),
        MeshMaterial3d(gold),
        Transform::from_xyz(0.95, 0.55, -0.08).with_rotation(Quat::from_rotation_z(-0.28)),
    ));
}

fn spawn_landing_gear(
    parent: &mut ChildSpawnerCommands,
    meshes: &mut Assets<Mesh>,
    metal: Handle<StandardMaterial>,
    foil: Handle<StandardMaterial>,
    dark: Handle<StandardMaterial>,
) {
    let strut_mesh = meshes.add(Cylinder::new(0.035, 1.75).mesh().resolution(12));
    let brace_mesh = meshes.add(Cylinder::new(0.022, 1.35).mesh().resolution(10));
    let foot_mesh = meshes.add(Cylinder::new(0.38, 0.09).mesh().resolution(32));
    let leg_mesh = meshes.add(Cuboid::new(0.12, 0.42, 0.12));

    for i in 0..4 {
        let angle = i as f32 * FRAC_PI_2 + PI / 4.0;
        let dir = Vec3::new(angle.cos(), 0.0, angle.sin());
        let foot = dir * 2.05 + Vec3::new(0.0, -1.16, 0.0);
        let upper = dir * 0.78 + Vec3::new(0.0, 0.18, 0.0);

        spawn_cylinder_between(parent, strut_mesh.clone(), metal.clone(), upper, foot);
        spawn_cylinder_between(
            parent,
            brace_mesh.clone(),
            metal.clone(),
            Vec3::new(0.0, 0.15, 0.0),
            foot + Vec3::new(0.0, 0.22, 0.0),
        );

        parent.spawn((
            Mesh3d(foot_mesh.clone()),
            MeshMaterial3d(foil.clone()),
            Transform::from_translation(foot),
        ));

        parent.spawn((
            Mesh3d(leg_mesh.clone()),
            MeshMaterial3d(dark.clone()),
            Transform::from_translation(upper + dir * 0.18)
                .with_rotation(Quat::from_rotation_y(-angle)),
        ));
    }
}

fn spawn_antennas(
    parent: &mut ChildSpawnerCommands,
    meshes: &mut Assets<Mesh>,
    metal: Handle<StandardMaterial>,
    dark: Handle<StandardMaterial>,
) {
    parent.spawn((
        Mesh3d(meshes.add(Sphere::new(0.28).mesh().uv(16, 8))),
        MeshMaterial3d(dark.clone()),
        Transform::from_xyz(-1.18, 2.08, -0.78).with_scale(Vec3::new(1.0, 0.12, 1.0)),
    ));

    parent.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.035, 0.16).mesh().resolution(10))),
        MeshMaterial3d(metal.clone()),
        Transform::from_xyz(-1.18, 1.97, -0.78),
    ));

    spawn_cylinder_between(
        parent,
        meshes.add(Cylinder::new(0.018, 0.55).mesh().resolution(8)),
        metal.clone(),
        Vec3::new(-0.74, 1.72, -0.45),
        Vec3::new(-1.12, 1.92, -0.74),
    );

    parent.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.08, 0.7).mesh().resolution(16))),
        MeshMaterial3d(metal),
        Transform::from_xyz(0.72, 1.92, -0.35).with_rotation(Quat::from_rotation_z(0.35)),
    ));
}

fn spawn_cylinder_between(
    parent: &mut ChildSpawnerCommands,
    mesh: Handle<Mesh>,
    material: Handle<StandardMaterial>,
    start: Vec3,
    end: Vec3,
) {
    let midpoint = (start + end) * 0.5;
    let delta = end - start;
    let rotation = Quat::from_rotation_arc(Vec3::Y, delta.normalize());

    parent.spawn((
        Mesh3d(mesh),
        MeshMaterial3d(material),
        Transform::from_translation(midpoint).with_rotation(rotation),
    ));
}
