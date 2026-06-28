use crate::apollo_spec::{ApolloMaterial, ApolloPart, ApolloShape, apollo_parts};
use bevy::math::primitives::{Cuboid, Cylinder, Sphere};
use bevy::prelude::*;

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
        for part in apollo_parts() {
            spawn_apollo_part(parent, meshes, materials, part);
        }
    });

    lander
}

fn spawn_apollo_part(
    parent: &mut ChildSpawnerCommands,
    meshes: &mut Assets<Mesh>,
    materials: &LanderMaterials,
    part: ApolloPart,
) {
    let material = apollo_material(materials, part.material);
    match part.shape {
        ApolloShape::Cuboid { size } => {
            parent.spawn((
                Mesh3d(meshes.add(Cuboid::from_size(size))),
                MeshMaterial3d(material),
                part.visual_transform(),
            ));
        }
        ApolloShape::Cylinder {
            radius,
            height,
            resolution,
        } => {
            parent.spawn((
                Mesh3d(meshes.add(Cylinder::new(radius, height).mesh().resolution(resolution))),
                MeshMaterial3d(material),
                part.visual_transform(),
            ));
        }
        ApolloShape::Sphere { radius } => {
            parent.spawn((
                Mesh3d(meshes.add(Sphere::new(radius).mesh().uv(16, 8))),
                MeshMaterial3d(material),
                part.visual_transform(),
            ));
        }
        ApolloShape::Strut {
            start,
            end,
            radius,
            resolution,
        } => {
            spawn_cylinder_between(
                parent,
                meshes.add(
                    Cylinder::new(radius, start.distance(end))
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
    let midpoint = (start + end) * 0.5;
    let delta = end - start;
    let rotation = Quat::from_rotation_arc(Vec3::Y, delta.normalize());

    parent.spawn((
        Mesh3d(mesh),
        MeshMaterial3d(material),
        Transform::from_translation(midpoint).with_rotation(rotation),
    ));
}
