use apollo_core::{ApolloMaterial, ApolloShape, ApolloVisualPart, apollo_visual_parts};
use bevy::math::primitives::{Cuboid, Cylinder, Sphere};
use bevy::prelude::*;

pub struct LanderMaterials {
    gold: Handle<StandardMaterial>,
    foil: Handle<StandardMaterial>,
    metal: Handle<StandardMaterial>,
    dark: Handle<StandardMaterial>,
    white: Handle<StandardMaterial>,
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
        for part in apollo_visual_parts() {
            spawn_apollo_part(parent, meshes, materials, part);
        }
    });

    lander
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

#[cfg(test)]
mod tests {
    use super::*;
    use apollo_core::ApolloState;

    #[test]
    fn f64_pose_conversion_preserves_identity_and_axes() {
        assert_eq!(
            dvec3(glam::DVec3::new(1.0, -2.0, 3.0)),
            Vec3::new(1.0, -2.0, 3.0)
        );
        assert!(dquat(ApolloState::ZERO.body_to_world).dot(Quat::IDENTITY) > 1.0 - 1.0e-6);
    }
}
