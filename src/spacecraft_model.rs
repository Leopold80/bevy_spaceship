use crate::apollo_spec::{ApolloMaterial, ApolloPart, ApolloShape, apollo_parts};
use bevy::asset::RenderAssetUsages;
use bevy::math::primitives::{Cone, Cuboid, Cylinder, Sphere};
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;
use std::f32::consts::{FRAC_PI_2, PI, TAU};

pub struct LanderMaterials {
    pub gold: Handle<StandardMaterial>,
    pub foil: Handle<StandardMaterial>,
    pub metal: Handle<StandardMaterial>,
    pub dark: Handle<StandardMaterial>,
    pub white: Handle<StandardMaterial>,
}

pub struct StarshipMaterials {
    pub stainless: Handle<StandardMaterial>,
    pub dark_tiles: Handle<StandardMaterial>,
    pub black: Handle<StandardMaterial>,
    pub engine: Handle<StandardMaterial>,
    pub glass: Handle<StandardMaterial>,
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

pub fn create_starship_materials(materials: &mut Assets<StandardMaterial>) -> StarshipMaterials {
    StarshipMaterials {
        stainless: materials.add(StandardMaterial {
            base_color: Color::srgb(0.78, 0.80, 0.79),
            metallic: 0.95,
            perceptual_roughness: 0.18,
            ..default()
        }),
        dark_tiles: materials.add(StandardMaterial {
            base_color: Color::srgb(0.025, 0.028, 0.032),
            metallic: 0.2,
            perceptual_roughness: 0.62,
            ..default()
        }),
        black: materials.add(StandardMaterial {
            base_color: Color::srgb(0.006, 0.007, 0.009),
            metallic: 0.35,
            perceptual_roughness: 0.38,
            ..default()
        }),
        engine: materials.add(StandardMaterial {
            base_color: Color::srgb(0.18, 0.18, 0.17),
            metallic: 0.9,
            perceptual_roughness: 0.24,
            ..default()
        }),
        glass: materials.add(StandardMaterial {
            base_color: Color::srgb(0.02, 0.055, 0.08),
            metallic: 0.1,
            perceptual_roughness: 0.16,
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

pub fn spawn_starship(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &StarshipMaterials,
    transform: Transform,
) -> Entity {
    let starship = commands.spawn((transform, Visibility::default())).id();

    commands.entity(starship).with_children(|parent| {
        spawn_starship_body(parent, meshes, materials);
        spawn_starship_heat_shield(parent, meshes, materials);
        spawn_starship_fins(parent, meshes, materials);
        spawn_starship_engines(parent, meshes, materials);
    });

    starship
}

fn spawn_starship_body(
    parent: &mut ChildSpawnerCommands,
    meshes: &mut Assets<Mesh>,
    materials: &StarshipMaterials,
) {
    parent.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.54, 4.35).mesh().resolution(72))),
        MeshMaterial3d(materials.stainless.clone()),
        Transform::from_xyz(0.0, 1.42, 0.0),
    ));

    parent.spawn((
        Mesh3d(meshes.add(rounded_starship_nose_mesh(72))),
        MeshMaterial3d(materials.stainless.clone()),
        Transform::default(),
    ));

    parent.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.57, 0.16).mesh().resolution(72))),
        MeshMaterial3d(materials.black.clone()),
        Transform::from_xyz(0.0, -0.83, 0.0),
    ));

    for y in [-0.48, -0.08, 0.32, 0.72, 1.12, 1.52, 1.92, 2.32, 2.72, 3.12] {
        parent.spawn((
            Mesh3d(meshes.add(Cylinder::new(0.545, 0.015).mesh().resolution(72))),
            MeshMaterial3d(materials.black.clone()),
            Transform::from_xyz(0.0, y, 0.0),
        ));
    }

    for (x, y) in [
        (-0.2, 3.55),
        (0.0, 3.58),
        (0.2, 3.55),
        (-0.1, 3.38),
        (0.1, 3.38),
    ] {
        parent.spawn((
            Mesh3d(meshes.add(Cuboid::new(0.105, 0.085, 0.03))),
            MeshMaterial3d(materials.glass.clone()),
            Transform::from_xyz(x, y, 0.54),
        ));
    }
}

fn spawn_starship_heat_shield(
    parent: &mut ChildSpawnerCommands,
    meshes: &mut Assets<Mesh>,
    materials: &StarshipMaterials,
) {
    let tile_mesh = meshes.add(Cylinder::new(0.035, 0.012).mesh().resolution(6));
    for row in 0..8 {
        let y = 3.32 + row as f32 * 0.16;
        let half_width = 0.35 - row as f32 * 0.033;
        let columns = (half_width / 0.078).floor() as i32;

        for column in -columns..=columns {
            let x = column as f32 * 0.078 + if row % 2 == 0 { 0.0 } else { 0.039 };
            if x.abs() > half_width {
                continue;
            }

            parent.spawn((
                Mesh3d(tile_mesh.clone()),
                MeshMaterial3d(materials.dark_tiles.clone()),
                Transform::from_xyz(x, y, 0.555).with_rotation(Quat::from_rotation_x(FRAC_PI_2)),
            ));
        }
    }

    for x in [-0.49, 0.49] {
        parent.spawn((
            Mesh3d(meshes.add(Cuboid::new(0.045, 3.8, 0.035))),
            MeshMaterial3d(materials.dark_tiles.clone()),
            Transform::from_xyz(x, 1.35, 0.275).with_rotation(Quat::from_euler(
                EulerRot::XYZ,
                0.0,
                x.signum() * 0.95,
                0.0,
            )),
        ));
    }

    for (x, y, height) in [(-0.36, 2.8, 0.18), (0.34, 1.5, 0.14), (-0.22, 0.25, 0.12)] {
        parent.spawn((
            Mesh3d(meshes.add(Cuboid::new(0.08, height, 0.022))),
            MeshMaterial3d(materials.glass.clone()),
            Transform::from_xyz(x, y, 0.545),
        ));
    }
}

fn spawn_starship_fins(
    parent: &mut ChildSpawnerCommands,
    meshes: &mut Assets<Mesh>,
    materials: &StarshipMaterials,
) {
    let lower_fin_mesh = meshes.add(trapezoid_fin_mesh(
        [
            Vec2::new(0.0, -0.64),
            Vec2::new(0.0, 0.72),
            Vec2::new(0.82, 0.34),
            Vec2::new(0.68, -0.38),
        ],
        0.05,
    ));
    let upper_fin_mesh = meshes.add(trapezoid_fin_mesh(
        [
            Vec2::new(0.0, -0.36),
            Vec2::new(0.0, 0.62),
            Vec2::new(0.58, 0.3),
            Vec2::new(0.72, -0.24),
        ],
        0.045,
    ));

    for side in [-1.0, 1.0] {
        parent.spawn((
            Mesh3d(lower_fin_mesh.clone()),
            MeshMaterial3d(materials.dark_tiles.clone()),
            Transform::from_xyz(side * 0.47, -0.18, 0.0).with_scale(Vec3::new(side, 1.0, 1.0)),
        ));

        parent.spawn((
            Mesh3d(upper_fin_mesh.clone()),
            MeshMaterial3d(materials.dark_tiles.clone()),
            Transform::from_xyz(side * 0.46, 3.02, 0.0).with_scale(Vec3::new(side, 1.0, 1.0)),
        ));
    }
}

fn spawn_starship_engines(
    parent: &mut ChildSpawnerCommands,
    meshes: &mut Assets<Mesh>,
    materials: &StarshipMaterials,
) {
    let engine_mesh = meshes.add(Cone::new(0.12, 0.34).mesh().resolution(32));
    let engine_positions = [
        Vec3::new(0.0, -1.05, 0.0),
        Vec3::new(0.2, -1.04, 0.16),
        Vec3::new(-0.2, -1.04, 0.16),
        Vec3::new(0.0, -1.04, -0.28),
        Vec3::new(0.27, -1.04, -0.14),
        Vec3::new(-0.27, -1.04, -0.14),
    ];

    for position in engine_positions {
        parent.spawn((
            Mesh3d(engine_mesh.clone()),
            MeshMaterial3d(materials.engine.clone()),
            Transform::from_translation(position).with_rotation(Quat::from_rotation_x(PI)),
        ));
    }
}

fn rounded_starship_nose_mesh(resolution: u32) -> Mesh {
    let profile = [
        Vec2::new(0.04, 4.84),
        Vec2::new(0.18, 4.76),
        Vec2::new(0.34, 4.48),
        Vec2::new(0.48, 4.02),
        Vec2::new(0.54, 3.6),
    ];
    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut uvs = Vec::new();
    let mut indices = Vec::new();

    for (ring, point) in profile.iter().enumerate() {
        for segment in 0..resolution {
            let theta = segment as f32 / resolution as f32 * TAU;
            let (sin, cos) = theta.sin_cos();
            positions.push([point.x * cos, point.y, point.x * sin]);
            normals.push(Vec3::new(cos, 0.28, sin).normalize().to_array());
            uvs.push([
                segment as f32 / resolution as f32,
                ring as f32 / (profile.len() - 1) as f32,
            ]);
        }
    }

    for ring in 0..profile.len() - 1 {
        let row = ring as u32 * resolution;
        let next_row = (ring as u32 + 1) * resolution;
        for segment in 0..resolution {
            let next_segment = (segment + 1) % resolution;
            indices.extend_from_slice(&[
                row + segment,
                next_row + segment,
                row + next_segment,
                row + next_segment,
                next_row + segment,
                next_row + next_segment,
            ]);
        }
    }

    Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    )
    .with_inserted_indices(Indices::U32(indices))
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
    .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, uvs)
}

fn trapezoid_fin_mesh(points: [Vec2; 4], thickness: f32) -> Mesh {
    let half = thickness * 0.5;
    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut uvs = Vec::new();

    for z in [half, -half] {
        for point in points {
            positions.push([point.x, point.y, z]);
            normals.push([0.0, 0.0, z.signum()]);
            uvs.push([point.x, point.y]);
        }
    }

    let indices = vec![
        0, 1, 2, 0, 2, 3, 4, 6, 5, 4, 7, 6, 0, 4, 5, 0, 5, 1, 1, 5, 6, 1, 6, 2, 2, 6, 7, 2, 7, 3,
        3, 7, 4, 3, 4, 0,
    ];

    Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    )
    .with_inserted_indices(Indices::U32(indices))
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
    .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, uvs)
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
