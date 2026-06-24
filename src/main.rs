mod attitude_control;

use attitude_control::{
    ATTITUDE_KP, AttitudeSample, AttitudeScenario, ControlLaw, attitude_command, current_scenario,
    desired_axis_angle, integrate_attitude, scenario_at, target_attitude,
};
use bevy::light::NotShadowCaster;
use bevy::math::primitives::{Cone, Cuboid, Cylinder, Sphere};
use bevy::prelude::*;
use std::f32::consts::{FRAC_PI_2, PI};
use std::fs::{File, create_dir_all};
use std::io::Write;

const LOG_PATH: &str = "logs/attitude_kinematics.csv";
const LOG_INTERVAL_SECS: f32 = 0.1;
const DEMO_CENTER: Vec3 = Vec3::new(0.0, 0.85, 0.0);
const TARGET_FRAME_CENTER: Vec3 = Vec3::new(3.7, 2.35, -1.45);
const TARGET_FRAME_AXIS_LENGTH: f32 = 1.8;

#[derive(Component)]
struct LanderRoot;

#[derive(Component)]
struct CurrentFrameRoot;

#[derive(Resource)]
struct AttitudeController {
    target: Quat,
    kp: f32,
    control_law: ControlLaw,
    scenario_index: usize,
    paused: bool,
    elapsed_secs: f32,
    next_log_secs: f32,
}

#[derive(Resource)]
struct AttitudeLog {
    path: String,
    file: File,
}

#[derive(Component)]
struct StatusText;

fn main() {
    if std::env::args().any(|arg| arg == "--headless-log") {
        run_headless_attitude_log(LOG_PATH, 8.0, 1.0 / 60.0);
        return;
    }

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Bevy Apollo-style Lunar Module".into(),
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
        .insert_resource(AttitudeController {
            target: target_attitude(),
            kp: ATTITUDE_KP,
            control_law: ControlLaw::ScaledQuaternion,
            scenario_index: 0,
            paused: false,
            elapsed_secs: 0.0,
            next_log_secs: 0.0,
        })
        .insert_resource(AttitudeLog::new(LOG_PATH))
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                handle_attitude_demo_input,
                control_lander_attitude,
                sync_current_frame,
                update_status_text,
            )
                .chain(),
        )
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
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

    let gold = materials.add(StandardMaterial {
        base_color: Color::srgb(0.92, 0.68, 0.22),
        metallic: 0.65,
        perceptual_roughness: 0.34,
        ..default()
    });
    let foil = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.78, 0.28),
        metallic: 0.9,
        perceptual_roughness: 0.18,
        ..default()
    });
    let metal = materials.add(StandardMaterial {
        base_color: Color::srgb(0.64, 0.67, 0.70),
        metallic: 0.85,
        perceptual_roughness: 0.28,
        ..default()
    });
    let dark = materials.add(StandardMaterial {
        base_color: Color::srgb(0.035, 0.038, 0.045),
        metallic: 0.25,
        perceptual_roughness: 0.45,
        ..default()
    });
    let white = materials.add(StandardMaterial {
        base_color: Color::srgb(0.86, 0.88, 0.84),
        metallic: 0.15,
        perceptual_roughness: 0.55,
        ..default()
    });
    let star = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        emissive: LinearRgba::rgb(2.8, 2.8, 3.4),
        ..default()
    });
    let axis_x = materials.add(StandardMaterial {
        base_color: Color::srgb(0.95, 0.12, 0.1),
        emissive: LinearRgba::rgb(0.35, 0.02, 0.02),
        unlit: true,
        ..default()
    });
    let axis_y = materials.add(StandardMaterial {
        base_color: Color::srgb(0.1, 0.85, 0.24),
        emissive: LinearRgba::rgb(0.02, 0.28, 0.05),
        unlit: true,
        ..default()
    });
    let axis_z = materials.add(StandardMaterial {
        base_color: Color::srgb(0.18, 0.36, 1.0),
        emissive: LinearRgba::rgb(0.04, 0.08, 0.35),
        unlit: true,
        ..default()
    });
    let current_axis_x = materials.add(StandardMaterial {
        base_color: Color::srgba(1.0, 0.48, 0.42, 0.45),
        emissive: LinearRgba::rgb(0.22, 0.04, 0.03),
        unlit: true,
        alpha_mode: AlphaMode::Blend,
        ..default()
    });
    let current_axis_y = materials.add(StandardMaterial {
        base_color: Color::srgba(0.5, 1.0, 0.58, 0.45),
        emissive: LinearRgba::rgb(0.04, 0.2, 0.04),
        unlit: true,
        alpha_mode: AlphaMode::Blend,
        ..default()
    });
    let current_axis_z = materials.add(StandardMaterial {
        base_color: Color::srgba(0.56, 0.72, 1.0, 0.45),
        emissive: LinearRgba::rgb(0.04, 0.06, 0.22),
        unlit: true,
        alpha_mode: AlphaMode::Blend,
        ..default()
    });
    let reference_origin = materials.add(StandardMaterial {
        base_color: Color::srgb(0.92, 0.96, 1.0),
        emissive: LinearRgba::rgb(0.28, 0.3, 0.34),
        unlit: true,
        ..default()
    });
    spawn_stars(&mut commands, &mut meshes, star);
    spawn_target_frame(
        &mut commands,
        &mut meshes,
        axis_x,
        axis_y,
        axis_z,
        reference_origin.clone(),
    );
    spawn_current_frame(
        &mut commands,
        &mut meshes,
        current_axis_x,
        current_axis_y,
        current_axis_z,
        reference_origin,
    );

    let lander = commands
        .spawn((
            LanderRoot,
            Transform::from_translation(DEMO_CENTER).with_rotation(current_scenario().initial),
            Visibility::default(),
        ))
        .id();

    commands.entity(lander).with_children(|parent| {
        spawn_body(
            parent,
            &mut meshes,
            metal.clone(),
            gold.clone(),
            dark.clone(),
            white.clone(),
        );
        spawn_landing_gear(
            parent,
            &mut meshes,
            metal.clone(),
            foil.clone(),
            dark.clone(),
        );
        spawn_antennas(parent, &mut meshes, metal.clone(), dark.clone());
    });

    commands.spawn((
        Text::new(status_text(
            current_scenario(),
            false,
            0.0,
            0.0,
            ATTITUDE_KP,
            ControlLaw::ScaledQuaternion,
        )),
        TextFont::from_font_size(15.0),
        TextColor(Color::srgb(0.92, 0.96, 1.0)),
        Node {
            position_type: PositionType::Absolute,
            left: px(18.0),
            right: px(18.0),
            top: px(12.0),
            max_width: px(1240.0),
            ..default()
        },
        StatusText,
    ));
}

fn spawn_body(
    parent: &mut ChildSpawnerCommands,
    meshes: &mut ResMut<Assets<Mesh>>,
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
    meshes: &mut ResMut<Assets<Mesh>>,
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
    meshes: &mut ResMut<Assets<Mesh>>,
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

fn spawn_stars(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
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

fn spawn_target_frame(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    axis_x: Handle<StandardMaterial>,
    axis_y: Handle<StandardMaterial>,
    axis_z: Handle<StandardMaterial>,
    origin_material: Handle<StandardMaterial>,
) {
    let origin = TARGET_FRAME_CENTER;
    let target = target_attitude();

    spawn_reference_sphere(commands, meshes, origin_material, origin, 0.065);
    spawn_axis_label(
        commands,
        "target q_d + current q",
        origin + Vec3::Y * 0.92,
        Color::srgb(0.92, 0.96, 1.0),
    );

    spawn_arrow(
        commands,
        meshes,
        axis_x.clone(),
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
        axis_y.clone(),
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
        axis_z.clone(),
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

fn spawn_current_frame(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    axis_x: Handle<StandardMaterial>,
    axis_y: Handle<StandardMaterial>,
    axis_z: Handle<StandardMaterial>,
    origin_material: Handle<StandardMaterial>,
) {
    let current = current_scenario().initial;
    let root = commands
        .spawn((
            CurrentFrameRoot,
            Transform::from_translation(TARGET_FRAME_CENTER).with_rotation(current),
            Visibility::default(),
        ))
        .id();

    commands.entity(root).with_children(|parent| {
        parent.spawn((
            Mesh3d(meshes.add(Sphere::new(0.045).mesh().uv(12, 6))),
            MeshMaterial3d(origin_material),
            Transform::default(),
            NotShadowCaster,
        ));
        spawn_local_arrow(
            parent,
            meshes,
            axis_x,
            Vec3::ZERO,
            Vec3::X * TARGET_FRAME_AXIS_LENGTH,
            AxisStyle::current_frame(),
        );
        spawn_local_arrow(
            parent,
            meshes,
            axis_y,
            Vec3::ZERO,
            Vec3::Y * TARGET_FRAME_AXIS_LENGTH,
            AxisStyle::current_frame(),
        );
        spawn_local_arrow(
            parent,
            meshes,
            axis_z,
            Vec3::ZERO,
            Vec3::Z * TARGET_FRAME_AXIS_LENGTH,
            AxisStyle::current_frame(),
        );
    });
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
    meshes: &mut ResMut<Assets<Mesh>>,
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
    meshes: &mut ResMut<Assets<Mesh>>,
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
    meshes: &mut ResMut<Assets<Mesh>>,
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

impl AttitudeLog {
    fn new(path: &str) -> Self {
        create_dir_all("logs").expect("failed to create logs directory");
        let mut file = File::create(path).expect("failed to create attitude log");
        writeln!(
            file,
            "time_s,qe0,qev_norm,error_angle_rad,omega_norm,omega_x,omega_y,omega_z"
        )
        .expect("failed to write attitude log header");

        Self {
            path: path.to_string(),
            file,
        }
    }

    fn reset(&mut self) {
        *self = Self::new(&self.path);
    }

    fn write_sample(&mut self, sample: AttitudeSample) {
        writeln!(
            self.file,
            "{:.4},{:.8},{:.8},{:.8},{:.8},{:.8},{:.8},{:.8}",
            sample.time_s,
            sample.qe0,
            sample.qev_norm,
            sample.error_angle_rad,
            sample.omega.length(),
            sample.omega.x,
            sample.omega.y,
            sample.omega.z
        )
        .expect("failed to write attitude log sample");
    }
}

fn run_headless_attitude_log(path: &str, duration_secs: f32, dt: f32) {
    let target = target_attitude();
    let mut current = current_scenario().initial;
    let mut log = AttitudeLog::new(path);
    let mut elapsed_secs = 0.0;
    let mut next_log_secs = 0.0;

    while elapsed_secs <= duration_secs {
        let (omega, mut sample) =
            attitude_command(target, current, ATTITUDE_KP, ControlLaw::ScaledQuaternion);

        if elapsed_secs >= next_log_secs {
            sample.time_s = elapsed_secs;
            log.write_sample(sample);
            next_log_secs += LOG_INTERVAL_SECS;
        }

        current = integrate_attitude(current, omega, dt);
        elapsed_secs += dt;
    }
}

fn reset_demo_run(
    scenario_index: usize,
    controller: &mut AttitudeController,
    log: &mut AttitudeLog,
    query: &mut Query<&mut Transform, With<LanderRoot>>,
) {
    let scenario = scenario_at(scenario_index);
    controller.scenario_index = scenario_index;
    controller.elapsed_secs = 0.0;
    controller.next_log_secs = 0.0;
    log.reset();

    for mut transform in query {
        transform.translation = DEMO_CENTER;
        transform.rotation = scenario.initial;
    }
}

fn handle_attitude_demo_input(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut controller: ResMut<AttitudeController>,
    mut log: ResMut<AttitudeLog>,
    mut query: Query<&mut Transform, With<LanderRoot>>,
) {
    let mut selected_scenario = None;

    if keyboard.just_pressed(KeyCode::Digit1) {
        selected_scenario = Some(0);
    } else if keyboard.just_pressed(KeyCode::Digit2) {
        selected_scenario = Some(1);
    } else if keyboard.just_pressed(KeyCode::Digit3) {
        selected_scenario = Some(2);
    } else if keyboard.just_pressed(KeyCode::Space) || keyboard.just_pressed(KeyCode::KeyR) {
        selected_scenario = Some(controller.scenario_index);
    }

    if keyboard.just_pressed(KeyCode::KeyP) {
        controller.paused = !controller.paused;
    }

    if keyboard.just_pressed(KeyCode::KeyC) {
        controller.control_law = controller.control_law.toggled();
        selected_scenario = Some(controller.scenario_index);
    }

    if let Some(index) = selected_scenario {
        reset_demo_run(index, &mut controller, &mut log, &mut query);
    }
}

fn control_lander_attitude(
    time: Res<Time>,
    mut controller: ResMut<AttitudeController>,
    mut log: ResMut<AttitudeLog>,
    mut query: Query<&mut Transform, With<LanderRoot>>,
) {
    if controller.paused {
        return;
    }

    let dt = time.delta_secs();
    controller.elapsed_secs += dt;

    for mut transform in &mut query {
        let (omega, mut sample) = attitude_command(
            controller.target,
            transform.rotation,
            controller.kp,
            controller.control_law,
        );
        transform.rotation = integrate_attitude(transform.rotation, omega, dt);

        if controller.elapsed_secs >= controller.next_log_secs {
            sample.time_s = controller.elapsed_secs;
            log.write_sample(sample);
            controller.next_log_secs += LOG_INTERVAL_SECS;
        }
    }
}

fn sync_current_frame(
    lander_query: Query<&Transform, With<LanderRoot>>,
    mut frame_query: Query<&mut Transform, (With<CurrentFrameRoot>, Without<LanderRoot>)>,
) {
    let Ok(lander_transform) = lander_query.single() else {
        return;
    };
    let Ok(mut frame_transform) = frame_query.single_mut() else {
        return;
    };

    frame_transform.translation = TARGET_FRAME_CENTER;
    frame_transform.rotation = lander_transform.rotation;
}

fn status_text(
    scenario: AttitudeScenario,
    paused: bool,
    elapsed_secs: f32,
    error_angle_rad: f32,
    kp: f32,
    control_law: ControlLaw,
) -> String {
    let status = if paused { "PAUSED" } else { "CONVERGING" };
    let (desired_axis, desired_angle) = desired_axis_angle();
    format!(
        "{status} | Mode: {mode} | Law: {formula} | Scenario: {scenario}\nTarget axis [{axis_x:.2}, {axis_y:.2}, {axis_z:.2}], {target_deg:.1} deg | Error {error_deg:.2} deg | t {elapsed_secs:.1}s | kp {kp:.2} | Space/R reset | 1/2/3 start | C switch law | P pause",
        scenario = scenario.name,
        mode = control_law.name(),
        formula = control_law.hud_formula(),
        error_deg = error_angle_rad.to_degrees(),
        axis_x = desired_axis.x,
        axis_y = desired_axis.y,
        axis_z = desired_axis.z,
        target_deg = desired_angle.to_degrees(),
    )
}

fn update_status_text(
    controller: Res<AttitudeController>,
    lander_query: Query<&Transform, With<LanderRoot>>,
    mut text_query: Query<&mut Text, With<StatusText>>,
) {
    let Ok(transform) = lander_query.single() else {
        return;
    };
    let Ok(mut text) = text_query.single_mut() else {
        return;
    };

    let (_, sample) = attitude_command(
        controller.target,
        transform.rotation,
        controller.kp,
        controller.control_law,
    );
    *text = Text::new(status_text(
        scenario_at(controller.scenario_index),
        controller.paused,
        controller.elapsed_secs,
        sample.error_angle_rad,
        controller.kp,
        controller.control_law,
    ));
}
