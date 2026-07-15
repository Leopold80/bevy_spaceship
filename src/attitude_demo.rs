use crate::attitude_control::{
    ATTITUDE_KP, AttitudeScenario, attitude_command, current_scenario, desired_axis_angle,
    integrate_attitude, scenario_at, target_attitude,
};
use crate::attitude_log::{AttitudeLog, LOG_INTERVAL_SECS, run_headless_attitude_log};
use crate::spacecraft_model::{create_lander_materials, spawn_lander};
use crate::visualization::{
    TARGET_FRAME_CENTER, create_reference_frame_materials, create_star_material,
    spawn_current_frame, spawn_default_camera_and_light, spawn_stars, spawn_target_frame,
};
use bevy::prelude::*;

pub const LOG_PATH: &str = "logs/attitude_kinematics.csv";
const DEMO_CENTER: Vec3 = Vec3::new(0.0, 0.85, 0.0);

#[derive(Component)]
struct LanderRoot;

#[derive(Component)]
struct CurrentFrameRoot;

#[derive(Resource)]
struct AttitudeController {
    target: Quat,
    kp: f32,
    scenario_index: usize,
    paused: bool,
    elapsed_secs: f32,
    next_log_secs: f32,
}

#[derive(Component)]
struct StatusText;

pub fn run() {
    if std::env::args().any(|arg| arg == "--headless-log") {
        run_headless_attitude_log(LOG_PATH, 8.0, 1.0 / 60.0);
        return;
    }

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Bevy Spacecraft Attitude Demo".into(),
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
    spawn_default_camera_and_light(&mut commands);

    let lander_materials = create_lander_materials(&mut materials);
    let star = create_star_material(&mut materials);
    let frame_materials = create_reference_frame_materials(&mut materials);

    spawn_stars(&mut commands, &mut meshes, star);
    spawn_target_frame(
        &mut commands,
        &mut meshes,
        &frame_materials,
        target_attitude(),
    );
    let current_frame = spawn_current_frame(
        &mut commands,
        &mut meshes,
        &frame_materials,
        current_scenario().initial,
    );
    commands.entity(current_frame).insert(CurrentFrameRoot);

    let lander = spawn_lander(
        &mut commands,
        &mut meshes,
        &lander_materials,
        Transform::from_translation(DEMO_CENTER).with_rotation(current_scenario().initial),
    );
    commands.entity(lander).insert(LanderRoot);

    commands.spawn((
        Text::new(status_text(
            current_scenario(),
            false,
            0.0,
            0.0,
            ATTITUDE_KP,
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
        let (omega, mut sample) =
            attitude_command(controller.target, transform.rotation, controller.kp);
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
) -> String {
    let status = if paused { "PAUSED" } else { "CONVERGING" };
    let (desired_axis, desired_angle) = desired_axis_angle();
    format!(
        "{status} | Mode: fixed-gain q_ev feedback | Law: qe=qd^-1*q, qe0>=0, wc=-kp*qev | Scenario: {scenario}\nTarget axis [{axis_x:.2}, {axis_y:.2}, {axis_z:.2}], {target_deg:.1} deg | Error {error_deg:.2} deg | t {elapsed_secs:.1}s | kp {kp:.2} | Space/R reset | 1/2/3 start | P pause",
        scenario = scenario.name,
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

    let (_, sample) = attitude_command(controller.target, transform.rotation, controller.kp);
    *text = Text::new(status_text(
        scenario_at(controller.scenario_index),
        controller.paused,
        controller.elapsed_secs,
        sample.error_angle_rad,
        controller.kp,
    ));
}
