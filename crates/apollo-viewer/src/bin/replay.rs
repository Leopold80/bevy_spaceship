use apollo_viewer::model::{create_lander_materials, dquat, dvec3, spawn_lander};
use apollo_viewer::scene::{
    create_star_material, default_window, insert_default_lighting, spawn_camera_and_light,
    spawn_stars,
};
use apollo_viewer::trajectory::Trajectory;
use bevy::prelude::*;
use std::ffi::OsString;
use std::path::PathBuf;

#[derive(Component)]
struct ApolloVisualRoot;

#[derive(Component)]
struct StatusText;

#[derive(Resource)]
struct ReplayState {
    trajectory: Trajectory,
    time_seconds: f64,
    playing: bool,
    playback_rate: f64,
}

fn main() {
    let arguments = replay_arguments().unwrap_or_else(|message| {
        eprintln!("{message}");
        std::process::exit(2);
    });
    let trajectory = Trajectory::load(&arguments.path).unwrap_or_else(|error| {
        eprintln!("无法读取轨迹 {}: {error}", arguments.path.display());
        std::process::exit(1);
    });
    if arguments.validate_only {
        let header = trajectory.header();
        println!(
            "轨迹有效: format={} version={} model={} frames={} duration={:.6}s",
            header.format,
            header.version,
            header.model,
            trajectory.frames().len(),
            trajectory.duration_seconds(),
        );
        return;
    }
    let start_time = trajectory.start_time_seconds();

    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(default_window("Apollo Trajectory Replay")));
    insert_default_lighting(&mut app);
    app.insert_resource(ReplayState {
        trajectory,
        time_seconds: start_time,
        playing: true,
        playback_rate: 1.0,
    });
    app.add_systems(Startup, setup)
        .add_systems(
            Update,
            (handle_input, advance_replay, sync_visual, update_status).chain(),
        )
        .run();
}

struct ReplayArguments {
    path: PathBuf,
    validate_only: bool,
}

fn replay_arguments() -> Result<ReplayArguments, &'static str> {
    parse_arguments(std::env::args_os().skip(1))
}

fn parse_arguments(
    arguments: impl IntoIterator<Item = OsString>,
) -> Result<ReplayArguments, &'static str> {
    let mut arguments = arguments.into_iter();
    let Some(first) = arguments.next() else {
        return Err("用法: apollo-replay [--validate-only] <trajectory.jsonl>");
    };
    let (validate_only, path) = if first == "--validate-only" {
        let Some(path) = arguments.next() else {
            return Err("--validate-only 后必须提供轨迹文件路径");
        };
        (true, path)
    } else {
        (false, first)
    };
    if arguments.next().is_some() {
        return Err("apollo-replay 只接受一个轨迹文件路径");
    }
    Ok(ReplayArguments {
        path: path.into(),
        validate_only,
    })
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
    let lander = spawn_lander(
        &mut commands,
        &mut meshes,
        &lander_materials,
        Transform::default(),
    );
    commands.entity(lander).insert(ApolloVisualRoot);

    commands.spawn((
        Text::new(""),
        TextFont::from_font_size(16.0),
        TextColor(Color::srgb(0.92, 0.96, 1.0)),
        Node {
            position_type: PositionType::Absolute,
            left: px(18.0),
            top: px(12.0),
            ..default()
        },
        StatusText,
    ));
}

fn handle_input(keyboard: Res<ButtonInput<KeyCode>>, mut replay: ResMut<ReplayState>) {
    if keyboard.just_pressed(KeyCode::Space) {
        replay.playing = !replay.playing;
    }
    if keyboard.just_pressed(KeyCode::KeyR) {
        replay.time_seconds = replay.trajectory.start_time_seconds();
    }
    if keyboard.just_pressed(KeyCode::ArrowUp) {
        replay.playback_rate = (replay.playback_rate * 2.0).min(16.0);
    }
    if keyboard.just_pressed(KeyCode::ArrowDown) {
        replay.playback_rate = (replay.playback_rate * 0.5).max(0.125);
    }
    if keyboard.just_pressed(KeyCode::ArrowRight) {
        replay.playing = false;
        let timing = replay.trajectory.header().timing;
        let end_control_tick = replay
            .trajectory
            .frames()
            .last()
            .map(|frame| frame.snapshot.control_tick)
            .unwrap_or(0);
        replay.time_seconds =
            stepped_control_time(timing, replay.time_seconds, end_control_tick, true);
    }
    if keyboard.just_pressed(KeyCode::ArrowLeft) {
        replay.playing = false;
        let timing = replay.trajectory.header().timing;
        let end_control_tick = replay
            .trajectory
            .frames()
            .last()
            .map(|frame| frame.snapshot.control_tick)
            .unwrap_or(0);
        replay.time_seconds =
            stepped_control_time(timing, replay.time_seconds, end_control_tick, false);
    }
}

fn stepped_control_time(
    timing: apollo_core::SimulationTiming,
    current_time_seconds: f64,
    end_control_tick: u64,
    forward: bool,
) -> f64 {
    let current_control_tick = (current_time_seconds / timing.control_step_seconds())
        .round()
        .clamp(0.0, end_control_tick as f64) as u64;
    let target_control_tick = if forward {
        current_control_tick.saturating_add(1).min(end_control_tick)
    } else {
        current_control_tick.saturating_sub(1)
    };
    let target_physics_tick = timing
        .physics_ticks_for_control_ticks(target_control_tick)
        .expect("validated trajectory tick must fit in u64");
    timing.sim_time_seconds(target_physics_tick)
}

fn advance_replay(time: Res<Time>, mut replay: ResMut<ReplayState>) {
    if !replay.playing {
        return;
    }
    replay.time_seconds += time.delta_secs_f64() * replay.playback_rate;
    if replay.time_seconds >= replay.trajectory.end_time_seconds() {
        replay.time_seconds = replay.trajectory.end_time_seconds();
        replay.playing = false;
    }
}

fn sync_visual(
    replay: Res<ReplayState>,
    mut lander: Query<&mut Transform, With<ApolloVisualRoot>>,
) {
    let Ok(mut transform) = lander.single_mut() else {
        return;
    };
    let sample = replay.trajectory.sample(replay.time_seconds);
    transform.translation = dvec3(sample.state.position_body_origin_world_m);
    transform.rotation = dquat(sample.state.body_to_world);
}

fn update_status(replay: Res<ReplayState>, mut text: Query<&mut Text, With<StatusText>>) {
    let Ok(mut text) = text.single_mut() else {
        return;
    };
    let sample = replay.trajectory.sample(replay.time_seconds);
    let status = if replay.playing { "PLAYING" } else { "PAUSED" };
    let wrench_status = match sample.applied_action {
        Some(action) => format!(
            "|F| {:.1} N | |τ| {:.1} N·m",
            action.force_body_n.length(),
            action.torque_about_com_body_nm.length(),
        ),
        None => "|F| unknown | |τ| unknown".to_owned(),
    };
    *text = Text::new(format!(
        "{status} | t {time:.3} / {end:.3} s | speed {rate:.3}x | |ω| {omega:.4} rad/s | {wrench_status}\nSpace play/pause | R restart | ←/→ single control tick | ↑/↓ speed",
        time = sample.sim_time_seconds,
        end = replay.trajectory.end_time_seconds(),
        rate = replay.playback_rate,
        omega = sample.state.angular_velocity_body_radps.length(),
    ));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_replay_and_headless_validation_modes() {
        let replay = parse_arguments([OsString::from("run.jsonl")]).unwrap();
        assert_eq!(replay.path, PathBuf::from("run.jsonl"));
        assert!(!replay.validate_only);

        let validation = parse_arguments([
            OsString::from("--validate-only"),
            OsString::from("run.jsonl"),
        ])
        .unwrap();
        assert_eq!(validation.path, PathBuf::from("run.jsonl"));
        assert!(validation.validate_only);
    }

    #[test]
    fn single_step_rebuilds_time_from_integer_ticks() {
        let timing = apollo_core::SimulationTiming::APOLLO;
        let tick_one = stepped_control_time(timing, 0.0, 10, true);
        let tick_two = stepped_control_time(timing, tick_one, 10, true);
        let tick_three = stepped_control_time(timing, tick_two, 10, true);
        let expected_tick_three = timing.sim_time_seconds(
            timing
                .physics_ticks_for_control_ticks(3)
                .expect("small test tick fits"),
        );

        assert_eq!(tick_three, expected_tick_three);
        assert_eq!(
            stepped_control_time(timing, tick_three, 10, false),
            tick_two
        );
    }
}
