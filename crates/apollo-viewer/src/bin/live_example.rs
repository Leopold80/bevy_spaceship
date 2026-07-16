//! 例程：在应用层手写闭环，并把最新 plant 快照交给 Bevy。
//!
//! 这里的线程、实时节拍、暂停和控制器都只属于这个可执行例程；
//! `apollo-mujoco` plant 仍然是同步、无 sleep、无控制器的 `step()` API。

use apollo_mujoco::{ApolloPlantFactory, ApolloState, BodyWrench, PlantSnapshot};
use apollo_viewer::model::{create_lander_materials, dquat, dvec3, spawn_lander};
use apollo_viewer::scene::{
    create_reference_frame_materials, create_star_material, default_window,
    insert_default_lighting, spawn_attitude_frame_legend, spawn_camera_and_light,
    spawn_current_attitude_frame, spawn_desired_attitude_axis_labels, spawn_desired_attitude_frame,
    spawn_stars,
};
use bevy::prelude::*;
use glam::{DQuat, DVec3, EulerRot};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

#[derive(Component)]
struct ApolloVisualRoot;

#[derive(Component)]
struct BodyFrameRoot;

#[derive(Component)]
struct StatusText;

#[derive(Resource)]
struct LiveExampleState {
    latest: Arc<Mutex<PlantSnapshot>>,
    running: Arc<AtomicBool>,
    paused: Arc<AtomicBool>,
    reset_requested: Arc<AtomicBool>,
    step_once_requested: Arc<AtomicBool>,
    worker_error: Arc<Mutex<Option<String>>>,
    worker: Option<JoinHandle<()>>,
}

impl Drop for LiveExampleState {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Release);
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}

fn main() {
    let live_state = start_live_example().unwrap_or_else(|error| {
        eprintln!("无法启动 Apollo live example: {error}");
        std::process::exit(1);
    });

    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(default_window("Apollo Plant API Live Example")));
    insert_default_lighting(&mut app);
    app.insert_resource(live_state)
        .add_systems(Startup, setup)
        .add_systems(Update, (handle_input, sync_visual, update_status).chain())
        .run();
}

fn start_live_example() -> Result<LiveExampleState, apollo_mujoco::PlantError> {
    let initial_state = challenge_initial_state();
    let factory = ApolloPlantFactory::apollo_touchdown()?;
    let mut plant = factory.spawn(initial_state)?;
    let timing = plant.timing();
    let tick_duration = Duration::from_secs_f64(timing.control_step_seconds());

    let latest = Arc::new(Mutex::new(plant.snapshot()));
    let running = Arc::new(AtomicBool::new(true));
    // 窗口和 GPU 初始化完成前保持暂停，让用户从显式初态开始观察。
    let paused = Arc::new(AtomicBool::new(true));
    let reset_requested = Arc::new(AtomicBool::new(false));
    let step_once_requested = Arc::new(AtomicBool::new(false));
    let worker_error = Arc::new(Mutex::new(None));

    let worker_latest = Arc::clone(&latest);
    let worker_running = Arc::clone(&running);
    let worker_paused = Arc::clone(&paused);
    let worker_reset = Arc::clone(&reset_requested);
    let worker_step_once = Arc::clone(&step_once_requested);
    let worker_error_slot = Arc::clone(&worker_error);

    // 闭环就在例程创建的普通线程中：读状态、算动作、调用一步、发布快照。
    // 这个循环不是 plant API，也没有被包装成可复用 runner。
    let worker = thread::spawn(move || {
        while worker_running.load(Ordering::Acquire) {
            if worker_reset.swap(false, Ordering::AcqRel) {
                match plant.reset(initial_state) {
                    Ok(snapshot) => publish_snapshot(&worker_latest, snapshot),
                    Err(error) => {
                        publish_worker_error(&worker_error_slot, error.to_string());
                        worker_running.store(false, Ordering::Release);
                        break;
                    }
                }
                // reset 快照必须至少保持到调用方下一次明确继续或单步。
                continue;
            }

            let paused = worker_paused.load(Ordering::Acquire);
            let step_once = worker_step_once.swap(false, Ordering::AcqRel);
            if paused && !step_once {
                thread::sleep(Duration::from_millis(2));
                continue;
            }

            let started = Instant::now();
            let snapshot = plant.snapshot();
            let action = attitude_pd(snapshot.state);
            match plant.step(action) {
                Ok(step) => publish_snapshot(&worker_latest, step.snapshot),
                Err(error) => {
                    publish_worker_error(&worker_error_slot, error.to_string());
                    worker_running.store(false, Ordering::Release);
                    break;
                }
            }

            if !paused && let Some(remaining) = tick_duration.checked_sub(started.elapsed()) {
                thread::sleep(remaining);
            }
        }
    });

    Ok(LiveExampleState {
        latest,
        running,
        paused,
        reset_requested,
        step_once_requested,
        worker_error,
        worker: Some(worker),
    })
}

fn challenge_initial_state() -> ApolloState {
    ApolloState {
        body_to_world: DQuat::from_euler(EulerRot::XYZ, -0.85, 0.55, 1.25),
        angular_velocity_body_radps: DVec3::new(0.55, -0.35, 0.25),
        ..ApolloState::ZERO
    }
}

fn desired_attitude() -> DQuat {
    DQuat::IDENTITY
}

/// 例程私有的单位姿态 PD；它不是 `apollo-mujoco` 的一部分。
fn attitude_pd(state: ApolloState) -> BodyWrench {
    let mut error_body = desired_attitude() * state.body_to_world.conjugate();
    if error_body.w < 0.0 {
        error_body = -error_body;
    }
    let rotation_error_body = 2.0 * DVec3::new(error_body.x, error_body.y, error_body.z);
    let unconstrained_torque =
        25_000.0 * rotation_error_body - 18_000.0 * state.angular_velocity_body_radps;

    BodyWrench {
        force_body_n: DVec3::ZERO,
        torque_about_com_body_nm: clamp_length(unconstrained_torque, 52_000.0),
    }
}

fn clamp_length(value: DVec3, maximum: f64) -> DVec3 {
    if value.length_squared() <= maximum * maximum {
        value
    } else {
        value.normalize() * maximum
    }
}

fn publish_snapshot(target: &Mutex<PlantSnapshot>, snapshot: PlantSnapshot) {
    *target.lock().expect("latest snapshot mutex poisoned") = snapshot;
}

fn publish_worker_error(target: &Mutex<Option<String>>, message: String) {
    *target.lock().expect("worker error mutex poisoned") = Some(message);
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    live: Res<LiveExampleState>,
) {
    spawn_camera_and_light(&mut commands);
    let lander_materials = create_lander_materials(&mut materials);
    let star = create_star_material(&mut materials);
    let frame_materials = create_reference_frame_materials(&mut materials);
    spawn_stars(&mut commands, &mut meshes, star);
    spawn_attitude_frame_legend(&mut commands, true);
    spawn_desired_attitude_frame(
        &mut commands,
        &mut meshes,
        &frame_materials,
        dquat(desired_attitude()),
        Visibility::Inherited,
    );
    spawn_desired_attitude_axis_labels(
        &mut commands,
        dquat(desired_attitude()),
        Visibility::Inherited,
    );
    let initial_snapshot = *live.latest.lock().expect("latest snapshot mutex poisoned");
    let body_frame = spawn_current_attitude_frame(
        &mut commands,
        &mut meshes,
        &frame_materials,
        dquat(initial_snapshot.state.body_to_world),
    );
    commands.entity(body_frame).insert(BodyFrameRoot);
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
            right: px(18.0),
            top: px(12.0),
            max_width: px(1240.0),
            ..default()
        },
        StatusText,
    ));
}

fn handle_input(keyboard: Res<ButtonInput<KeyCode>>, live: Res<LiveExampleState>) {
    if keyboard.just_pressed(KeyCode::Space) {
        let was_paused = live.paused.fetch_xor(true, Ordering::AcqRel);
        if was_paused {
            live.step_once_requested.store(false, Ordering::Release);
        }
    }
    if keyboard.just_pressed(KeyCode::KeyR) {
        live.paused.store(true, Ordering::Release);
        live.step_once_requested.store(false, Ordering::Release);
        live.reset_requested.store(true, Ordering::Release);
    }
    if keyboard.just_pressed(KeyCode::ArrowRight) && live.paused.load(Ordering::Acquire) {
        live.step_once_requested.store(true, Ordering::Release);
    }
}

fn sync_visual(
    live: Res<LiveExampleState>,
    mut lander: Query<&mut Transform, (With<ApolloVisualRoot>, Without<BodyFrameRoot>)>,
    mut body_frame: Query<&mut Transform, (With<BodyFrameRoot>, Without<ApolloVisualRoot>)>,
) {
    let Ok(mut transform) = lander.single_mut() else {
        return;
    };
    let snapshot = *live.latest.lock().expect("latest snapshot mutex poisoned");
    transform.translation = dvec3(snapshot.state.position_body_origin_world_m);
    transform.rotation = dquat(snapshot.state.body_to_world);
    if let Ok(mut frame_transform) = body_frame.single_mut() {
        frame_transform.rotation = transform.rotation;
    }
}

fn update_status(live: Res<LiveExampleState>, mut text: Query<&mut Text, With<StatusText>>) {
    let Ok(mut text) = text.single_mut() else {
        return;
    };
    let snapshot = *live.latest.lock().expect("latest snapshot mutex poisoned");
    let attitude_error_deg = 2.0
        * snapshot
            .state
            .body_to_world
            .w
            .abs()
            .clamp(0.0, 1.0)
            .acos()
            .to_degrees();
    let mode = if live.running.load(Ordering::Acquire) {
        if live.paused.load(Ordering::Acquire) {
            "PAUSED"
        } else {
            "RUNNING"
        }
    } else {
        "STOPPED"
    };
    let worker_error = live
        .worker_error
        .lock()
        .expect("worker error mutex poisoned")
        .clone()
        .unwrap_or_else(|| "none".to_owned());

    *text = Text::new(format!(
        "{mode} | control tick {tick} | attitude error {error:.3} deg | |omega| {omega:.4} rad/s\nSpace pause/resume | R reset | Right Arrow one tick while paused\nworker error: {worker_error}",
        tick = snapshot.control_tick,
        error = attitude_error_deg,
        omega = snapshot.state.angular_velocity_body_radps.length(),
    ));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn example_controller_is_zero_at_the_target() {
        assert_eq!(attitude_pd(ApolloState::ZERO), BodyWrench::ZERO);
    }

    #[test]
    fn example_controller_opposes_a_positive_body_y_error() {
        let state = ApolloState {
            body_to_world: DQuat::from_rotation_y(0.25),
            ..ApolloState::ZERO
        };
        assert!(attitude_pd(state).torque_about_com_body_nm.y < 0.0);
    }
}
