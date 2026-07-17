//! Apollo 11 LM 推进系统的直接交互例程。
//!
//! 键盘只生成 16 路 RCS 门控时间和 DPS 离散工作档位；真实的阀门瞬态、
//! 点力、力矩和刚体运动仍全部由 `ApolloPropulsionPlant::step` 决定。

use apollo_core::{
    ApolloPropulsionSpec, ApolloState, DpsCommand, DpsMode, PlantSnapshot, PropulsionCommand,
    PropulsionStep, RcsCommand, SimulationTiming,
};
use apollo_mujoco::{ApolloPropulsionPlantFactory, PlantError};
use apollo_viewer::model::{
    DpsEngineVisual, DpsPlumeVisual, RCS_DEFLECTED_PLUME_RADIUS_SCALE, RCS_FREE_PLUME_LENGTH_M,
    RCS_PLUME_EXIT_OFFSET_M, RcsDeflectedPlumeVisual, RcsPlumePath, RcsPlumeVisual,
    create_lander_materials, dquat, dvec3, spawn_lander,
};
use apollo_viewer::propulsion_controls::{
    DPS_GIMBAL_STEP_RAD, DemoDpsRequest, DemoMode, DemoPropulsionRequest, DpsControlLimits,
    PropulsionDemoControls,
};
use apollo_viewer::scene::{
    create_star_material, default_window, insert_default_lighting, spawn_camera_and_light,
    spawn_stars,
};
use apollo_viewer::torque_couples::{AxisTorqueSet, ThrusterCandidate, select_axis_torque_sets};
use bevy::prelude::*;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

const DPS_PLUME_EXIT_Y: f32 = -1.75;
const DPS_PLUME_HALF_HEIGHT_M: f32 = 1.20;

#[derive(Component)]
struct ApolloVisualRoot;

#[derive(Component)]
struct StatusText;

#[derive(Clone, Copy, Debug)]
struct DemoTelemetry {
    snapshot: PlantSnapshot,
    last_step: Option<PropulsionStep>,
}

#[derive(Clone, Copy, Debug, Default)]
struct SharedCommandIntent {
    controls: PropulsionDemoControls,
    pulse_queued: bool,
    continuous_requested: bool,
}

#[derive(Resource, Default)]
struct InputLatch {
    /// “全关”后必须先松开 F，避免仍按住的连续点火在下一帧自动恢复。
    continuous_blocked_until_release: bool,
}

#[derive(Resource)]
struct PropulsionDemoState {
    latest: Arc<Mutex<DemoTelemetry>>,
    command_intent: Arc<Mutex<SharedCommandIntent>>,
    running: Arc<AtomicBool>,
    paused: Arc<AtomicBool>,
    reset_requested: Arc<AtomicBool>,
    step_once_requested: Arc<AtomicBool>,
    worker_error: Arc<Mutex<Option<String>>>,
    timing: SimulationTiming,
    propulsion_spec: ApolloPropulsionSpec,
    torque_sets: [AxisTorqueSet; 6],
    worker: Option<JoinHandle<()>>,
}

impl Drop for PropulsionDemoState {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Release);
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}

fn main() {
    let demo = start_propulsion_demo().unwrap_or_else(|error| {
        eprintln!("无法启动 Apollo 推进交互例程: {error}");
        std::process::exit(1);
    });

    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(default_window("Apollo 11 LM Propulsion Demo")));
    insert_default_lighting(&mut app);
    app.insert_resource(demo)
        .init_resource::<InputLatch>()
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                handle_input,
                sync_lander_pose,
                sync_rcs_plumes,
                sync_rcs_deflected_plumes,
                sync_dps_visuals,
                update_status,
            )
                .chain(),
        )
        .run();
}

fn start_propulsion_demo() -> Result<PropulsionDemoState, String> {
    let initial_state = ApolloState::ZERO;
    let factory =
        ApolloPropulsionPlantFactory::apollo11_touchdown().map_err(display_plant_error)?;
    let propulsion_spec = factory.propulsion_spec();
    let model_spec = factory.model_spec();
    let timing = factory.timing();
    let control_step_ns =
        u64::try_from(timing.control_step_ns()).map_err(|_| "控制周期纳秒数超出 u64".to_owned())?;
    let limits = dps_limits(propulsion_spec);
    let torque_sets = torque_sets(propulsion_spec, model_spec.center_of_mass_body_m)?;
    let mut plant = factory.spawn(initial_state).map_err(display_plant_error)?;

    let latest = Arc::new(Mutex::new(DemoTelemetry {
        snapshot: plant.snapshot(),
        last_step: None,
    }));
    let command_intent = Arc::new(Mutex::new(SharedCommandIntent::default()));
    let running = Arc::new(AtomicBool::new(true));
    // GPU/窗口尚未准备好时不推进状态，打开后由用户显式继续或单步。
    let paused = Arc::new(AtomicBool::new(true));
    let reset_requested = Arc::new(AtomicBool::new(false));
    let step_once_requested = Arc::new(AtomicBool::new(false));
    let worker_error = Arc::new(Mutex::new(None));

    let worker_latest = Arc::clone(&latest);
    let worker_intent = Arc::clone(&command_intent);
    let worker_running = Arc::clone(&running);
    let worker_paused = Arc::clone(&paused);
    let worker_reset = Arc::clone(&reset_requested);
    let worker_step_once = Arc::clone(&step_once_requested);
    let worker_error_slot = Arc::clone(&worker_error);
    let worker_torque_sets = torque_sets.clone();
    let tick_duration = Duration::from_secs_f64(timing.control_step_seconds());

    // 普通线程只负责固定节拍地调用同步 plant；没有隐藏 runner 或分配器。
    let worker = thread::spawn(move || {
        while worker_running.load(Ordering::Acquire) {
            if worker_reset.swap(false, Ordering::AcqRel) {
                match plant.reset(initial_state) {
                    Ok(snapshot) => publish_telemetry(
                        &worker_latest,
                        DemoTelemetry {
                            snapshot,
                            last_step: None,
                        },
                    ),
                    Err(error) => {
                        stop_with_error(&worker_error_slot, &worker_running, error.to_string());
                        break;
                    }
                }
                continue;
            }

            let paused_now = worker_paused.load(Ordering::Acquire);
            let step_once = worker_step_once.swap(false, Ordering::AcqRel);
            if paused_now && !step_once {
                thread::sleep(Duration::from_millis(2));
                continue;
            }

            let started = Instant::now();
            let request = {
                let mut intent = worker_intent
                    .lock()
                    .expect("propulsion command intent mutex poisoned");
                let pulse_requested = std::mem::take(&mut intent.pulse_queued);
                intent.controls.build_request(
                    &worker_torque_sets,
                    pulse_requested,
                    intent.continuous_requested,
                    control_step_ns,
                    limits,
                )
            };
            match plant.step(to_core_command(request)) {
                Ok(step) => publish_telemetry(
                    &worker_latest,
                    DemoTelemetry {
                        snapshot: step.snapshot,
                        last_step: Some(step),
                    },
                ),
                Err(error) => {
                    stop_with_error(&worker_error_slot, &worker_running, error.to_string());
                    break;
                }
            }

            if !paused_now && let Some(remaining) = tick_duration.checked_sub(started.elapsed()) {
                thread::sleep(remaining);
            }
        }
    });

    Ok(PropulsionDemoState {
        latest,
        command_intent,
        running,
        paused,
        reset_requested,
        step_once_requested,
        worker_error,
        timing,
        propulsion_spec,
        torque_sets,
        worker: Some(worker),
    })
}

fn display_plant_error(error: PlantError) -> String {
    error.to_string()
}

fn dps_limits(spec: ApolloPropulsionSpec) -> DpsControlLimits {
    DpsControlLimits {
        variable_min_thrust_n: spec.dps.variable_min_thrust_n,
        variable_max_thrust_n: spec.dps.variable_max_thrust_n,
        full_thrust_n: spec.dps.full_thrust_n,
        maximum_gimbal_rad: spec.dps.maximum_gimbal_rad,
    }
}

fn torque_sets(
    spec: ApolloPropulsionSpec,
    center_of_mass_body_m: glam::DVec3,
) -> Result<[AxisTorqueSet; 6], String> {
    let candidates: Vec<_> = spec
        .rcs_thrusters
        .iter()
        .map(|thruster| ThrusterCandidate {
            stable_index: thruster.id.index(),
            position_body_m: thruster.position_body_m,
            force_direction_body: thruster.force_direction_body,
            maximum_thrust_n: thruster.steady_thrust_n,
        })
        .collect();
    select_axis_torque_sets(&candidates, center_of_mass_body_m).map_err(str::to_owned)
}

fn to_core_command(request: DemoPropulsionRequest) -> PropulsionCommand {
    let dps = match request.dps {
        DemoDpsRequest::Off => DpsCommand::Off,
        DemoDpsRequest::Variable {
            thrust_n,
            gimbal_x_rad,
            gimbal_z_rad,
        } => DpsCommand::Variable {
            thrust_n,
            gimbal_x_rad,
            gimbal_z_rad,
        },
        DemoDpsRequest::FullThrust {
            gimbal_x_rad,
            gimbal_z_rad,
        } => DpsCommand::FullThrust {
            gimbal_x_rad,
            gimbal_z_rad,
        },
    };
    PropulsionCommand {
        rcs: RcsCommand::from_on_times(request.rcs_on_time_ns),
        dps,
    }
}

fn publish_telemetry(target: &Mutex<DemoTelemetry>, telemetry: DemoTelemetry) {
    *target
        .lock()
        .expect("latest propulsion telemetry mutex poisoned") = telemetry;
}

fn stop_with_error(target: &Mutex<Option<String>>, running: &AtomicBool, message: String) {
    *target
        .lock()
        .expect("propulsion worker error mutex poisoned") = Some(message);
    running.store(false, Ordering::Release);
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
        TextFont::from_font_size(15.0),
        TextColor(Color::srgb(0.92, 0.96, 1.0)),
        Node {
            position_type: PositionType::Absolute,
            left: px(16.0),
            right: px(16.0),
            top: px(10.0),
            max_width: px(1240.0),
            ..default()
        },
        StatusText,
    ));
}

fn handle_input(
    keyboard: Res<ButtonInput<KeyCode>>,
    demo: Res<PropulsionDemoState>,
    mut latch: ResMut<InputLatch>,
) {
    let f_pressed = keyboard.pressed(KeyCode::KeyF);
    if !f_pressed {
        latch.continuous_blocked_until_release = false;
    }

    let mut intent = demo
        .command_intent
        .lock()
        .expect("propulsion command intent mutex poisoned");
    intent.continuous_requested = f_pressed && !latch.continuous_blocked_until_release;

    if keyboard.just_pressed(KeyCode::Tab) {
        intent.controls.cycle_mode();
    }
    if keyboard.just_pressed(KeyCode::BracketLeft) {
        intent.controls.select_previous();
    }
    if keyboard.just_pressed(KeyCode::BracketRight) {
        intent.controls.select_next();
    }
    if keyboard.just_pressed(KeyCode::Enter) && intent.controls.mode != DemoMode::Dps {
        intent.pulse_queued = true;
    }
    if keyboard.just_pressed(KeyCode::KeyI) {
        intent.controls.toggle_dps();
    }
    if keyboard.just_pressed(KeyCode::ArrowUp) {
        intent.controls.step_dps_thrust_up();
    }
    if keyboard.just_pressed(KeyCode::ArrowDown) {
        intent.controls.step_dps_thrust_down();
    }

    let maximum_gimbal_rad = demo.propulsion_spec.dps.maximum_gimbal_rad;
    if keyboard.just_pressed(KeyCode::KeyA) {
        intent
            .controls
            .adjust_gimbal(-DPS_GIMBAL_STEP_RAD, 0.0, maximum_gimbal_rad);
    }
    if keyboard.just_pressed(KeyCode::KeyD) {
        intent
            .controls
            .adjust_gimbal(DPS_GIMBAL_STEP_RAD, 0.0, maximum_gimbal_rad);
    }
    if keyboard.just_pressed(KeyCode::KeyW) {
        intent
            .controls
            .adjust_gimbal(0.0, DPS_GIMBAL_STEP_RAD, maximum_gimbal_rad);
    }
    if keyboard.just_pressed(KeyCode::KeyS) {
        intent
            .controls
            .adjust_gimbal(0.0, -DPS_GIMBAL_STEP_RAD, maximum_gimbal_rad);
    }

    if keyboard.just_pressed(KeyCode::Digit0) || keyboard.just_pressed(KeyCode::Backspace) {
        intent.controls.all_off();
        intent.pulse_queued = false;
        intent.continuous_requested = false;
        latch.continuous_blocked_until_release = f_pressed;
        // 暂停时也推进一次 OFF 命令，以便画面显示执行器真实余振而非键盘状态。
        if demo.paused.load(Ordering::Acquire) {
            demo.step_once_requested.store(true, Ordering::Release);
        }
    }

    if keyboard.just_pressed(KeyCode::KeyR) {
        intent.controls.reset();
        intent.pulse_queued = false;
        intent.continuous_requested = false;
        latch.continuous_blocked_until_release = f_pressed;
        demo.paused.store(true, Ordering::Release);
        demo.step_once_requested.store(false, Ordering::Release);
        demo.reset_requested.store(true, Ordering::Release);
    }
    drop(intent);

    if keyboard.just_pressed(KeyCode::Space) {
        let was_paused = demo.paused.fetch_xor(true, Ordering::AcqRel);
        if was_paused {
            demo.step_once_requested.store(false, Ordering::Release);
        }
    }
    if keyboard.just_pressed(KeyCode::ArrowRight) && demo.paused.load(Ordering::Acquire) {
        demo.step_once_requested.store(true, Ordering::Release);
    }
}

fn sync_lander_pose(
    demo: Res<PropulsionDemoState>,
    mut lander: Query<&mut Transform, With<ApolloVisualRoot>>,
) {
    let Ok(mut transform) = lander.single_mut() else {
        return;
    };
    let telemetry = *demo
        .latest
        .lock()
        .expect("latest propulsion telemetry mutex poisoned");
    transform.translation = dvec3(telemetry.snapshot.state.position_body_origin_world_m);
    transform.rotation = dquat(telemetry.snapshot.state.body_to_world);
}

fn sync_rcs_plumes(
    demo: Res<PropulsionDemoState>,
    mut plumes: Query<(&RcsPlumeVisual, &mut Transform, &mut Visibility)>,
) {
    let step = demo
        .latest
        .lock()
        .expect("latest propulsion telemetry mutex poisoned")
        .last_step;
    for (marker, mut transform, mut visibility) in &mut plumes {
        let mean_thrust_n = step
            .map(|step| step.applied.rcs[marker.thruster_index].mean_thrust_n)
            .unwrap_or(0.0);
        let steady_thrust_n =
            demo.propulsion_spec.rcs_thrusters[marker.thruster_index].steady_thrust_n;
        let fraction = (mean_thrust_n / steady_thrust_n).clamp(0.0, 1.0) as f32;
        if fraction <= 1.0e-5 {
            *visibility = Visibility::Hidden;
            continue;
        }
        // RCS 是全推力脉冲；D 喷口只要实际仍有推力，直线段就保持到
        // 导流板交汇处。自由羽流才用长度表达平均占空比。
        let length_fraction = match marker.path {
            RcsPlumePath::Free => fraction.max(0.08),
            RcsPlumePath::DeflectorIntercept => 1.0,
        };
        let length_m = marker.maximum_length_m * length_fraction;
        let geometry_scale = length_m / RCS_FREE_PLUME_LENGTH_M;
        let radial_scale = geometry_scale * fraction.sqrt().max(0.28);
        transform.translation.y = -RCS_PLUME_EXIT_OFFSET_M - length_m * 0.5;
        transform.scale = Vec3::new(radial_scale, geometry_scale, radial_scale);
        *visibility = Visibility::Inherited;
    }
}

fn sync_rcs_deflected_plumes(
    demo: Res<PropulsionDemoState>,
    mut plumes: Query<(&RcsDeflectedPlumeVisual, &mut Transform, &mut Visibility)>,
) {
    let step = demo
        .latest
        .lock()
        .expect("latest propulsion telemetry mutex poisoned")
        .last_step;
    for (marker, mut transform, mut visibility) in &mut plumes {
        let mean_thrust_n = step
            .map(|step| step.applied.rcs[marker.thruster_index].mean_thrust_n)
            .unwrap_or(0.0);
        let steady_thrust_n =
            demo.propulsion_spec.rcs_thrusters[marker.thruster_index].steady_thrust_n;
        let fraction = (mean_thrust_n / steady_thrust_n).clamp(0.0, 1.0) as f32;
        if fraction <= 1.0e-5 {
            *visibility = Visibility::Hidden;
            continue;
        }
        let length_m = marker.maximum_length_m * (0.45 + 0.55 * fraction);
        let direction = marker.plume_direction_body;
        let geometry_scale = length_m / RCS_FREE_PLUME_LENGTH_M * RCS_DEFLECTED_PLUME_RADIUS_SCALE;
        let radial_scale = geometry_scale * fraction.sqrt().max(0.24);
        transform.translation = marker.source_body_m + direction * (length_m * 0.5);
        transform.rotation = Quat::from_rotation_arc(Vec3::Y, -direction);
        transform.scale = Vec3::new(radial_scale, geometry_scale, radial_scale);
        *visibility = Visibility::Inherited;
    }
}

fn sync_dps_visuals(
    demo: Res<PropulsionDemoState>,
    mut plume: Query<(&mut Transform, &mut Visibility), With<DpsPlumeVisual>>,
    mut engine: Query<&mut Transform, (With<DpsEngineVisual>, Without<DpsPlumeVisual>)>,
) {
    let step = demo
        .latest
        .lock()
        .expect("latest propulsion telemetry mutex poisoned")
        .last_step;
    let applied = step.map(|step| step.applied.dps);

    if let Ok(mut engine_transform) = engine.single_mut() {
        let direction = applied
            .map(|applied| applied.force_direction_body)
            .unwrap_or(demo.propulsion_spec.dps.nominal_force_direction_body);
        engine_transform.rotation = Quat::from_rotation_arc(Vec3::Y, dvec3(direction).normalize());
    }

    let Ok((mut plume_transform, mut visibility)) = plume.single_mut() else {
        return;
    };
    let thrust_n = applied.map(|applied| applied.thrust_n).unwrap_or(0.0);
    let fraction = (thrust_n / demo.propulsion_spec.dps.full_thrust_n).clamp(0.0, 1.0) as f32;
    if fraction <= 1.0e-5 {
        *visibility = Visibility::Hidden;
        return;
    }
    let axial_scale = fraction.max(0.10);
    let radial_scale = fraction.sqrt().max(0.24);
    plume_transform.translation.y = DPS_PLUME_EXIT_Y - DPS_PLUME_HALF_HEIGHT_M * axial_scale;
    plume_transform.scale = Vec3::new(radial_scale, axial_scale, radial_scale);
    *visibility = Visibility::Inherited;
}

fn update_status(demo: Res<PropulsionDemoState>, mut text: Query<&mut Text, With<StatusText>>) {
    let Ok(mut text) = text.single_mut() else {
        return;
    };
    let telemetry = *demo
        .latest
        .lock()
        .expect("latest propulsion telemetry mutex poisoned");
    let controls = demo
        .command_intent
        .lock()
        .expect("propulsion command intent mutex poisoned")
        .controls;
    let run_state = if demo.running.load(Ordering::Acquire) {
        if demo.paused.load(Ordering::Acquire) {
            "PAUSED"
        } else {
            "RUNNING"
        }
    } else {
        "STOPPED"
    };
    let selected = selected_description(controls, &demo.torque_sets, demo.propulsion_spec);
    let applied = telemetry
        .last_step
        .map(|step| applied_description(step, demo.propulsion_spec))
        .unwrap_or_else(|| "requested: none | applied: none (reset state)".to_owned());
    let worker_error = demo
        .worker_error
        .lock()
        .expect("propulsion worker error mutex poisoned")
        .clone()
        .unwrap_or_else(|| "none".to_owned());

    *text = Text::new(format!(
        "{run_state} | tick {tick} | sim {sim_time:.3} s | mode {mode}\nselected: {selected}\n{applied}\nRCS label suffix = plume direction; spacecraft force = opposite.\nD jets: plume graphic impinges on the Apollo 11 deflector, then bends outward; thrust loss is not modeled.\nSpace pause | R reset | Right one tick | Tab mode | [ ] select | Enter 14 ms pulse | hold F continuous\nI DPS on/off | Up/Down 525 lbf detent | A/D gimbal -X/+X | W/S gimbal +Z/-Z | 0/Backspace all off\nworker error: {worker_error}",
        tick = telemetry.snapshot.control_tick,
        sim_time = telemetry.snapshot.sim_time_seconds(demo.timing),
        mode = mode_name(controls.mode),
    ));
}

fn mode_name(mode: DemoMode) -> &'static str {
    match mode {
        DemoMode::SingleRcs => "SINGLE RCS",
        DemoMode::TorqueCouple => "PURE TORQUE COUPLE",
        DemoMode::Dps => "DPS",
    }
}

fn selected_description(
    controls: PropulsionDemoControls,
    torque_sets: &[AxisTorqueSet; 6],
    spec: ApolloPropulsionSpec,
) -> String {
    match controls.mode {
        DemoMode::SingleRcs => {
            let thruster = spec.rcs_thrusters[controls.selected_single_thruster];
            format!("{} [index {}]", thruster.label, thruster.id.index())
        }
        DemoMode::TorqueCouple => {
            let set = &torque_sets[controls.selected_torque_axis];
            let ids = set
                .thruster_indices
                .iter()
                .map(|index| spec.rcs_thrusters[*index].label)
                .collect::<Vec<_>>()
                .join("+");
            format!(
                "{} torque via {ids} ({:.1} N m)",
                axis_name(controls.selected_torque_axis),
                set.torque_about_com_body_nm.dot(set.target_axis_body)
            )
        }
        DemoMode::Dps => "DPS focus; RCS pulse keys intentionally idle".to_owned(),
    }
}

fn axis_name(index: usize) -> &'static str {
    ["+X", "-X", "+Y", "-Y", "+Z", "-Z"][index]
}

fn applied_description(step: PropulsionStep, spec: ApolloPropulsionSpec) -> String {
    let requested_rcs = step
        .requested_command
        .rcs
        .on_time_ns
        .iter()
        .enumerate()
        .filter(|(_, on_time_ns)| **on_time_ns > 0)
        .map(|(index, on_time_ns)| {
            format!(
                "{}:{:.0}ms",
                spec.rcs_thrusters[index].label,
                *on_time_ns as f64 * 1.0e-6
            )
        })
        .collect::<Vec<_>>();
    let applied_rcs = step
        .applied
        .rcs
        .iter()
        .enumerate()
        .filter(|(_, applied)| applied.mean_thrust_n > 1.0e-6)
        .map(|(index, applied)| {
            format!(
                "{}:{:.1}N",
                spec.rcs_thrusters[index].label, applied.mean_thrust_n
            )
        })
        .collect::<Vec<_>>();
    let requested_rcs = if requested_rcs.is_empty() {
        "off".to_owned()
    } else {
        requested_rcs.join(",")
    };
    let applied_rcs = if applied_rcs.is_empty() {
        "off".to_owned()
    } else {
        applied_rcs.join(",")
    };
    let requested_dps = requested_dps_description(step.requested_command.dps);
    let applied_dps = step.applied.dps;
    format!(
        "requested RCS {requested_rcs} | DPS {requested_dps}\napplied RCS {applied_rcs} | DPS {} {:.0}N gimbal({:+.1},{:+.1})deg | mean F({:+.0},{:+.0},{:+.0})N T({:+.0},{:+.0},{:+.0})Nm",
        dps_mode_name(applied_dps.mode),
        applied_dps.thrust_n,
        applied_dps.gimbal_x_rad.to_degrees(),
        applied_dps.gimbal_z_rad.to_degrees(),
        step.applied.mean_wrench_body.force_body_n.x,
        step.applied.mean_wrench_body.force_body_n.y,
        step.applied.mean_wrench_body.force_body_n.z,
        step.applied.mean_wrench_body.torque_about_com_body_nm.x,
        step.applied.mean_wrench_body.torque_about_com_body_nm.y,
        step.applied.mean_wrench_body.torque_about_com_body_nm.z,
    )
}

fn requested_dps_description(command: DpsCommand) -> String {
    match command {
        DpsCommand::Off => "OFF".to_owned(),
        DpsCommand::Variable {
            thrust_n,
            gimbal_x_rad,
            gimbal_z_rad,
        } => format!(
            "VARIABLE {:.0}N ({:+.1},{:+.1})deg",
            thrust_n,
            gimbal_x_rad.to_degrees(),
            gimbal_z_rad.to_degrees()
        ),
        DpsCommand::FullThrust {
            gimbal_x_rad,
            gimbal_z_rad,
        } => format!(
            "FTP ({:+.1},{:+.1})deg",
            gimbal_x_rad.to_degrees(),
            gimbal_z_rad.to_degrees()
        ),
    }
}

fn dps_mode_name(mode: DpsMode) -> &'static str {
    match mode {
        DpsMode::Off => "OFF",
        DpsMode::Variable => "VARIABLE",
        DpsMode::FullThrust => "FTP",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn adapter_preserves_all_sixteen_pulse_times_and_full_thrust_gimbal() {
        let mut on_time_ns = [0; 16];
        on_time_ns[2] = 14_000_000;
        on_time_ns[11] = 20_000_000;
        let command = to_core_command(DemoPropulsionRequest {
            rcs_on_time_ns: on_time_ns,
            dps: DemoDpsRequest::FullThrust {
                gimbal_x_rad: 0.02,
                gimbal_z_rad: -0.03,
            },
        });
        assert_eq!(command.rcs.on_time_ns, on_time_ns);
        assert!(matches!(
            command.dps,
            DpsCommand::FullThrust {
                gimbal_x_rad: 0.02,
                gimbal_z_rad: -0.03,
            }
        ));
    }

    #[test]
    fn shared_spec_produces_named_two_jet_couples_for_every_axis() {
        let factory = ApolloPropulsionPlantFactory::apollo11_touchdown().unwrap();
        let sets = torque_sets(
            factory.propulsion_spec(),
            factory.model_spec().center_of_mass_body_m,
        )
        .unwrap();
        assert!(sets.iter().all(|set| set.thruster_indices.len() == 2));
        let description = selected_description(
            PropulsionDemoControls {
                mode: DemoMode::TorqueCouple,
                ..default()
            },
            &sets,
            factory.propulsion_spec(),
        );
        assert!(description.contains("+X torque via"));
    }
}
