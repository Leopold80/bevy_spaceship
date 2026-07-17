use apollo_viewer::model::{
    RcsDeflectedPlumeVisual, RcsPlumePath, RcsPlumeVisual, create_lander_materials, spawn_lander,
};
use apollo_viewer::scene::{
    create_reference_frame_materials, create_star_material, default_window,
    insert_default_lighting, spawn_attitude_frame_legend, spawn_camera_and_light,
    spawn_current_attitude_frame, spawn_stars,
};
use bevy::prelude::*;
use bevy::render::view::screenshot::{Screenshot, ScreenshotCaptured, save_to_disk};
#[cfg(test)]
use std::collections::HashSet;
use std::ffi::OsString;
use std::path::PathBuf;

const LANDER_TRANSLATION: Vec3 = Vec3::new(0.0, 0.65, 0.0);
const LANDER_SCALE: f32 = 0.82;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum CaptureKind {
    Full,
    CloseUp,
}

#[derive(Clone, Copy, Debug)]
struct CaptureView {
    name: &'static str,
    kind: CaptureKind,
    eye: Vec3,
    target: Vec3,
    up: Vec3,
    diagnostic_quad: Option<usize>,
    show_all_down_diagnostics: bool,
}

impl CaptureView {
    const fn full(name: &'static str, eye: Vec3) -> Self {
        Self {
            name,
            kind: CaptureKind::Full,
            eye,
            target: Vec3::new(0.0, 1.45, 0.0),
            up: Vec3::Y,
            diagnostic_quad: None,
            show_all_down_diagnostics: false,
        }
    }

    const fn full_with_up(
        name: &'static str,
        eye: Vec3,
        up: Vec3,
        show_all_down_diagnostics: bool,
    ) -> Self {
        Self {
            name,
            kind: CaptureKind::Full,
            eye,
            target: Vec3::new(0.0, 1.45, 0.0),
            up,
            diagnostic_quad: None,
            show_all_down_diagnostics,
        }
    }

    const fn close(name: &'static str, eye: Vec3, target: Vec3, up: Vec3) -> Self {
        Self {
            name,
            kind: CaptureKind::CloseUp,
            eye,
            target,
            up,
            diagnostic_quad: None,
            show_all_down_diagnostics: false,
        }
    }

    const fn close_plume(name: &'static str, eye: Vec3, target: Vec3, quad_index: usize) -> Self {
        Self {
            name,
            kind: CaptureKind::CloseUp,
            eye,
            target,
            up: Vec3::Y,
            diagnostic_quad: Some(quad_index),
            show_all_down_diagnostics: false,
        }
    }
}

// 固定视角是模型回归检查的一部分：前八张覆盖整船；四张无羽流特写检查
// 独立安装板/法兰；四张斜侧诊断图检查 D 羽流撞板后转向；最后检查 DPS。
const CAPTURE_VIEWS: [CaptureView; 17] = [
    CaptureView::full("full_front", Vec3::new(0.0, 3.2, 13.8)),
    CaptureView::full("full_rear", Vec3::new(0.0, 3.2, -13.8)),
    CaptureView::full("full_right", Vec3::new(13.8, 3.2, 0.0)),
    CaptureView::full("full_left", Vec3::new(-13.8, 3.2, 0.0)),
    CaptureView::full_with_up("full_top", Vec3::new(0.0, 14.4, 0.01), Vec3::Z, false),
    CaptureView::full_with_up("full_bottom", Vec3::new(0.0, -12.2, 0.01), Vec3::Z, true),
    CaptureView::full("full_front_right_high", Vec3::new(10.0, 7.4, 10.0)),
    CaptureView::full("full_rear_left_high", Vec3::new(-10.0, 7.4, -10.0)),
    CaptureView::close(
        "close_mount_front_right",
        Vec3::new(3.25, 3.75, 3.25),
        Vec3::new(1.42, 3.0, 1.42),
        Vec3::Y,
    ),
    CaptureView::close(
        "close_mount_front_left",
        Vec3::new(-3.25, 3.75, 3.25),
        Vec3::new(-1.42, 3.0, 1.42),
        Vec3::Y,
    ),
    CaptureView::close(
        "close_mount_rear_right",
        Vec3::new(3.25, 3.75, -3.25),
        Vec3::new(1.42, 3.0, -1.42),
        Vec3::Y,
    ),
    CaptureView::close(
        "close_mount_rear_left",
        Vec3::new(-3.25, 3.75, -3.25),
        Vec3::new(-1.42, 3.0, -1.42),
        Vec3::Y,
    ),
    CaptureView::close_plume(
        "close_plume_front_right",
        Vec3::new(1.92, 3.70, 4.04),
        Vec3::new(1.42, 2.82, 1.42),
        3,
    ),
    CaptureView::close_plume(
        "close_plume_front_left",
        Vec3::new(-4.04, 3.70, 1.92),
        Vec3::new(-1.42, 2.82, 1.42),
        0,
    ),
    CaptureView::close_plume(
        "close_plume_rear_right",
        Vec3::new(4.04, 3.70, -1.92),
        Vec3::new(1.42, 2.82, -1.42),
        2,
    ),
    CaptureView::close_plume(
        "close_plume_rear_left",
        Vec3::new(-1.92, 3.70, -4.04),
        Vec3::new(-1.42, 2.82, -1.42),
        1,
    ),
    CaptureView::close(
        "close_dps_bell",
        Vec3::new(0.0, -4.1, 4.3),
        Vec3::new(0.0, 0.55, 0.0),
        Vec3::Y,
    ),
];

#[derive(Resource)]
struct ModelViewerOptions {
    capture_directory: Option<PathBuf>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum CapturePhase {
    WarmUp(u16),
    PositionCamera,
    Settle(u8),
    AwaitScreenshot,
    Complete(u8),
}

#[derive(Resource)]
struct CapturePlan {
    directory: PathBuf,
    next_view: usize,
    phase: CapturePhase,
}

#[derive(Component)]
struct CaptureJob {
    view_index: usize,
}

fn main() {
    let options = parse_arguments(std::env::args_os().skip(1)).unwrap_or_else(|message| {
        eprintln!("{message}");
        std::process::exit(2);
    });
    if let Some(directory) = &options.capture_directory {
        std::fs::create_dir_all(directory).unwrap_or_else(|error| {
            eprintln!("无法创建截图目录 {}: {error}", directory.display());
            std::process::exit(1);
        });
    }
    let capture_directory = options.capture_directory.clone();

    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(default_window("Apollo Model Viewer")));
    insert_default_lighting(&mut app);
    app.insert_resource(options).add_systems(Startup, setup);
    if let Some(directory) = capture_directory {
        app.insert_resource(CapturePlan {
            directory,
            next_view: 0,
            // 给材质、阴影和管线留出稳定帧，避免第一张图处于着色器预热状态。
            phase: CapturePhase::WarmUp(24),
        })
        .add_systems(Update, drive_capture_plan);
    }
    app.run();
}

fn parse_arguments(
    arguments: impl IntoIterator<Item = OsString>,
) -> Result<ModelViewerOptions, &'static str> {
    let mut arguments = arguments.into_iter();
    let Some(first) = arguments.next() else {
        return Ok(ModelViewerOptions {
            capture_directory: None,
        });
    };
    if first != "--capture-all" {
        return Err("用法: apollo-model-viewer [--capture-all <输出目录>]");
    }
    let Some(directory) = arguments.next() else {
        return Err("--capture-all 后必须提供输出目录");
    };
    if arguments.next().is_some() {
        return Err("apollo-model-viewer 不接受更多参数");
    }
    Ok(ModelViewerOptions {
        capture_directory: Some(directory.into()),
    })
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    options: Res<ModelViewerOptions>,
) {
    spawn_camera_and_light(&mut commands);
    let lander_materials = create_lander_materials(&mut materials);
    let star = create_star_material(&mut materials);
    spawn_stars(&mut commands, &mut meshes, star);

    // 批量视觉 QA 使用干净画面；交互查看时保留原有参考坐标系 API 和显示。
    if options.capture_directory.is_none() {
        let frame_materials = create_reference_frame_materials(&mut materials);
        spawn_attitude_frame_legend(&mut commands, false);
        spawn_current_attitude_frame(&mut commands, &mut meshes, &frame_materials, Quat::IDENTITY);
    }
    spawn_lander(
        &mut commands,
        &mut meshes,
        &lander_materials,
        Transform::from_translation(LANDER_TRANSLATION).with_scale(Vec3::splat(LANDER_SCALE)),
    );
}

fn drive_capture_plan(
    mut commands: Commands,
    mut plan: ResMut<CapturePlan>,
    mut camera: Single<&mut Transform, With<Camera3d>>,
    mut straight_plumes: Query<
        (&RcsPlumeVisual, &mut Visibility),
        Without<RcsDeflectedPlumeVisual>,
    >,
    mut deflected_plumes: Query<
        (&RcsDeflectedPlumeVisual, &mut Visibility),
        Without<RcsPlumeVisual>,
    >,
    mut exit: MessageWriter<AppExit>,
) {
    match plan.phase {
        CapturePhase::WarmUp(frames) if frames > 0 => {
            plan.phase = CapturePhase::WarmUp(frames - 1);
        }
        CapturePhase::WarmUp(_) | CapturePhase::PositionCamera => {
            let view = CAPTURE_VIEWS[plan.next_view];
            **camera = Transform::from_translation(view.eye).looking_at(view.target, view.up);
            // Mount 特写保持无羽流；斜侧 plume 特写只显示当前 quad，避免透明
            // 远侧羽流叠到机体上造成“穿透”错觉。底视图只显示四路 D 诊断。
            for (marker, mut visibility) in &mut straight_plumes {
                let selected_quad = view
                    .diagnostic_quad
                    .is_some_and(|quad| marker.thruster_index / 4 == quad);
                let selected_down = view.show_all_down_diagnostics
                    && marker.path == RcsPlumePath::DeflectorIntercept;
                *visibility = if selected_quad || selected_down {
                    Visibility::Inherited
                } else {
                    Visibility::Hidden
                };
            }
            for (marker, mut visibility) in &mut deflected_plumes {
                let selected_quad = view
                    .diagnostic_quad
                    .is_some_and(|quad| marker.thruster_index / 4 == quad);
                *visibility = if selected_quad || view.show_all_down_diagnostics {
                    Visibility::Inherited
                } else {
                    Visibility::Hidden
                };
            }
            plan.phase = CapturePhase::Settle(3);
        }
        CapturePhase::Settle(frames) if frames > 0 => {
            plan.phase = CapturePhase::Settle(frames - 1);
        }
        CapturePhase::Settle(_) => {
            let view_index = plan.next_view;
            let view = CAPTURE_VIEWS[view_index];
            let path = plan
                .directory
                .join(format!("{:02}_{}.png", view_index + 1, view.name));
            commands
                .spawn((Screenshot::primary_window(), CaptureJob { view_index }))
                .observe(save_to_disk(path))
                .observe(mark_capture_complete);
            plan.phase = CapturePhase::AwaitScreenshot;
        }
        CapturePhase::AwaitScreenshot => {}
        CapturePhase::Complete(frames) if frames > 0 => {
            plan.phase = CapturePhase::Complete(frames - 1);
        }
        CapturePhase::Complete(_) => {
            let full_views = CAPTURE_VIEWS
                .iter()
                .filter(|view| view.kind == CaptureKind::Full)
                .count();
            let closeups = CAPTURE_VIEWS.len() - full_views;
            println!(
                "视觉 QA 完成：{full_views} 张全景 + {closeups} 张推进特写已写入 {}",
                plan.directory.display()
            );
            exit.write(AppExit::Success);
        }
    }
}

fn mark_capture_complete(
    captured: On<ScreenshotCaptured>,
    jobs: Query<&CaptureJob>,
    mut plan: ResMut<CapturePlan>,
) {
    let Ok(job) = jobs.get(captured.entity) else {
        return;
    };
    if job.view_index != plan.next_view {
        return;
    }
    plan.next_view += 1;
    plan.phase = if plan.next_view == CAPTURE_VIEWS.len() {
        // `save_to_disk` 在观察器中同步写出；额外保留几帧让渲染资源完成清理。
        CapturePhase::Complete(4)
    } else {
        CapturePhase::PositionCamera
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn capture_plan_separates_mount_and_plume_diagnostics() {
        assert_eq!(
            CAPTURE_VIEWS
                .iter()
                .filter(|view| view.kind == CaptureKind::Full)
                .count(),
            8
        );
        assert_eq!(
            CAPTURE_VIEWS
                .iter()
                .filter(|view| view.kind == CaptureKind::CloseUp)
                .count(),
            9
        );
        assert_eq!(
            CAPTURE_VIEWS
                .iter()
                .filter(|view| view.name.starts_with("close_mount_"))
                .count(),
            4
        );
        assert_eq!(
            CAPTURE_VIEWS
                .iter()
                .filter(|view| view.diagnostic_quad.is_some())
                .count(),
            4
        );
        let unique_names: HashSet<_> = CAPTURE_VIEWS.iter().map(|view| view.name).collect();
        assert_eq!(unique_names.len(), CAPTURE_VIEWS.len());
        assert!(CAPTURE_VIEWS.iter().all(|view| {
            view.eye.is_finite()
                && view.target.is_finite()
                && view.up.is_normalized()
                && view.eye.distance(view.target) > 0.5
        }));
    }

    #[test]
    fn capture_all_argument_is_explicit_and_keeps_interactive_default() {
        assert!(
            parse_arguments(Vec::<OsString>::new())
                .unwrap()
                .capture_directory
                .is_none()
        );
        let parsed = parse_arguments(["--capture-all".into(), "qa-output".into()]).unwrap();
        assert_eq!(parsed.capture_directory, Some(PathBuf::from("qa-output")));
        assert!(parse_arguments(["--capture-all".into()]).is_err());
        assert!(parse_arguments(["--unknown".into()]).is_err());
    }
}
