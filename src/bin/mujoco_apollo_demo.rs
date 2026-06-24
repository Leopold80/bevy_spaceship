use bevy::prelude::*;
use bevy_spacecraft::mujoco_dynamics::{ApolloDynamics, ApolloWrench};
use bevy_spacecraft::spacecraft_model::{create_lander_materials, spawn_lander};
use bevy_spacecraft::visualization::{
    create_star_material, spawn_default_camera_and_light, spawn_stars,
};

#[derive(Component)]
struct ApolloVisualRoot;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "MuJoCo Apollo 6DoF Dynamics".into(),
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
        .insert_resource(ApolloDynamics::new().expect("failed to initialize Apollo MuJoCo model"))
        .add_systems(Startup, setup)
        .add_systems(Update, (handle_input, step_mujoco_and_sync_visual).chain())
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

    spawn_stars(&mut commands, &mut meshes, star);
    let lander = spawn_lander(
        &mut commands,
        &mut meshes,
        &lander_materials,
        Transform::from_xyz(0.0, 0.85, 0.0),
    );
    commands.entity(lander).insert(ApolloVisualRoot);

    commands.spawn((
        Text::new("MuJoCo Apollo 6DoF | R reset | body-frame force + torque demo"),
        TextFont::from_font_size(16.0),
        TextColor(Color::srgb(0.92, 0.96, 1.0)),
        Node {
            position_type: PositionType::Absolute,
            left: px(18.0),
            top: px(12.0),
            ..default()
        },
    ));
}

fn handle_input(keyboard: Res<ButtonInput<KeyCode>>, mut dynamics: ResMut<ApolloDynamics>) {
    if keyboard.just_pressed(KeyCode::KeyR) {
        dynamics.reset();
    }
}

fn step_mujoco_and_sync_visual(
    time: Res<Time>,
    mut dynamics: ResMut<ApolloDynamics>,
    mut query: Query<&mut Transform, With<ApolloVisualRoot>>,
) {
    let dt = time.delta_secs();
    let steps = (dt / 0.01).ceil().clamp(1.0, 6.0) as usize;
    let wrench = ApolloWrench {
        force_body: Vec3::new(180.0, 30.0, 0.0),
        torque_body: Vec3::new(0.0, 0.0, 40.0),
    };

    let mut state = dynamics.state();
    for _ in 0..steps {
        state = dynamics.step(wrench);
    }

    let Ok(mut transform) = query.single_mut() else {
        return;
    };

    transform.translation = state.position + Vec3::Y * 0.85;
    transform.rotation = state.rotation;
}
