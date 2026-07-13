use bevy::prelude::*;
use bevy_spacecraft::attitude_control::target_attitude;
use bevy_spacecraft::control_env::SharedApolloState;
use bevy_spacecraft::spacecraft_model::{create_lander_materials, spawn_lander};
use bevy_spacecraft::visualization::{
    TARGET_FRAME_CENTER, create_reference_frame_materials, create_star_material,
    spawn_current_frame, spawn_default_camera_and_light, spawn_stars, spawn_target_frame,
};

#[derive(Component)]
struct ApolloVisualRoot;

#[derive(Component)]
struct ApolloCurrentFrameRoot;

#[derive(Resource)]
struct ApolloStateResource(SharedApolloState);

fn main() {
    let shared_state =
        SharedApolloState::start().expect("failed to start Apollo simulation thread");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "MuJoCo Apollo Cascaded Attitude Control".into(),
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
        .insert_resource(ApolloStateResource(shared_state))
        .add_systems(Startup, setup)
        .add_systems(Update, (handle_input, sync_visual_from_simulation).chain())
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
    let current_frame =
        spawn_current_frame(&mut commands, &mut meshes, &frame_materials, Quat::IDENTITY);
    commands
        .entity(current_frame)
        .insert(ApolloCurrentFrameRoot);

    let lander = spawn_lander(
        &mut commands,
        &mut meshes,
        &lander_materials,
        Transform::from_xyz(0.0, 0.85, 0.0),
    );
    commands.entity(lander).insert(ApolloVisualRoot);

    commands.spawn((
        Text::new(
            "MuJoCo Apollo 6DoF | cascaded attitude control | outer quaternion kinematics + inner rate PID torque | R reset",
        ),
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

fn handle_input(keyboard: Res<ButtonInput<KeyCode>>, shared_state: Res<ApolloStateResource>) {
    if keyboard.just_pressed(KeyCode::KeyR) {
        shared_state.0.request_reset();
    }
}

fn sync_visual_from_simulation(
    shared_state: Res<ApolloStateResource>,
    mut lander_query: Query<&mut Transform, With<ApolloVisualRoot>>,
    mut frame_query: Query<
        &mut Transform,
        (With<ApolloCurrentFrameRoot>, Without<ApolloVisualRoot>),
    >,
) {
    let state = shared_state.0.snapshot().state;
    let position = Vec3::from_array(state.position.to_array());
    let rotation = Quat::from_array(state.rotation.to_array());

    let Ok(mut transform) = lander_query.single_mut() else {
        return;
    };
    transform.translation = position + Vec3::Y * 0.85;
    transform.rotation = rotation;

    let Ok(mut frame_transform) = frame_query.single_mut() else {
        return;
    };
    frame_transform.translation = TARGET_FRAME_CENTER;
    frame_transform.rotation = rotation;
}
