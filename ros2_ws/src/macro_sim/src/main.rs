use std::sync::{Arc, Mutex};
use std::time::Duration;
use std::{f32::consts::FRAC_PI_2, ops::Range};

use bevy::{
    color::palettes::{basic::*, tailwind::*},
    input::mouse::AccumulatedMouseMotion,
    picking::pointer::PointerInteraction,
    prelude::*,
};

use std_msgs::msg::String as StringMsg;

const ASTEROID_RADIUS: f32 = 50.;
const ASTEROID_SURFACE_RADIUS: f32 = ASTEROID_RADIUS + 0.01;
const ROVER_CONE_BASE_RADIUS: f32 = 0.75;
const ROVER_CONE_HEIGHT: f32 = 2.;
fn main() {
    App::new()
        .add_plugins((DefaultPlugins, MeshPickingPlugin))
        .init_resource::<CameraSettings>()
        .init_resource::<World>()
        .insert_resource(Ros::new())
        .add_systems(Startup, setup_cameras)
        .add_systems(Startup, setup_world)
        .add_systems(Startup, setup_rover)
        .add_systems(Update, ros_spin_once)
        .add_systems(Update, draw_mesh_intersections)
        .add_systems(Update, orbit)
        .run();
}

#[derive(Resource, Default)]
struct World {
    rover_position: (f32, f32),
}

#[derive(Resource)]
struct Ros {
    context: rclrs::Context,
    republisher: RepublisherNode,
}
impl Ros {
    fn new() -> Self {
        let context = rclrs::Context::new(std::env::args()).unwrap();
        let republisher = RepublisherNode::new(&context);
        Self {
            context,
            republisher,
        }
    }
}

struct RepublisherNode {
    node: Arc<rclrs::Node>,
    _subscription: Arc<rclrs::Subscription<StringMsg>>,
    data: Arc<Mutex<Option<StringMsg>>>,
}
impl RepublisherNode {
    fn new(context: &rclrs::Context) -> Self {
        let node = rclrs::Node::new(context, "republisher").unwrap();
        let data = Arc::new(Mutex::new(None));
        let cb_data = Arc::clone(&data);
        let _subscription = node
            .create_subscription(
                "in_topic",
                rclrs::QOS_PROFILE_DEFAULT,
                move |msg: StringMsg| {
                    info!("Received message: {}", msg.data);
                    *cb_data.lock().unwrap() = Some(msg);
                },
            )
            .unwrap();
        Self {
            node,
            _subscription,
            data,
        }
    }
}

// fn ros_setup(mut ros: ResMut<Ros>) {
//     ros.context = rclrs::Context::new(std::env::args()).unwrap();
//     ros.republisher = RepublisherNode::new(&ros.context);
// }

fn ros_spin_once(ros: ResMut<Ros>) {
    rclrs::spin_once(ros.republisher.node.clone(), Some(Duration::new(0, 1000)));
}

fn setup_cameras(mut commands: Commands) {
    commands.spawn((
        Transform::from_translation(Vec3::new(0.0, 0.0, 1.0)).looking_at(Vec3::ZERO, Vec3::Y),
        Camera3d::default(),
    ));
}

fn setup_rover(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let rover = meshes.add(Cone::new(ROVER_CONE_BASE_RADIUS, ROVER_CONE_HEIGHT));
    let green_mtl = materials.add(StandardMaterial {
        emissive: GREEN.into(),
        base_color: Color::from(GREEN),
        ..Default::default()
    });

    commands.spawn((
        Transform::from_xyz(0., 0., ASTEROID_SURFACE_RADIUS),
        Mesh3d(rover),
        MeshMaterial3d(green_mtl),
    ));
}

fn setup_world(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // let sphere = asset_server.load(GltfAssetLabel::Scene(0).from_asset("models/sphere/sphere.glb"));
    let sphere = meshes.add(Sphere::new(ASTEROID_RADIUS).mesh().ico(5).unwrap());
    let gray_mtl = materials.add(Color::from(GRAY_500));

    commands.spawn((
        PointLight {
            intensity: 1e9,
            shadows_enabled: true,
            range: 500.0,
            ..default()
        },
        Transform::from_xyz(100.0, 200.0, 100.0),
    ));
    commands.spawn((
        Transform::from_xyz(0., 0., 0.),
        Mesh3d(sphere),
        MeshMaterial3d(gray_mtl),
    ));
}

fn draw_mesh_intersections(pointers: Query<&PointerInteraction>, mut gizmos: Gizmos) {
    for (point, normal) in pointers
        .iter()
        .filter_map(|interaction| interaction.get_nearest_hit())
        .filter_map(|(_entity, hit)| hit.position.zip(hit.normal))
    {
        gizmos.sphere(point, 0.05, RED_500);
        gizmos.arrow(point, point + normal.normalize() * 0.5, PINK_100);
    }
}

#[derive(Debug, Resource)]
struct CameraSettings {
    pub orbit_distance: f32,
    pub pitch_speed: f32,
    // Clamp pitch to this range
    pub pitch_range: Range<f32>,
    pub roll_speed: f32,
    pub yaw_speed: f32,
}

impl Default for CameraSettings {
    fn default() -> Self {
        // Limiting pitch stops some unexpected rotation past 90° up or down.
        let pitch_limit = FRAC_PI_2 - 0.01;
        Self {
            // These values are completely arbitrary, chosen because they seem to produce
            // "sensible" results for this example. Adjust as required.
            orbit_distance: 200.0,
            pitch_speed: 0.005,
            pitch_range: -pitch_limit..pitch_limit,
            roll_speed: 1.0,
            yaw_speed: 0.005,
        }
    }
}

fn orbit(
    mut camera: Single<&mut Transform, With<Camera>>,
    camera_settings: Res<CameraSettings>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    mouse_motion: Res<AccumulatedMouseMotion>,
    time: Res<Time>,
) {
    let delta = mouse_motion.delta;
    let mut delta_pitch = 0.0;
    let mut delta_yaw = 0.0;

    if mouse_buttons.pressed(MouseButton::Left) {
        // Mouse motion is one of the few inputs that should not be multiplied by delta time,
        // as we are already receiving the full movement since the last frame was rendered. Multiplying
        // by delta time here would make the movement slower that it should be.
        delta_pitch = -delta.y * camera_settings.pitch_speed;
        delta_yaw = -delta.x * camera_settings.yaw_speed;
    }

    // Obtain the existing pitch, yaw, and roll values from the transform.
    let (yaw, pitch, roll) = camera.rotation.to_euler(EulerRot::YXZ);

    // Establish the new yaw and pitch, preventing the pitch value from exceeding our limits.
    let pitch = (pitch + delta_pitch).clamp(
        camera_settings.pitch_range.start,
        camera_settings.pitch_range.end,
    );
    let yaw = yaw + delta_yaw;
    camera.rotation = Quat::from_euler(EulerRot::YXZ, yaw, pitch, roll);

    // Adjust the translation to maintain the correct orientation toward the orbit target.
    // In our example it's a static target, but this could easily be customized.
    let target = Vec3::ZERO;
    camera.translation = target - camera.forward() * camera_settings.orbit_distance;
}
