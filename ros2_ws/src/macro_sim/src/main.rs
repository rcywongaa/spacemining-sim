use std::f32::consts::PI;
use std::sync::LazyLock;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use std::{f32::consts::FRAC_PI_2, ops::Range};

use bevy::{
    color::palettes::{basic::*, tailwind::*},
    input::mouse::AccumulatedMouseMotion,
    picking::pointer::PointerInteraction,
    prelude::*,
};

use geographiclib_rs::{DirectGeodesic, Geodesic};

use geometry_msgs::msg::Twist as TwistMsg;
use std_msgs::msg::String as StringMsg;

const ASTEROID_RADIUS: f32 = 50.;
const ASTEROID_SURFACE_RADIUS: f32 = ASTEROID_RADIUS + 0.01;
const ROVER_CONE_BASE_RADIUS: f32 = 0.75;
const ROVER_CONE_HEIGHT: f32 = 2.;

// #![feature(const_fn_floating_point_arithmetic)]
/// a: equatorial radius, f: flattening of ellipsoid (https://geographiclib.sourceforge.io/C++/doc/classGeographicLib_1_1Geodesic.html#ae66c9cecfcbbcb1da52cb408e69f65de)
static GEODESIC: LazyLock<Geodesic> = LazyLock::new(|| Geodesic::new(ASTEROID_RADIUS as f64, 0.0));

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
        .add_systems(Update, update_rover_position)
        .add_systems(Update, draw_mesh_intersections)
        .add_systems(Update, orbit)
        .run();
}

#[derive(Default)]
struct LatLong((f64, f64));

#[derive(Resource, Default)]
struct World {
    rover: Rover,
}

#[derive(Default)]
struct Rover {
    entity: Option<Entity>,
    // velocity: (f32, f32, f32), // body frame x, y, yaw
    latlon: LatLong,
    heading: f64, // deg, 0 means facing north
}

pub trait RosNodeAccessor: Send + Sync {
    fn get_node(&self) -> Arc<rclrs::Node>;
}

#[derive(Resource)]
struct Ros {
    context: rclrs::Context,
    nodes: Mutex<Vec<Arc<dyn RosNodeAccessor>>>,
}
impl Ros {
    fn new() -> Self {
        Self {
            context: rclrs::Context::new(std::env::args()).unwrap(),
            nodes: Mutex::new(Vec::new()),
        }
    }
}

fn ros_spin_once(ros: ResMut<Ros>, mut world: ResMut<World>) {
    for node in &*(ros.nodes.lock().unwrap()) {
        rclrs::spin_once(node.get_node(), Some(Duration::new(0, 1000)));
    }
}

#[derive(Component)]
struct RoverVelocityListenerComponent(Arc<RoverVelocityListener>);

struct RoverVelocityListener {
    node: Arc<rclrs::Node>,
    _subscription: Arc<rclrs::Subscription<TwistMsg>>,
    data: Arc<Mutex<Option<TwistMsg>>>,
}

impl RoverVelocityListener {
    fn new(context: &rclrs::Context, name: &str, topic: &str) -> Self {
        let node = rclrs::Node::new(context, name).unwrap();
        let data = Arc::new(Mutex::new(None));
        let cb_data = Arc::clone(&data);
        let _subscription = node
            .create_subscription(topic, rclrs::QOS_PROFILE_DEFAULT, move |msg: TwistMsg| {
                info!("Received message: {:?}", msg);
                *cb_data.lock().unwrap() = Some(msg);
            })
            .unwrap();
        Self {
            node,
            _subscription,
            data,
        }
    }

    fn get_data(&self) -> Option<TwistMsg> {
        self.data.lock().unwrap().clone()
    }
}

impl RosNodeAccessor for RoverVelocityListener {
    fn get_node(&self) -> Arc<rclrs::Node> {
        self.node.clone()
    }
}

fn setup_cameras(mut commands: Commands) {
    commands.spawn((
        Transform::from_translation(Vec3::new(0.0, 0.0, 1.0)).looking_at(Vec3::ZERO, Vec3::Y),
        Camera3d::default(),
    ));
}

fn setup_rover(
    mut world: ResMut<World>,
    mut ros: ResMut<Ros>,
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mesh = meshes.add(Cone::new(ROVER_CONE_BASE_RADIUS, ROVER_CONE_HEIGHT));
    let green_mtl = materials.add(StandardMaterial {
        emissive: GREEN.into(),
        base_color: Color::from(GREEN),
        ..Default::default()
    });

    let rover_velocity_listener = Arc::new(RoverVelocityListener::new(
        &ros.context,
        "rover_velocity",
        "rover_velocity_cmd",
    ));
    ros.nodes
        .lock()
        .unwrap()
        .push(rover_velocity_listener.clone());

    world.rover.entity = Some(
        commands
            .spawn((
                Transform::from_xyz(0., 0., ASTEROID_SURFACE_RADIUS),
                Visibility::default(),
                RoverVelocityListenerComponent(rover_velocity_listener.clone()),
            ))
            .with_children(|parent| {
                parent.spawn((
                    Transform::from_rotation(Quat::from_rotation_x(0.5 * PI)),
                    Mesh3d(mesh),
                    MeshMaterial3d(green_mtl),
                ));
            })
            .id(),
    );
    world.rover.latlon = LatLong((0.0, 0.0));
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

fn latlon_to_xyz(lat: f64, lon: f64) -> Vec3 {
    Vec3::new(
        ASTEROID_SURFACE_RADIUS * (lat.to_radians().cos() * lon.to_radians().sin()) as f32,
        ASTEROID_SURFACE_RADIUS * (lat.to_radians().sin()) as f32,
        ASTEROID_SURFACE_RADIUS * (lat.to_radians().cos() * lon.to_radians().cos()) as f32,
    )
}

/// 0 azimuth is facing east while 0 heading is facing north
fn heading_to_azimuth(heading: f64) -> f64 {
    heading + 90.0
}
fn azimuth_to_heading(azimuth: f64) -> f64 {
    azimuth - 90.0
}

fn update_rover_position(
    mut rover_velocity_listener_query: Query<&RoverVelocityListenerComponent>,
    mut world: ResMut<World>,
    mut transforms: Query<&mut Transform>,
    time: Res<Time>,
) {
    let mut rover = &mut world.rover;
    let twist = rover_velocity_listener_query
        .get(rover.entity.unwrap())
        .unwrap()
        .0
        .get_data();
    // let Some(twist) = twist else { return };
    let twist = twist.unwrap_or(TwistMsg::default());
    // info!("Twist: {:?}", twist);

    let dist = twist.linear.x * (time.delta_secs() as f64);

    /** FIXME: Sloppy math! **/
    let rover_azimuth = heading_to_azimuth(rover.heading);
    let (lat, lon, az) = GEODESIC.direct(rover.latlon.0 .0, rover.latlon.0 .1, rover_azimuth, dist);
    let (look_at_lat, look_at_lon, _) =
        GEODESIC.direct(rover.latlon.0 .0, rover.latlon.0 .1, rover_azimuth, 0.01);
    rover.latlon = LatLong((lat, lon));
    rover.heading =
        azimuth_to_heading(az) + twist.angular.z.to_degrees() * (time.delta_secs() as f64);
    let rover_position = latlon_to_xyz(rover.latlon.0 .0, rover.latlon.0 .1);
    /*****/

    // up direction is defined by the vector pointing from the center of the asteroid (0, 0, 0) to the rover position
    let up = rover_position;

    let prev_transform = transforms.get(rover.entity.unwrap()).unwrap();
    *transforms.get_mut(rover.entity.unwrap()).unwrap() = Transform {
        translation: rover_position,
        // rotation: Quat::from_rotation_y(rover.heading as f32),
        ..default()
    }
    .looking_at(
        latlon_to_xyz(look_at_lat, look_at_lon),
        Dir3::new(up).unwrap(),
    )
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
