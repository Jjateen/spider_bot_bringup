#!/usr/bin/env python3
"""Load Big Bertha into Isaac Sim (PhysX) and hold the sim open.

Isaac Sim equivalent of what
big_bertha_sim_bringup/launch/simulation/simulation.launch.py does for
Gazebo: bring up a world and spawn the robot. Run via ``run_isaac_sim.sh``
(sources ~/isaac.sh first) rather than directly -- SimulationApp needs the
Isaac Sim venv's Python.

The URDF import here reuses big_bertha_description/urdf/big_bertha.urdf.xacro
unmodified (see prepare_urdf.sh for how it's expanded with use_gz:=false).
"""
import argparse
import os
import subprocess
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Pin the bridge to its bundled Humble libs, explicitly (not relying on the
# login shell's ROS_DISTRO=humble from ~/.bashrc sourcing /opt/ros/humble).
#
# This Isaac Sim 5.1.0 install also bundles Jazzy libs (matching the rest of
# the stack), but they're broken: enabling the bridge with ROS_DISTRO=jazzy
# fails to load with "undefined symbol:
# rcl_interfaces__msg__FloatingPointRange__get_type_hash" out of
# .../isaacsim.ros2.bridge/jazzy/lib/librcl_interfaces__rosidl_typesupport_c.so
# -- an ABI mismatch inside NVIDIA's own bundle, not an environment issue on
# this machine. Humble's bundle loads and works. The two are wire-compatible
# for the plain sensor_msgs/std_msgs/tf2_msgs/nav_msgs topics this bridge
# publishes, so a Humble-flavored bridge talking to the rest of the (Jazzy)
# stack over DDS is fine; only the in-process rclpy build differs.
os.environ["ROS_DISTRO"] = "humble"


def expand_urdf() -> str:
    """Run prepare_urdf.sh and return the path to the expanded URDF."""
    result = subprocess.run(
        [os.path.join(SCRIPT_DIR, "prepare_urdf.sh")],
        capture_output=True, text=True, check=True,
    )
    return result.stdout.strip().splitlines()[-1]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--headless", action="store_true", default=False,
        help="Run without the Kit GUI (default: windowed, like Gazebo's gui:=true)",
    )
    parser.add_argument(
        "--no-odom-tf", dest="odom_tf", action="store_false", default=True,
        help="Skip publishing odom->base_link tf (mirrors rsp.launch.py's "
             "odom_tf arg: set false when the EKF owns that transform "
             "instead, e.g. in the full SLAM bringup)",
    )
    args = parser.parse_args()

    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": args.headless})

    # Kit extension imports must come after SimulationApp starts the app.
    import numpy as np
    import omni.kit.commands
    from isaacsim.core.api import World
    from isaacsim.core.prims import Articulation
    from isaacsim.core.utils.extensions import enable_extension
    from pxr import UsdLux, Sdf
    from scipy.spatial.transform import Rotation

    # Not loaded by default in this app profile. enable_extension() only
    # queues the load -- the extension's on_startup (which does the actual
    # rclpy import) runs on later app update ticks, not synchronously here.
    # Pump updates until it's done, otherwise anything built against the
    # bridge's node types races a bridge that hasn't loaded rclpy yet (its
    # ROS2Context node silently produces a null context, no error raised).
    enable_extension("isaacsim.ros2.bridge")
    for _ in range(60):
        simulation_app.update()

    urdf_path = expand_urdf()
    print(f"[launch_isaac] expanded URDF: {urdf_path}", flush=True)

    # World owns the physics scene + default ground plane and handles the
    # reset/step calls articulation handles need to become valid.
    world = World()
    world.scene.add_default_ground_plane()
    UsdLux.DistantLight.Define(world.stage, Sdf.Path("/DistantLight")) \
        .CreateIntensityAttr(500)

    # obstacle_world.sdf's layout, shifted +3.5/+3.5 so the robot's spawn
    # (Isaac always spawns at world origin -- see isaac_bringup.launch.py's
    # docstring) sits where Gazebo's spawn (-3.5, -3.5) sits relative to the
    # room, keeping every robot-to-obstacle range identical between the two
    # sims. All obstacles are static and axis-aligned (SDF yaw=0 throughout).
    from isaacsim.core.api.objects import FixedCuboid, FixedCylinder
    boxes = [
        # name, (x, y, z), (size_x, size_y, size_z)
        ("wall_north", (3.5, 8.5, 0.25), (10.2, 0.1, 0.5)),
        ("wall_south", (3.5, -1.5, 0.25), (10.2, 0.1, 0.5)),
        ("wall_east", (8.5, 3.5, 0.25), (0.1, 10.2, 0.5)),
        ("wall_west", (-1.5, 3.5, 0.25), (0.1, 10.2, 0.5)),
        ("box_1", (2.3, 2.3, 0.2), (0.8, 0.8, 0.4)),
        ("box_2", (4.5, 4.5, 0.2), (0.9, 0.9, 0.4)),
        ("box_3", (3.7, 1.7, 0.2), (0.7, 0.7, 0.4)),
    ]
    for name, pos, size in boxes:
        FixedCuboid(
            prim_path=f"/World/{name}",
            name=name,
            position=np.array(pos),
            scale=np.array(size),
        )
    cylinders = [
        # name, (x, y, z), radius, height
        ("pillar_1", (3.5, 3.5, 0.25), 0.3, 0.5),
        ("pillar_2", (5.5, 3.0, 0.25), 0.25, 0.5),
    ]
    for name, pos, radius, height in cylinders:
        FixedCylinder(
            prim_path=f"/World/{name}",
            name=name,
            position=np.array(pos),
            radius=radius,
            height=height,
        )

    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    # Free-floating base (legs carry the robot), matching Gazebo -- base_link
    # is not welded to the world there either.
    import_config.fix_base = False
    import_config.make_default_prim = True
    # Matches self_collide=false in big_bertha.gazebo.xacro (legs use STL
    # mesh collision and don't overlap in a normal gait; self-contact was
    # disabled there to avoid noisy mesh-mesh forces).
    import_config.self_collision = False
    import_config.distance_scale = 1.0
    # Keep the URDF's own <inertial> masses (already tuned against the
    # deployed policy) instead of recomputing from geometry.
    import_config.density = 0.0
    # default_drive_type governs the *target* semantics authored on each
    # joint's DriveAPI (position vs velocity vs none) -- separate from the
    # *force interpretation* (force vs acceleration) fixed further below,
    # which this setting has no control over. default_drive_strength/damping
    # here are placeholders; set_gains() overwrites them with the real
    # kp=20/kd=2 right after import.
    from isaacsim.asset.importer.urdf._urdf import UrdfJointTargetType
    import_config.default_drive_type = UrdfJointTargetType.JOINT_DRIVE_POSITION
    import_config.default_drive_strength = 20.0
    import_config.default_position_drive_damping = 2.0

    # get_articulation_root=True: PublishJointState needs the prim carrying
    # PhysxArticulationRootAPI specifically, not the import's base path --
    # without this it's a level off ("Prim /big_bertha is not an
    # articulation") and joint states silently never publish.
    status, robot_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
        get_articulation_root=True,
    )
    print(f"[launch_isaac] imported robot at prim path: {robot_path}", flush=True)
    # Flat import layout confirmed by inspection: links are siblings under
    # /big_bertha (e.g. /big_bertha/lidar_link), not nested by kinematic
    # depth -- the importer encodes the kinematic tree via joint prims, not
    # USD parenting.
    robot_root = "/" + robot_path.strip("/").split("/")[0]  # "/big_bertha"
    lidar_link = f"{robot_root}/lidar_link"

    # The URDF importer authors every joint's DriveAPI with type="acceleration"
    # regardless of import_config (no import-time control over this exists;
    # confirmed by exhausting the importer's own default_drive_* options and
    # inspecting the physics tensor view's get_drive_types() == Acceleration
    # for all 12 DOFs). In acceleration mode, stiffness/damping are rad/s^2-
    # per-rad gains, not the N*m/rad torque-based PD ros2_control.yaml
    # defines and JointEffortPdController implements for Gazebo -- with
    # kp=20 interpreted as an acceleration gain, the resulting torque on
    # these joints' low inertia is negligible, so commanded targets were
    # silently never tracked (confirmed: identical non-response at
    # effort_limit=1.18 and 50 N*m, ruling out torque clamping). Switching
    # to "force" here, before world.reset() creates the physics view (drive
    # type has no runtime setter -- get_drive_types() is read-only), makes
    # stiffness/damping genuine N*m/rad torque gains, matching the Isaac
    # actuator ros2_control.yaml was tuned against.
    from pxr import UsdPhysics
    for p in world.stage.Traverse():
        if p.HasAPI(UsdPhysics.DriveAPI, "angular"):
            UsdPhysics.DriveAPI(p, "angular").GetTypeAttr().Set("force")

    robot = Articulation(prim_paths_expr=robot_path, name="big_bertha")
    world.scene.add(robot)
    world.reset()

    print(f"[launch_isaac] num_dof={robot.num_dof}", flush=True)
    print(f"[launch_isaac] dof_names={robot.dof_names}", flush=True)

    # MUST match the deployed policy's Isaac actuator (ros2_control.yaml:
    # kp=20, kd=2, effort_limit=1.18) -- this IS that actuator now, PhysX's
    # own implicit PD drive, not an emulation of it like
    # big_bertha_controllers/JointEffortPdController was for Gazebo.
    robot.set_gains(
        kps=np.full((1, robot.num_dof), 20.0),
        kds=np.full((1, robot.num_dof), 2.0),
    )
    robot.set_max_efforts(np.full((1, robot.num_dof), 1.18))

    # Training default pose, in Isaac articulation order (hips, knees,
    # ankles) -- see ros2_control.yaml's default_positions. Holds this pose
    # until the first /position_controller/commands message arrives.
    default_pose = np.array(
        [[0.0, 0.0, 0.0, 0.0,
          -0.32, -0.32, -0.32, -0.32,
          2.00, 2.00, 2.00, 2.00]]
    )
    from isaacsim.core.utils.types import ArticulationActions
    robot.apply_action(ArticulationActions(joint_positions=default_pose))

    lidar_prim = create_sensors(lidar_link)
    build_ros2_graph(robot_path, lidar_prim, publish_odom_tf=args.odom_tf)
    print("[launch_isaac] ROS 2 bridge graph built "
          f"(/clock, /joint_states, /scan, /imu, /odom"
          f"{', /tf' if args.odom_tf else ' -- /tf owned by the EKF'})",
          flush=True)

    command_node, latest_commands = create_command_subscriber()
    print("[launch_isaac] subscribed /position_controller/commands", flush=True)

    import rclpy
    import omni.graph.core as og

    # angularVelocity is computed here from finite-differenced orientation,
    # not read from any built-in Isaac angular-velocity API. Two were tried
    # and both proved wrong: IsaacComputeOdometry's angularVelocity output
    # went stale (confirmed via repeated idle tests -- robot genuinely
    # motionless, orientation exactly constant -- where /imu's
    # angular_velocity.z came back as a perfectly-constant nonzero value
    # across hundreds of samples within one run, a different constant each
    # run); switching to Articulation.get_angular_velocities() (PhysX's own
    # root-velocity tensor) looked promising -- confirmed via direct
    # instrumentation that it freezes solid the moment the articulation goes
    # to physics sleep, explaining the exact symptom -- but even with sleep
    # disabled (sleepThreshold=0.0) it still reported a persistent nonzero
    # reading (~0.007 rad/s) while orientation stayed flat to 8 significant
    # figures over 15s: inconsistent with real motion, so still wrong,
    # unclear why (possibly an artifact of how PhysX's reduced-coordinate
    # articulation solver reports root velocity under active force-mode PD
    # holds). Orientation itself (via get_world_poses(), same underlying
    # ComputeOdometry-adjacent physics-transform data) has never shown this
    # problem in any test -- correctly flat when genuinely still, correctly
    # changing during real walking -- so it's the trustworthy signal to
    # derive velocity from instead of trusting a separately-computed one.
    # World-frame, not imu_link's body frame like a real gyro -- accurate
    # for this near-level walker (roll/pitch stay small) but would need
    # revisiting for a robot that tips substantially.
    imu_angvel_attr = og.Controller.attribute(
        "/ROS2ActionGraph/PublishImu.inputs:angularVelocity")
    odom_angvel_attr = og.Controller.attribute(
        "/ROS2ActionGraph/PublishOdometry.inputs:angularVelocity")
    physics_dt = world.get_physics_dt()
    prev_orientation = None  # scalar-first (w, x, y, z), get_world_poses()'s convention

    # render=True regardless of --headless: "render" here means Kit's
    # app.update() tick (extensions, OmniGraph, timeline), not opening a GUI
    # window -- with render=False, SimulationContext.step() only calls the
    # raw physics step and skips app.update() entirely, so the ROS 2 action
    # graph (tied to OnPlaybackTick) never computes and nothing ever
    # publishes, silently, no error. --headless only controls the Kit window.
    while simulation_app.is_running():
        world.step(render=True)

        _, orientations = robot.get_world_poses()
        orientation = orientations[0]
        if prev_orientation is None:
            angular_velocity = [0.0, 0.0, 0.0]
        else:
            # scipy uses scalar-last (x, y, z, w) quaternions.
            r_prev = Rotation.from_quat(
                [prev_orientation[1], prev_orientation[2],
                 prev_orientation[3], prev_orientation[0]])
            r_curr = Rotation.from_quat(
                [orientation[1], orientation[2],
                 orientation[3], orientation[0]])
            delta = r_curr * r_prev.inv()
            angular_velocity = (delta.as_rotvec() / physics_dt).tolist()
        prev_orientation = orientation
        og.Controller.set(imu_angvel_attr, angular_velocity)
        og.Controller.set(odom_angvel_attr, angular_velocity)

        rclpy.spin_once(command_node, timeout_sec=0.0)
        if latest_commands:
            robot.apply_action(ArticulationActions(
                joint_positions=np.array([latest_commands])))

    simulation_app.close()


def create_sensors(lidar_link: str) -> str:
    """Create the lidar sensor prim. Returns its prim path.

    Physical params match big_bertha.gazebo.xacro's YDLidar X2 (360 samples,
    10 Hz, 0.35-8.0 m) -- a first pass, not yet tuned against Isaac's own
    noise/rate behavior the way the Gazebo values were.

    rotation_rate=10 (not 0): nonzero makes RangeSensorCreateLidar's PhysX
    RangeSensor genuinely sweep -- RotatingLidarPhysX accumulates beam
    columns progressively across physics ticks and IsaacReadLidarBeams only
    fires once a full rotation completes, stamped at sweep-end, so one
    message spans ~100 ms of real body tilt (60 Hz physics, no physics_dt
    override) corrected with scan_ground_filter's single end-of-sweep IMU
    sample -- a mismatch Gazebo's gpu_lidar doesn't have (always
    instantaneous per update). Tried rotation_rate=0 ("fire all rays at
    once", matching Gazebo's model -- see rangeSensorSchema/lidar.h's doc
    comment on the field) to remove this; measured result was worse, not
    better (near-band ghost-wall survival 89.2%->79.8%, measure_scan.py
    against the obstacle_world.sdf layout), and /scan's publish rate jumped
    ~10x (execOut then fires ~every physics tick instead of every rotation,
    and the sim was running faster than realtime) with no offsetting
    benefit. Reverted. The sweep/single-IMU-sample mismatch is real
    (confirmed against Isaac's own source) but not the dominant ghost-scan
    contributor -- don't re-try this without addressing why removing it
    made things worse first.

    No dedicated IMU sensor prim: isaacsim.sensors.physics.IsaacReadIMU never
    produced a reading against one here ("no valid sensor reading, is the
    sensor enabled?", persistent, not a startup race -- useLatestData=true
    and waiting well past sensor_period didn't change it, cause unconfirmed).
    /imu is synthesized instead (see build_ros2_graph and the angular
    velocity note in main()'s loop) -- imu_link is a fixed mount with no
    relative motion to base_link, so orientation (from IsaacComputeOdometry)
    and angular velocity (read directly from the Articulation's physics
    state each tick, not from ComputeOdometry -- see main()) both match a
    real IMU there; linear acceleration is the chassis's kinematic
    acceleration, not proper acceleration (no +g reaction when stationary,
    unlike a real accelerometer) -- worth revisiting if the policy leans on
    that channel.
    """
    import omni.kit.commands

    _, lidar_schema = omni.kit.commands.execute(
        "RangeSensorCreateLidar",
        path="Lidar",
        parent=lidar_link,
        min_range=0.35,
        max_range=8.0,
        horizontal_fov=360.0,
        vertical_fov=1.0,
        horizontal_resolution=1.0,
        vertical_resolution=1.0,
        rotation_rate=10.0,
    )
    return str(lidar_schema.GetPath())


def create_command_subscriber():
    """Subscribe /position_controller/commands (Float64MultiArray).

    big_bertha_policy_controller publishes 12 position targets (radians) on
    this topic, positionally ordered to match ros2_control.yaml's
    position_controller.joints list -- the same Isaac articulation order
    robot.dof_names already comes back in, so no reordering is needed here.
    This is a plain rclpy subscription rather than an OmniGraph node: the
    bridge extension has no generic Float64MultiArray subscriber node, and
    since rclpy is already loaded in-process (isaacsim.ros2.bridge's own
    startup proved that out), a direct subscription is simpler than forcing
    a non-standard message through OmniGraph.
    """
    import rclpy
    from std_msgs.msg import Float64MultiArray

    if not rclpy.ok():
        rclpy.init()

    node = rclpy.create_node("isaac_bridge_joint_commands")
    latest_commands = []

    def on_commands(msg: Float64MultiArray) -> None:
        latest_commands[:] = list(msg.data)

    node.create_subscription(
        Float64MultiArray, "/position_controller/commands", on_commands, 1)

    return node, latest_commands


def build_ros2_graph(robot_path: str, lidar_prim: str, publish_odom_tf: bool = True) -> None:
    """Sensor + clock/joint_state publishing over isaacsim.ros2.bridge.

    Topic names below match big_bertha_sim_bringup/config/ros_gz_bridge.yaml
    exactly (/scan, /imu, /odom, /tf) plus /joint_states (matches
    gz_ros2_control's JointStateBroadcaster today), so nothing downstream
    (leg_odometry, big_bertha_policy_controller) needs to change.

    publish_odom_tf=False skips the PublishTf node entirely -- mirrors
    rsp.launch.py's odom_tf arg: the full SLAM bringup's EKF owns
    odom->base_link instead (fed from this node's /odom), same as Gazebo.
    """
    import omni.graph.core as og
    import usdrt.Sdf

    keys = og.Controller.Keys

    create_nodes = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
        ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
        ("ReadLidarBeams", "isaacsim.sensors.physx.IsaacReadLidarBeams"),
        ("PublishLaserScan", "isaacsim.ros2.bridge.ROS2PublishLaserScan"),
        ("PublishImu", "isaacsim.ros2.bridge.ROS2PublishImu"),
        ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
        ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
    ]
    connect = [
        ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "ReadLidarBeams.inputs:execIn"),
        ("ReadLidarBeams.outputs:execOut", "PublishLaserScan.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
        ("ComputeOdometry.outputs:execOut", "PublishOdometry.inputs:execIn"),
        ("ComputeOdometry.outputs:execOut", "PublishImu.inputs:execIn"),

        ("Context.outputs:context", "PublishClock.inputs:context"),
        ("Context.outputs:context", "PublishJointState.inputs:context"),
        ("Context.outputs:context", "PublishLaserScan.inputs:context"),
        ("Context.outputs:context", "PublishImu.inputs:context"),
        ("Context.outputs:context", "PublishOdometry.inputs:context"),

        ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
        ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
        ("ReadSimTime.outputs:simulationTime", "PublishLaserScan.inputs:timeStamp"),
        ("ReadSimTime.outputs:simulationTime", "PublishImu.inputs:timeStamp"),
        ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),

        ("ReadLidarBeams.outputs:azimuthRange", "PublishLaserScan.inputs:azimuthRange"),
        ("ReadLidarBeams.outputs:depthRange", "PublishLaserScan.inputs:depthRange"),
        ("ReadLidarBeams.outputs:horizontalFov", "PublishLaserScan.inputs:horizontalFov"),
        ("ReadLidarBeams.outputs:horizontalResolution", "PublishLaserScan.inputs:horizontalResolution"),
        ("ReadLidarBeams.outputs:intensitiesData", "PublishLaserScan.inputs:intensitiesData"),
        ("ReadLidarBeams.outputs:linearDepthData", "PublishLaserScan.inputs:linearDepthData"),
        ("ReadLidarBeams.outputs:numCols", "PublishLaserScan.inputs:numCols"),
        ("ReadLidarBeams.outputs:numRows", "PublishLaserScan.inputs:numRows"),
        ("ReadLidarBeams.outputs:rotationRate", "PublishLaserScan.inputs:rotationRate"),

        # angularVelocity is NOT wired from ComputeOdometry -- see the
        # "stale angular velocity" note in main()'s loop for why; it's set
        # directly from Articulation.get_angular_velocities() every tick
        # instead, via og.Controller.set() on these two inputs.
        ("ComputeOdometry.outputs:linearAcceleration", "PublishImu.inputs:linearAcceleration"),
        ("ComputeOdometry.outputs:orientation", "PublishImu.inputs:orientation"),

        ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
        ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
        ("ComputeOdometry.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
    ]
    set_values = [
        ("PublishJointState.inputs:topicName", "joint_states"),
        ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(robot_path)]),

        ("ReadLidarBeams.inputs:lidarPrim", [usdrt.Sdf.Path(lidar_prim)]),
        ("PublishLaserScan.inputs:topicName", "scan"),
        ("PublishLaserScan.inputs:frameId", "lidar_link"),

        ("PublishImu.inputs:topicName", "imu"),
        ("PublishImu.inputs:frameId", "imu_link"),

        ("ComputeOdometry.inputs:chassisPrim", [usdrt.Sdf.Path(robot_path)]),
        ("PublishOdometry.inputs:topicName", "odom"),
        ("PublishOdometry.inputs:odomFrameId", "odom"),
        ("PublishOdometry.inputs:chassisFrameId", "base_link"),
    ]

    if publish_odom_tf:
        create_nodes.append(
            ("PublishTf", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"))
        connect += [
            ("ComputeOdometry.outputs:execOut", "PublishTf.inputs:execIn"),
            ("Context.outputs:context", "PublishTf.inputs:context"),
            ("ReadSimTime.outputs:simulationTime", "PublishTf.inputs:timeStamp"),
            ("ComputeOdometry.outputs:position", "PublishTf.inputs:translation"),
            ("ComputeOdometry.outputs:orientation", "PublishTf.inputs:rotation"),
        ]
        set_values += [
            ("PublishTf.inputs:topicName", "tf"),
            ("PublishTf.inputs:parentFrameId", "odom"),
            ("PublishTf.inputs:childFrameId", "base_link"),
        ]

    og.Controller.edit(
        {"graph_path": "/ROS2ActionGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: create_nodes,
            keys.CONNECT: connect,
            keys.SET_VALUES: set_values,
        },
    )


if __name__ == "__main__":
    main()
