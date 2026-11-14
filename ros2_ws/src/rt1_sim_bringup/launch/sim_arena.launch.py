"""
==============================================================================
 FILE: rt1_sim_bringup/launch/sim_arena.launch.py
==============================================================================
 Target:
   One command that brings up the full arena: Gazebo with sorting_arena.sdf,
   OpenMANIPULATOR-X with both cameras, its controllers, and the bridge.

 Physics / Math Implemented:
   None. Timing invariant: every node runs on simulation time (/clock), so a
   real-time factor below 1.0 slows wall-clock speed but never changes data
   timing (30 Hz cameras and 5 Hz logging stay exact in sim time).

 Inputs & Outputs:
   Launch args : gui (true|false), world (path to .sdf)
   Publishes   : /clock, /cam_side/image, /cam_side/camera_info,
                 /cam_top/image, /cam_top/camera_info, /joint_states, /tf
   Actions     : /arm_controller/follow_joint_trajectory
                 /gripper_controller/gripper_cmd
   Frames      : link1 (base, at world origin), cam_*_optical_frame
==============================================================================
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # == 1. PATHS ==
    pkg_share = get_package_share_directory('rt1_sim_bringup')
    world_default = os.path.join(pkg_share, 'worlds', 'sorting_arena.sdf')
    robot_xacro = os.path.join(pkg_share, 'urdf', 'open_manipulator_dual_cam.urdf.xacro')
    bridge_yaml = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')

    # Gazebo resolves package:// mesh URIs by searching this path for a
    # folder named after the package. The parent of the share dir holds it.
    om_description_parent = os.path.dirname(
        get_package_share_directory('open_manipulator_description'))

    # == 2. LAUNCH ARGUMENTS ==
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='false = headless server (use during data collection)')
    declare_world = DeclareLaunchArgument('world', default_value=world_default)

    set_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        om_description_parent + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', ''))

    # == 3. GAZEBO ==
    # -r starts the simulation running; -s skips the GUI (server only).
    gz_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    gazebo_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': ['-r -v 3 ', world],
                          'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(gui))
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': ['-r -s -v 3 ', world],
                          'on_exit_shutdown': 'true'}.items(),
        condition=UnlessCondition(gui))

    # == 4. ROBOT DESCRIPTION ==
    # ParameterValue(str) stops ROS from trying to parse the URDF as YAML.
    robot_description = ParameterValue(Command(['xacro ', robot_xacro]), value_type=str)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
        output='screen')

    # == 5. SPAWN ROBOT ==
    # Reads the URDF from /robot_description. Base at world origin, so the
    # table top (z = 0) and link1 share one frame.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'open_manipulator_x',
                   '-x', '0.0', '-y', '0.0', '-z', '0.0'],
        output='screen')

    # == 6. CONTROLLERS ==
    # Names verified in step 1.1. Broadcaster first so /joint_states exists
    # before the trajectory controllers start.
    def spawner(name):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager'],
            parameters=[{'use_sim_time': True}],
            output='screen')

    jsb_spawner = spawner('joint_state_broadcaster')
    arm_spawner = spawner('arm_controller')
    gripper_spawner = spawner('gripper_effort_controller')

    # Event chain: robot spawned -> broadcaster -> arm + gripper.
    # Stops spawners racing the gz_ros2_control plugin at startup.
    start_jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[jsb_spawner]))
    start_arm_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner,
                      on_exit=[arm_spawner, gripper_spawner]))

    # == 7. GAZEBO -> ROS 2 BRIDGE ==
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_yaml, 'use_sim_time': True}],
        output='screen')

        # == 8. GRIPPER FORCE LAYER ==
    # One force command F (N, > 0 squeeze) -> equal efforts on both fingers.
    gripper_force = Node(
        package='rt1_sim_bringup',
        executable='gripper_force_node.py',
        parameters=[{'use_sim_time': True}],
        output='screen')

    return LaunchDescription([
        declare_gui,
        declare_world,
        set_resource_path,
        gazebo_with_gui,
        gazebo_headless,
        robot_state_publisher,
        spawn_robot,
        start_jsb_after_spawn,
        start_arm_after_jsb,
        bridge, gripper_force,
    ])