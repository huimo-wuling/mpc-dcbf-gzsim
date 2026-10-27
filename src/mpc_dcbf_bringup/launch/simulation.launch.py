"""Bringup for the MPC-D-CBF simulation on ROS 2 + Gazebo Sim.

Port of the original dynamic.launch: starts Gazebo Sim with the world,
spawns the Jackal, bridges ros_gz topics, and runs the full algorithm
chain (odom TF, moving obstacles, local map, KF prediction, global path,
MPC-D-CBF planner, controller, RViz).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('mpc_dcbf_bringup')
    jackal_share = get_package_share_directory('jackal_gz')
    gz_sim_share = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(bringup_share, 'worlds', 'mpc_dcbf_world.sdf')
    model_path = os.path.join(jackal_share, 'models', 'jackal', 'model.sdf')
    urdf_path = os.path.join(bringup_share, 'urdf', 'jackal_tf.urdf')
    obstacles_yaml = os.path.join(bringup_share, 'config', 'obstacles.yaml')
    rviz_config = os.path.join(bringup_share, 'rviz', 'simulation.rviz')
    bridge_yaml = os.path.join(bringup_share, 'config', 'bridge.yaml')

    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('rviz')

    set_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(jackal_share, 'models') + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', ''))

    # Gazebo Sim: server only when headless:=true, otherwise server + GUI.
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-s -r -v 2 --headless-rendering ' + world_path}.items(),
        condition=IfCondition(headless))

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r -v 2 ' + world_path}.items(),
        condition=UnlessCondition(headless))

    # Spawn the Jackal at (-10, 0), as in the original dynamic.launch
    spawn_jackal = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'jackal', '-file', model_path, '-x', '-10', '-y', '0', '-z', '1.0'],
        output='screen')

    # gz <-> ROS bridge driven by config/bridge.yaml so that the gz-side topic
    # names (/velodyne_points/points, /model/jackal/odometry, ...) can differ
    # from the original ROS names the algorithm nodes subscribe to.
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'config_file': bridge_yaml}],
        output='screen')

    robot_state_pub = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_path).read(), 'use_sim_time': True}],
        output='screen')

    odom_tf = Node(package='mpc_scene', executable='odom_tf', output='screen')

    movetest = Node(
        package='mpc_scene', executable='movetest',
        parameters=[obstacles_yaml, {'use_sim_time': True}], output='screen')

    global_path = Node(
        package='global_path_publisher', executable='global_path_pub',
        parameters=[{'start_x': -10.0, 'start_y': 0.0, 'end_x': 9.0, 'end_y': 0.0, 'use_sim_time': True}],
        output='screen')

    local_map = Node(
        package='local_map', executable='local_map_pub',
        parameters=[{'localmap_x_size': 14.0, 'localmap_y_size': 14.0,
                     'resolution': 0.1, 'obs_height': 0.4, 'step_height': 0.5,
                     'DBSCAN_R': 5.0, 'DBSCAN_N': 4, 'use_sim_time': True}],
        output='screen')

    obs_kf = Node(package='obs_param', executable='obs_kf', output='screen')

    planner = Node(
        package='local_planner', executable='local_planner',
        parameters=[{'replan_period': 0.05, 'use_sim_time': True}], output='screen')

    controller = Node(package='local_planner', executable='controller', output='screen')

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz), output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run only the Gazebo server (no GUI).'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Start RViz2.'),
        set_resource_path,
        gz_sim_headless,
        gz_sim_gui,
        spawn_jackal,
        bridge,
        robot_state_pub,
        odom_tf,
        movetest,
        global_path,
        local_map,
        obs_kf,
        planner,
        controller,
        rviz,
    ])
