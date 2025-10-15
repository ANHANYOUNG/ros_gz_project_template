# Copyright 2022 Open Source Robotics Foundation, Inc.
# ... (라이선스 헤더는 기존과 동일) ...

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # ── 기존 시뮬레이션 설정 (tracked_v1.launch.py) ──

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('ros_gz_example_bringup')
    pkg_project_gazebo = get_package_share_directory('ros_gz_example_gazebo')
    pkg_project_description = get_package_share_directory('ros_gz_example_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Load the SDF file from "description" package
    sdf_file  =  os.path.join(pkg_project_description, 'models', 'tracked_v2', 'tracked_v2.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'agriculture_v1.sdf'
        ])}.items(),
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

        # Visualize in RViz
    # rviz = Node(
    #    package='rviz2',
    #    executable='rviz2',
    #    arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'tracked_v1.rviz')],
    #    condition=IfCondition(LaunchConfiguration('rviz'))
    # )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_example_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    ####  ros_gz_example_application Nodes ####
    pure_pursuit_node = Node(
        package='ros_gz_example_application',
        executable='pure_pursuit_controller.py',
        name='pure_pursuit_node',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Localization, TF, etc. (기존 노드들)
    ekf_local_node = Node(package="robot_localization", executable="ekf_node", name="ekf_local", output="screen", parameters=[os.path.join(pkg_project_bringup, "config", "ekf.yaml"), {'use_sim_time': True}], remappings=[('odometry/filtered', 'odometry/local')])

    ekf_global_node = Node(package="robot_localization", executable="ekf_node", name="ekf_global", output="screen", parameters=[os.path.join(pkg_project_bringup, "config", "ekf.yaml"), {'use_sim_time': True}], remappings=[('odometry/filtered', 'odometry/global')])

    static_base_prefix_tf = Node(package="tf2_ros", executable="static_transform_publisher", name="tf_base_prefix", arguments=["0","0","0","0","0","0", "chassis_link", "tracked_v2/chassis_link"], output="screen")

    static_imu_tf = Node(package="tf2_ros", executable="static_transform_publisher", name="tf_imu_to_base", arguments=["0.1", "0", "0", "0", "0", "0", "chassis_link", "tracked_v2/chassis_link/imu_sensor"], output="screen")

    static_gps_tf = Node(package="tf2_ros", executable="static_transform_publisher", name="tf_gps_to_base", arguments=["0.1", "0", "0", "0", "0", "0", "chassis_link", "tracked_v2/chassis_link/navsat_sensor"], output="screen")

    twist_mux_node = Node(package='twist_mux', executable='twist_mux', name='twist_mux', parameters=[os.path.join(pkg_project_bringup, 'config', 'twist_mux.yaml')], output='screen')

    # ── 🔷 micro-ROS Agent 추가 부분 🔷 ──
    
    # 런치 인자 선언
    serial_dev_arg = DeclareLaunchArgument(
        'serial_dev', default_value='/dev/ttyACM0',
        description='micro-ROS Agent serial device (e.g., /dev/ttyACM0 or /dev/ttyUSB0)'
    )
    baud_arg = DeclareLaunchArgument(
        'baud', default_value='115200',
        description='micro-ROS Agent baudrate'
    )
    start_agent_arg = DeclareLaunchArgument(
        'start_agent', default_value='true',
        description='Start micro-ROS agent from this launch (true/false)'
    )

    # 런치 인자 값 가져오기
    serial_dev = LaunchConfiguration('serial_dev')
    baud = LaunchConfiguration('baud')
    start_agent = LaunchConfiguration('start_agent')

    # micro-ROS Agent 노드 정의
    micro_ros_agent = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial', '--dev', serial_dev, '-b', baud
        ],
        output='screen',
        condition=IfCondition(start_agent)
    )

    # ── 최종 런치 리스트 ──
    return LaunchDescription([
        # 기존 시뮬레이션 노드들
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        bridge,
        robot_state_publisher,
        pure_pursuit_node,
        ekf_local_node,
        ekf_global_node,
        static_base_prefix_tf,
        static_imu_tf,
        static_gps_tf,
        twist_mux_node,
        # rviz,
        
        # micro-ROS 관련 인자 및 노드 추가
        serial_dev_arg,
        baud_arg,
        start_agent_arg,
        micro_ros_agent,
    ])