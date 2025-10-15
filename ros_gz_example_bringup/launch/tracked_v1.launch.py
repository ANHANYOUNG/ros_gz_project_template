# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# git test
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
    # Configure ROS nodes for launch

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

    slip_plot_node = Node(
    package='ros_gz_example_application',
    executable='plot_slip_vs_sim_time.py',
    name='slip_plot_node',
    parameters=[{'use_sim_time': True}],
    output='screen'
    )


    # 6-1. 로컬 EKF (IMU -> odom): IMU 데이터만 사용하여 부드럽지만 드리프트가 있는 지역(local) 주행 거리계를 생성합니다.
    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        output="screen",
        parameters=[os.path.join(pkg_project_bringup, "config", "ekf.yaml"),
                    {'use_sim_time': True}
                    ],
        remappings=[('odometry/filtered', 'odometry/local')] # 출력 토픽 이름을 변경
    )

    # 6-2. 글로벌 EKF (GPS+IMU -> map): GPS와 IMU 데이터를 융합하여 드리프트가 없는 전역(global) 위치를 추정합니다.
    ekf_global_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global",
        output="screen",
        parameters=[os.path.join(pkg_project_bringup, "config", "ekf.yaml"),
                    {'use_sim_time': True}
                    ],
        remappings=[('odometry/filtered', 'odometry/global')] # 출력 토픽 이름을 변경
    )

    # 6-3. NavSat Transform: GPS의 위도/경도 데이터를 EKF가 사용할 수 있는 UTM 또는 지역 좌표계(x,y)로 변환합니다.
    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[os.path.join(pkg_project_bringup, "config", "ekf.yaml"),
                    {'use_sim_time': True}
                    ],
        remappings=[
            ("imu", "/imu"),
            ("gps/fix", "/navsat"),
            ("odometry/filtered", "/odometry/global"),
            ("gps/filtered", "/gps/filtered"),
            ("odometry/gps", "/odometry/gps"),
            
        ]
    )

    static_base_prefix_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_prefix",
        arguments=["0","0","0","0","0","0", "chassis_link", "tracked_v2/chassis_link"],
        output="screen",
    )
    static_imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_imu_to_base",
        arguments=["0.1", "0", "0", "0", "0", "0", "chassis_link", "tracked_v2/chassis_link/imu_sensor"],
        output="screen",
    )
    static_gps_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_gps_to_base",
        arguments=["0.1", "0", "0", "0", "0", "0", "chassis_link", "tracked_v2/chassis_link/navsat_sensor"],
        output="screen",
    )

    #twist_mux_node for priority of cmd_vel
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[os.path.join(pkg_project_bringup, 'config', 'twist_mux.yaml')],
        output='screen',
    )


    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        bridge,
        robot_state_publisher,
        # rviz,
        #pure_pursuit_node,
        # slip_plot_node,
        ekf_local_node,
        ekf_global_node,
        # navsat_transform_node,
        static_base_prefix_tf,
        static_imu_tf,
        static_gps_tf,
        twist_mux_node,
    ])
