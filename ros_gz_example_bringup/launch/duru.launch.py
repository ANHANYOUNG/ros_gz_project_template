import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():

    # 0) 패키지 경로
    pkg_bringup      = get_package_share_directory('ros_gz_example_bringup')
    pkg_gazebo       = get_package_share_directory('ros_gz_example_gazebo')
    pkg_description  = get_package_share_directory('ros_gz_example_description')
    pkg_ros_gz_sim   = get_package_share_directory('ros_gz_sim')
    pkg_application  = get_package_share_directory('ros_gz_example_application')

    # 1) world / model / config 파일 경로
    world_file = os.path.join(pkg_gazebo, 'worlds', 'agriculture_v2.sdf')
    sdf_file   = os.path.join(pkg_description, 'models', 'tracked_v2', 'tracked_v2.sdf')
    bridge_yaml = os.path.join(pkg_bringup, 'config', 'ros_gz_example_bridge.yaml')
    twist_mux_yaml = os.path.join(pkg_bringup, 'config', 'twist_mux.yaml')
    path_yaml  = os.path.join(pkg_bringup, 'config', 'path.yaml')

    # 2) SDF 로봇 기술서 로드 (robot_state_publisher 입력)
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 3) Gazebo (Ignition/GZ) 실행
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': PathJoinSubstitution([world_file])
        }.items(),
    )

    # 4) robot_state_publisher (시뮬 시간 사용)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ],
    )

    # 5) ROS ↔ GZ 브리지 (브리지 설정 파일 사용)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_yaml,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
    )

    # 6) 정적 TF (센서 프레임 고정)
    static_base_prefix_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_prefix",
        arguments=["0", "0", "0", "0", "0", "0", "chassis_link", "tracked_v2/chassis_link"],
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

    # 7) 로컬/글로벌 EKF (전역 좌표 /odometry/global 제공)
    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        output="screen",
        parameters=[os.path.join(pkg_bringup, "config", "ekf.yaml"),
                    {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odometry/local')],
    )
    ekf_global_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global",
        output="screen",
        parameters=[os.path.join(pkg_bringup, "config", "ekf.yaml"),
                    {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odometry/global')],
    )

    # 8) twist_mux (우선순위: stop > teleop > auto)
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_yaml],
    )

    # 9) State Manager — path.yaml 로드 + 모드 관리 + /ppc/path 퍼블리시
    state_machine_node = Node(
        package='ros_gz_example_application',
        executable='state_machine.py',
        name='state_manager_node',
        output='screen',
        parameters=[path_yaml, {'use_sim_time': True}],
    )

    # 10) Pure Pursuit 컨트롤러 — /ppc/path 구독, /cmd_vel_ppc 퍼블리시
    pure_pursuit_node = Node(
        package='ros_gz_example_application',
        executable='pure_pursuit_controller.py',
        name='pure_pursuit_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # (옵션) 시각화/디버깅 노드
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d', os.path.join(pkg_bringup, 'config', 'tracked_v1.rviz')],
    #     output='screen',
    # )

    return LaunchDescription([
        # Gazebo 시뮬레이터
        gz_sim,

        # 브리지 / 상태 발행
        bridge,
        robot_state_publisher,

        # EKF & TF
        ekf_local_node,
        ekf_global_node,
        static_base_prefix_tf,
        static_imu_tf,
        static_gps_tf,

        # 제어 선택기
        twist_mux_node,

        # 애플리케이션 노드
        state_machine_node,
        pure_pursuit_node,

        # RViz (원하면 주석 해제)
        # rviz,
    ])
