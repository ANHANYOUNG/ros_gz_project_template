# teleop_tracked_real.launch.py
# - 실기 전용(NUCLEO-G474RE + micro-ROS)
# - teleop_twist_keyboard 는 포함하지 않음(별도 터미널에서 실행)
# - micro_ros_agent(serial) + /cmd_vel→/motor_cmd 브릿지만 실행
#
# 실행 예:
#   ros2 launch ros_gz_example_bringup teleop_tracked_real.launch.py
#   ros2 launch ros_gz_example_bringup teleop_tracked_real.launch.py serial_dev:=/dev/ttyUSB0
#   ros2 launch ros_gz_example_bringup teleop_tracked_real.launch.py start_agent:=false  # 에이전트 수동 실행 시

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # ── 런치 인자 ──
    serial_dev_arg = DeclareLaunchArgument(
        'serial_dev', default_value='/dev/ttyACM0',
        description='micro-ROS Agent serial device (e.g., /dev/ttyACM0 or /dev/ttyUSB0)'
    )
    baud_arg = DeclareLaunchArgument(
        'baud', default_value='115200',
        description='micro-ROS Agent baudrate'
    )
    lin_thresh_arg = DeclareLaunchArgument(
        'lin_thresh', default_value='0.05',
        description='|linear.x| > lin_thresh -> 전/후'
    )
    ang_thresh_arg = DeclareLaunchArgument(
        'ang_thresh', default_value='0.10',
        description='|angular.z| > ang_thresh AND |linear.x| < lin_thresh -> 좌/우(제자리)'
    )
    deadman_arg = DeclareLaunchArgument(
        'deadman_seconds', default_value='1.0',
        description='입력 끊긴 뒤 N초 후 자동 정지(/motor_cmd [0,0,0])'
    )
    start_agent_arg = DeclareLaunchArgument(
        'start_agent', default_value='true',
        description='Start micro-ROS agent from this launch (true/false)'
    )

    serial_dev = LaunchConfiguration('serial_dev')
    baud = LaunchConfiguration('baud')
    lin_thresh = LaunchConfiguration('lin_thresh')
    ang_thresh = LaunchConfiguration('ang_thresh')
    deadman_seconds = LaunchConfiguration('deadman_seconds')
    start_agent = LaunchConfiguration('start_agent')

    # ── ① micro-ROS Agent ──
    micro_ros_agent = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial', '--dev', serial_dev, '-b', baud
        ],
        output='screen',
        condition=IfCondition(start_agent)
    )

    # ── ② /cmd_vel → /motor_cmd 브릿지 ──
    bridge = Node(
        package='ros_gz_example_application',
        executable='cmd_vel_to_motor_cmd.py',
        name='cmdvel_to_motorcmd',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'lin_thresh': lin_thresh},
            {'ang_thresh': ang_thresh},
            {'deadman_seconds': deadman_seconds},
        ]
    )
    #     # ── ③ IMU 센서 노드 (추가된 부분) ──
    # imu_node = Node(
    #     package='ros_gz_example_application',
    #     executable='imu.py',
    #     name='imu_sensor_node',
    #     output='screen'
    # )

    return LaunchDescription([
        serial_dev_arg, baud_arg,
        lin_thresh_arg, ang_thresh_arg, deadman_arg,
        start_agent_arg,
        micro_ros_agent,
        bridge,
        # imu_node,
    ])
