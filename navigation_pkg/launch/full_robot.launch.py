from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():

    reset_old_nodes = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'pkill -f position_pid_node || true; '
            'pkill -f mecanum_controller_node || true; '
            'pkill -f mecanum_odometry_node || true; '
            'pkill -f path_follower_node || true; '
            'pkill -f obstacle_avoid_node || true; '
            'pkill -f line_follower || true; '
            'sleep 1'
        ],
        output='screen'
    )

    # Launch the base robot nodes first
    start_base_nodes = TimerAction(
        period=1.5,
        actions=[
            Node(
                package='imu_pkg',
                executable='imu_node',
                output='screen'
            ),
            Node(
                package='imu_pkg',
                executable='position_pid_node',
                output='screen'
            ),
            Node(
                package='mecanum_controller_pkg',
                executable='mecanum_controller_node',
                output='screen'
            ),
            Node(
                package='mecanum_controller_pkg',
                executable='mecanum_odometry_node',
                output='screen'
            ),
            Node(
                package='navigation_pkg',
                executable='obstacle_avoid_node',
                output='screen'
            ),
            Node(
                package='navigation_pkg',
                executable='line_follower',
                output='screen'
            ),
        ]
    )

    # Launch the camera nodes last (2.5 seconds after base nodes)
    start_camera_nodes = TimerAction(
        period=4.0, 
        actions=[
            Node(
                package='camera_pkg',
                executable='lane_detection_node',
                output='screen'
            ),
            Node(
                package='robot_gui',
                executable='camera_publisher',
                output='screen'
            ),
        ]
    )

    return LaunchDescription([
        reset_old_nodes,
        start_base_nodes,
        start_camera_nodes
    ])