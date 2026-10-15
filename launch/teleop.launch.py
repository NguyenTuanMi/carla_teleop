from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('carla_teleop'),
        'config',
        'teleop.yaml',
    ])

    return LaunchDescription([
        Node(
            package='carla_teleop',
            executable='teleop_node',
            name='carla_teleop',
            output='screen',
            emulate_tty=True,
            parameters=[config],
        ),
    ])
