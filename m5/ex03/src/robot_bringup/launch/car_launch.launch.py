from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=['-entity', 'car', '-x', '0.0', '-y', '0.0', '-z', '0.5', '-file', 'path/to/your/package/car.urdf']
        )
    ])

