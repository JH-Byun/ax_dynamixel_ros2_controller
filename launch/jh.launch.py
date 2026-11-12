from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    common_params = [{'ids': [3]}]
    position_speed = 30  # Raw joint speed: 1–1023; lower values move slower.

    return LaunchDescription([
        Node(
            package='dynamixel_test_ctrl',
            executable='master_node',
            name='master_node',
            output='screen',
            parameters=common_params + [{'position_speed': position_speed}],
        ),
        Node(
            package='dynamixel_test_ctrl',
            executable='gui_node',
            name='gui_node',
            output='screen',
            parameters=common_params,
        ),
    ])
