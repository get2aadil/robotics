from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stt_package',
            executable='stt_node',
            name='stt_node',
        ),
        Node(
            package='nlp_package',
            executable='nlp_node',
            name='nlp_node',
        ),
        Node(
            package='image_capture_package',
            executable='image_capture_node',
            name='image_capture_node',
        ),
        Node(
            package='navigation_package',
            executable='navigation_node',
            name='navigation_node',
        ),
    ])

