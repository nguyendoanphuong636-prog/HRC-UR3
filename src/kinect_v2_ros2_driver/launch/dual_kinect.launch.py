from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='kinect_v2_ros2_driver', executable='kinect_publisher',
             namespace='kinect_front', name='driver', output='screen',
             parameters=[{'serial': '501294742442', 'frame_prefix': 'kinect_front', 'packet_pipeline': 'opengl', 'enable_registration': True}]),
        Node(package='kinect_v2_ros2_driver', executable='kinect_publisher',
             namespace='kinect_back', name='driver', output='screen',
             parameters=[{'serial': '000441552747', 'frame_prefix': 'kinect_back', 'packet_pipeline': 'opengl'}]),
    ])
