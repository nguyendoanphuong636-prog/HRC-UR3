from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    py = str(Path.home() / 'ur3_hri_ws/.venv-mediapipe/bin/python')
    actions = []
    for cam, serial in [('front','501294742442'), ('back','000441552747')]:
        prefix = '/kinect_' + cam
        actions.append(Node(package='kinect_v2_ros2_driver', executable='kinect_publisher',
            namespace=prefix, name='driver', output='screen', parameters=[{
                'serial': ParameterValue(serial, value_type=str), 'frame_prefix': prefix[1:], 'packet_pipeline':'opengl',
                'enable_registration':True}]))
        actions.append(ExecuteProcess(cmd=[py, '-m', 'ur3_pose_tracking_2d.pose_node', '--ros-args',
            '-r', '__node:=pose2d_'+cam, '-p', 'image_topic:='+prefix+'/color_registered/image_raw',
            '-p', 'output_prefix:='+prefix+'/pose2d', '-p','max_width:=384','-p','max_fps:=8.0',
            '-p','publish_overlay:=true'], output='screen'))
        actions.append(ExecuteProcess(cmd=[py, '-m', 'ur3_pose_tracking_3d.node', '--ros-args',
            '-r','__node:=pose3d_'+cam, '-p','prefix:='+prefix], output='screen'))
    return LaunchDescription(actions)
