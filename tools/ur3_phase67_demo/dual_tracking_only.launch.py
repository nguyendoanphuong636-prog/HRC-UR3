from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    py = str(Path.home() / 'ur3_hri_ws/.venv-mediapipe/bin/python')
    actions = []

    for cam in ['front', 'back']:
        prefix = '/kinect_' + cam

        actions.append(ExecuteProcess(
            cmd=[
                py, '-m', 'ur3_pose_tracking_2d.pose_node',
                '--ros-args',
                '-r', '__node:=pose2d_' + cam,
                '-p', 'image_topic:=' + prefix + '/color_registered/image_raw',
                '-p', 'output_prefix:=' + prefix + '/pose2d',
                '-p', 'max_width:=384',
                '-p', 'max_fps:=8.0',
                '-p', 'publish_overlay:=false'
            ],
            output='screen'
        ))

        actions.append(ExecuteProcess(
            cmd=[
                py, '-m', 'ur3_pose_tracking_3d.node',
                '--ros-args',
                '-r', '__node:=pose3d_' + cam,
                '-p', 'prefix:=' + prefix
            ],
            output='screen'
        ))

    return LaunchDescription(actions)
