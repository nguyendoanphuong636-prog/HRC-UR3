from setuptools import setup

package_name = 'ur3_pose_tracking_2d'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UR3 HRI Research',
    maintainer_email='research@example.invalid',
    description='Read-only MediaPipe Pose landmarks from Kinect v2 color ROS topics (not a safety system).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pose_tracker_2d=ur3_pose_tracking_2d.pose_node:main',
        ],
    },
)
