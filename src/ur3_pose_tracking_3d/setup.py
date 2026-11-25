from setuptools import setup

package_name = 'ur3_pose_tracking_3d'
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
    maintainer='research',
    maintainer_email='research@example.invalid',
    description='Read-only 3D skeleton reconstruction in Kinect depth optical frame.',
    license='MIT',
    entry_points={'console_scripts': ['pose3d_node = ur3_pose_tracking_3d.node:main']},
)
