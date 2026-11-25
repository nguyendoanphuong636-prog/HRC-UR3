from setuptools import setup
from glob import glob
import os

package_name = 'ur3_human_fusion'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Research Team',
    maintainer_email='maintainer@example.invalid',
    description='Read-only dual Kinect skeleton fusion prototype',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'fusion_node = ur3_human_fusion.fusion_node:main',
        'synthetic_skeleton = ur3_human_fusion.synthetic_skeleton:main',
    ]},
)
