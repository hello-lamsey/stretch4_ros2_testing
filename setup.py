import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'stretch4_ros2_testing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lamsey',
    maintainer_email='lamsey@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "flying_gripper = stretch4_ros2_testing.flying_gripper:main",
            "mapping_trigger = stretch4_ros2_testing.mapping_trigger:main",
            "move_joints = stretch4_ros2_testing.move_joints:main",
            "move_to_pose = stretch4_ros2_testing.move_to_pose:main",
            "test_omnibase_primitives = stretch4_ros2_testing.test_omnibase_primitives:main",
        ],
    },
)
