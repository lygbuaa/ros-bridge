"""
Setup for carla_spawn_parking
"""
import os
from glob import glob
ROS_VERSION = int(os.environ['ROS_VERSION'])

if ROS_VERSION == 1:
    from distutils.core import setup
    from catkin_pkg.python_setup import generate_distutils_setup

    d = generate_distutils_setup(
        packages=['carla_spawn_parking'],
        package_dir={'': 'src'}
    )

    setup(**d)

elif ROS_VERSION == 2:
    from setuptools import setup

    package_name = 'carla_spawn_parking'
    setup(
        name=package_name,
        version='0.0.0',
        packages=['src/' + package_name],
        data_files=[
            ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            ('share/' + package_name + '/config',
             ['config/objects.json', 'config/objects_lite.json', 'config/objects_microlino.json', 'config/objects_lincoln.json', 'config/objects_microlino_pcie.json']),
            # (os.path.join('share', package_name), glob('config/*.json')),
            (os.path.join('share', package_name), glob('launch/*.launch.py'))
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='CARLA Simulator Team',
        maintainer_email='carla.simulator@gmail.com',
        description='CARLA spawn_objects for ROS2 bridge',
        license='MIT',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'carla_spawn_parking = src.carla_spawn_parking.carla_spawn_parking:main',
                'set_initial_pose = src.carla_spawn_parking.set_initial_pose:main'
            ],
        },
    )
