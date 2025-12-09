from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_gazebo_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        # Include URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # Include scripts
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
        
        # INSTALL ALL MESHES RECURSIVELY
        ('share/' + package_name + '/meshes/catapaf', [
            'meshes/catapaf/baseEtServo.stl',
            'meshes/catapaf/catapaf.stl'
        ]),
        # install subfolders too:
        ('share/' + package_name + '/meshes/bases', [
            'meshes/bases/burger_base.stl'
        ]),
        ('share/' + package_name + '/meshes/wheels', [
            'meshes/wheels/left_tire.stl',
            'meshes/wheels/right_tire.stl'
        ]),
        ('share/' + package_name + '/meshes/sensors', [
            'meshes/sensors/OAK-D-PRO.stl',
        ])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bilbo',
    maintainer_email='billebaultbaptiste@gmail.com',
    description='Gazebo simulation package for TurtleBot3 with random walker',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'random_walker = robot_gazebo_sim.random_walker:main',
        ],
    },
)
