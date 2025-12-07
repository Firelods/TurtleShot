from setuptools import setup
import os
from glob import glob

package_name = 'catapaf_gazebo'
def list_files(path):
    files = []
    for root, dirs, filenames in os.walk(path):
        for f in filenames:
            files.append(os.path.join(root, f))
    return files

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # MODELS
        (os.path.join('share', package_name, 'models'),
            list_files('models')),

        # WORLDS
        (os.path.join('share', package_name, 'worlds'),
            list_files('worlds')),

        # CONFIG
        (os.path.join('share', package_name, 'config'),
            list_files('config')),

        # LAUNCH
        (os.path.join('share', package_name, 'launch'),
            list_files('launch')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='clement',
    maintainer_email='clement@example.com',
    description='Gazebo simulation for TurtleShot / CataPAF',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'catapaf_arm_controller = catapaf_gazebo.catapaf_arm_controller:main',
            'odom_to_tf = catapaf_gazebo.odom_to_tf:main',
        ],
    },
)
