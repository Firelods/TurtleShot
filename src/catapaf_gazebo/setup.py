from setuptools import setup
import os
from glob import glob

package_name = 'catapaf_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    include_package_data=True,
    data_files=[
        # ('share/ament_index/resource_index/packages',
        #     ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), [f for f in glob('config/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'config', 'nav2'), glob('config/nav2/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), [f for f in glob('worlds/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'models', 'first_2015_trash_can'), 
        glob('models/first_2015_trash_can/model.*')),
        
        (os.path.join('share', package_name, 'models', 'first_2015_trash_can', 'meshes'), 
        glob('models/first_2015_trash_can/meshes/*')),

        (os.path.join('share', package_name, 'models', 'ball'),
            glob('models/ball/*')),
        
        (os.path.join('share', package_name, 'models', 'turtlebot_catapaf'),
            glob('models/turtlebot_catapaf/*')),

        (os.path.join('share', package_name, 'models', 'catapaf_common'),
            glob('models/catapaf_common/model.config')),

        (os.path.join('share', package_name, 'models', 'catapaf_common', 'meshes', 'bases'),
            glob('models/catapaf_common/meshes/bases/*.stl')),

        (os.path.join('share', package_name, 'models', 'catapaf_common', 'meshes', 'wheels'),
            glob('models/catapaf_common/meshes/wheels/*.stl')),

        (os.path.join('share', package_name, 'models', 'catapaf_common', 'meshes', 'sensors'),
            glob('models/catapaf_common/meshes/sensors/*.stl')),

        (os.path.join('share', package_name, 'models', 'catapaf_common', 'meshes', 'catapaf'),
            glob('models/catapaf_common/meshes/catapaf/*.stl')),
        (
            os.path.join('share', package_name, 'models', 'turtlebot3_world'),
            glob('models/turtlebot3_world/*.config')
        ),
        (
            os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')
        ),
        (
            os.path.join('share', package_name, 'models', 'turtlebot3_world'),
            glob('models/turtlebot3_world/*.sdf')
        ),
        (
            os.path.join('share', package_name, 'models', 'turtlebot3_world', 'meshes'),
            glob('models/turtlebot3_world/meshes/*')
        ),
        (
            os.path.join('share', package_name, 'models', 'trash_bin'),
            glob('models/trash_bin/model.*')
        ),
        (
            os.path.join('share', package_name, 'models', 'trash_bin', 'meshes'),
            glob('models/trash_bin/meshes/*')
        ),
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
            'arm_controller = catapaf_gazebo.arm_controller:main',
            'odom_to_tf = catapaf_gazebo.odom_to_tf:main',

            'ball_spawner = catapaf_gazebo.ball_spawner:main',
            'camera_tf_tuner = catapaf_gazebo.camera_tf_tuner:main',
            'autonomous_explorer = catapaf_gazebo.autonomous_explorer:main',
        ],
    },
)
