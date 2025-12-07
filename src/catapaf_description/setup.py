from setuptools import setup

package_name = 'catapaf_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/launch', [
            'launch/display.launch.py'
        ]),
        ('share/' + package_name + '/urdf', [
            'urdf/turtlebot_with_catapaf.urdf',
            'urdf/catapaf.urdf.xacro',
            'urdf/turtlebot_with_catapaf_gz.urdf.xacro'
        ]),
        
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
            'meshes/sensors/lds.stl',
        ]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Clément',
    maintainer_email='example@example.com',
    description='Catapulte URDF + STL for TurtleShot project',
    license='MIT',
    tests_require=['pytest'],
    entry_points={},
)
