from setuptools import setup
import os
from glob import glob

package_name = 'oakd_camera_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bilbo',
    maintainer_email='bilbo@todo.todo',
    description='Oak-D Camera Driver Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_tf_publisher = oakd_camera_driver.imu_tf_publisher:main',
            'camera_publisher = oakd_camera_driver.camera_publisher:main',
        ],
    },
)
