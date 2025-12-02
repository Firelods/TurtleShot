from setuptools import find_packages, setup

package_name = 'fake_lidar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bilbo',
    maintainer_email='billebaultbaptiste@gmail.com',
    description='Package for simulating a fake LiDAR sensor',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fake_lidar = fake_lidar.fake_lidar:main',
            'scene_publisher = fake_lidar.scene_publisher:main',
        ],
    },
)
