from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'video_to_ai'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'),
            glob('models/*.pt')),
    ],
    install_requires=['setuptools', 'numpy<2'],
    zip_safe=True,
    maintainer='tom',
    maintainer_email='tom@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'video_inference_node = video_to_ai.video_inference_node:main',
            'fake_oak_publisher = video_to_ai.fake_oak_publisher:main',
        ],
    },
)
