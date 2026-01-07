from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yolo_oak_driver'



setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),  
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/yolo_oak_driver']),
        ('share/yolo_oak_driver', ['package.xml']),
        (os.path.join('share', 'yolo_oak_driver', 'models'), glob('models/*')),
        (os.path.join('share', 'yolo_oak_driver', 'helpers'), glob('helpers/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tom',
    maintainer_email='tom06530@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_oak_driver = yolo_oak_driver.oak_node:main'
        ],
    },
)
