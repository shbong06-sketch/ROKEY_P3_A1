from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'smart_farm_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shbong',
    maintainer_email='shbong06@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'object_detection = smart_farm_vision.inspection_executor_node:main',
            'topic_audit = smart_farm_vision.topic_audit:main',
            'frame_grab = smart_farm_vision.frame_grab:main',
        ],
    },
)
