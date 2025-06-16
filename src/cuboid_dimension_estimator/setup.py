from setuptools import setup
import os
from glob import glob

package_name = 'cuboid_dimension_estimator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your-email@example.com',
    description='ROS2 package for estimating cuboid dimensions using back projection',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cuboid_estimator = cuboid_dimension_estimator.cuboid_estimator:main',
        ],
    },
)
