from setuptools import find_packages, setup

package_name = 'perception_vision'

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
    maintainer='devika',
    maintainer_email='devikaskumar2005@gmail.com',
    description='YOLOv8 OBB real-time detection package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'yolov8_obb_node = perception_vision.yolov8_obb_node:main',
        ],
    },
)
