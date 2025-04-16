from setuptools import find_packages, setup

package_name = 'cylinder_position_estimator'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest','opencv-python'],
    entry_points={
    'console_scripts': [
        'pose_estimator_node = cylinder_position_estimator.pose_estimator_node:main',  # Ensure this points to your main function
    ],

    },
)
