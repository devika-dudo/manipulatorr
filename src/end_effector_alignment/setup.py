from setuptools import setup

package_name = 'end_effector_alignment'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/alignment_launch.py']),
        ('share/' + package_name + '/config', ['config/alignment_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='devika',
    maintainer_email='your_email@example.com',
    description='End effector alignment based on ArUco markers',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'end_effector_aligner = end_effector_alignment.end_effector_aligner:main',
        ],
    },
)
