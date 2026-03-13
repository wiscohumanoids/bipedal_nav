from setuptools import find_packages, setup

package_name = 'slam_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/slam.launch.py']),
        ('share/' + package_name + '/config', ['config/slam_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SLAM Team',
    maintainer_email='club@example.com',
    description='SLAM package for humanoid navigation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'slam_node = slam_pkg.slam_node:main',
            'path_planner_node = slam_pkg.path_planner_node:main',
        ],
    },
)
