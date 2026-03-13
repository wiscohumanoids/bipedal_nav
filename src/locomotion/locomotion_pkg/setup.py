from setuptools import find_packages, setup

package_name = 'locomotion_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/locomotion.launch.py']),
        ('share/' + package_name + '/config', ['config/locomotion_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Locomotion Team',
    maintainer_email='club@example.com',
    description='RL locomotion policy for humanoid walking',
    license='MIT',
    entry_points={
        'console_scripts': [
            'locomotion_node = locomotion_pkg.locomotion_node:main',
            'gait_scheduler_node = locomotion_pkg.gait_scheduler_node:main',
        ],
    },
)
