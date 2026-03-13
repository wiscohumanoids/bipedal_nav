from setuptools import find_packages, setup

package_name = 'motion_primitives_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Humanoid Robotics Club',
    maintainer_email='club@example.com',
    description='Scripted motion primitives for testing sensors and perception',
    license='MIT',
    entry_points={
        'console_scripts': [
            'motion_primitives_node = motion_primitives_pkg.motion_primitives_node:main',
        ],
    },
)
