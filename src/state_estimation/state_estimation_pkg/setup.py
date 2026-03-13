from setuptools import find_packages, setup

package_name = 'state_estimation_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/state_estimation.launch.py']),
        ('share/' + package_name + '/config', ['config/state_estimation_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='State Estimation Team',
    maintainer_email='club@example.com',
    description='State estimation for humanoid robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'state_estimator_node = state_estimation_pkg.state_estimator_node:main',
            'contact_estimator_node = state_estimation_pkg.contact_estimator_node:main',
        ],
    },
)
