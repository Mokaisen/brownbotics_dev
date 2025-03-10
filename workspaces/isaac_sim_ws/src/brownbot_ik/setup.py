from setuptools import find_packages, setup

package_name = 'brownbot_ik'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Action Servers
            'ur5_controller = ur5_isaac_simulation.action_servers.ur5_controller:main',
            'gripper_controller = ur5_isaac_simulation.action_servers.gripper_controller:main',

            # Nodes
            'ur5_isaac_ros2 = ur5_isaac_simulation.ur5_isaac_ros2:main'
        ],
    },
)
