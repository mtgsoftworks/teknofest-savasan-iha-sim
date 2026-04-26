from setuptools import find_packages, setup

package_name = 'offboard_takeoff'

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
    maintainer='mtg',
    maintainer_email='mtg@todo.todo',
    description='PX4 MAVROS offboard takeoff helper node',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'offboard_takeoff = offboard_takeoff.offboard_takeoff_node:main',
            'offboard_mission = offboard_takeoff.offboard_mission_node:main',
            'fw_mission = offboard_takeoff.fw_mission_node:main',
            'geofence = offboard_takeoff.geofence_node:main',
            'telemetry_logger = offboard_takeoff.telemetry_logger_node:main',
        ],
    },
)
