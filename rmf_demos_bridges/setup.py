from setuptools import setup

package_name = 'rmf_demos_bridges'

setup(
    name=package_name,
    version='2.11.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yadunund',
    maintainer_email='yadunund@openrobotics.org',
    description='Nodes for bridging between different communication stacks',
    license='Apache License 2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'fleet_socketio_bridge='
            'rmf_demos_bridges.fleet_socketio_bridge:main',
            'fleet_robotmanager_mqtt_bridge='
            'rmf_demos_bridges.fleet_robotmanager_mqtt_bridge:main',
        ],
    },
)
