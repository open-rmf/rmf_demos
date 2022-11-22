import os
from glob import glob
from setuptools import setup

package_name = 'rmf_demos_door_adapter'

setup(
    name=package_name,
    version='2.0.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luca Della Vedova',
    maintainer_email='luca@openrobotics.org',
    description='Demo door adapter to be used with rmf_demos simulations',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'door_adapter = rmf_demos_door_adapter.door_adapter:main',
            'door_manager = rmf_demos_door_adapter.door_manager:main'
        ],
    },
)
