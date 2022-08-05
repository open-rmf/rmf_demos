import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'rmf_demos_fleet_adapter'

setup(
    name=package_name,
    version='1.4.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['config.yaml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.xml')),

    ],
    install_requires=['setuptools', 'fastapi>=0.79.0', 'uvicorn>=0.18.2'],
    zip_safe=True,
    maintainer='Xi Yu Oh',
    maintainer_email='xiyu@openrobotics.org',
    description='Fleet adapters for interfacing with RMF Demos robots with a '
                'fleet manager via REST API',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_adapter=rmf_demos_fleet_adapter.fleet_adapter:main',
            'fleet_manager=rmf_demos_fleet_adapter.fleet_manager:main',
        ],
    },
)
