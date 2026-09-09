from glob import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'rmf_demos_path_guide_adapter'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['config.yaml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.xml'),
        ),
    ],
    install_requires=['setuptools', 'fastapi>=0.79.0', 'uvicorn>=0.18.2'],
    zip_safe=True,
    maintainer='Leong Teck Tey',
    maintainer_email='leong_teck_tey@cgh.com.sg',
    description='Path Guide fleet adapter for RMF Demos robots, which hands '
    'the fleet manager an entire path in one request instead of one waypoint '
    'at a time',
    license='Apache License 2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'fleet_adapter='
            'rmf_demos_path_guide_adapter.fleet_adapter:main',
            'fleet_manager='
            'rmf_demos_path_guide_adapter.fleet_manager:main',
        ],
    },
)
