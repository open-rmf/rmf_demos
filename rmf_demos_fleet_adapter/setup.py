import os
from shutil import rmtree
from subprocess import check_call

from glob import glob
from setuptools import setup, find_packages
from distutils.command.build import build as _build

package_name = 'rmf_demos_fleet_adapter'
vendor_pkgs = ['fastapi==0.79.0', 'uvicorn==0.18.2']
here = os.path.dirname(__file__)


class build(_build):
    def run(self):
        vendor_dir = f'{here}/{package_name}/_vendor'
        rmtree(vendor_dir, ignore_errors=True)
        check_call(('pip3', 'install', '-I', '-t', vendor_dir, *vendor_pkgs))
        with open(f'{vendor_dir}/__init__.py', 'w'):
            pass
        super().run()

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
    cmdclass={'build': build},
)
