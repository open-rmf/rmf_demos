from setuptools import setup
from glob import glob
import sys

package_name = 'rmf_demos_panel'
py_version = ".".join(map(str, sys.version_info[:2]))
site_pkgs_path = 'lib/python' + py_version + '/site-packages/rmf_demos_panel'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            site_pkgs_path + '/templates',
            glob(package_name + '/templates/*')
        ),
        (
            site_pkgs_path + '/static/dist',
            glob(package_name + '/static/dist/*.*')
        )
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='youliang',
    maintainer_email='youliang@openrobotics.org',
    description='RMF demo web based panel',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_api_server=rmf_demos_panel.simple_api_server:main',
        ],
    },
)
