from setuptools import setup, find_packages

package_name = 'rmf_demos_adapters'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Charayaphan Nakorn Boon Han',
    maintainer_email='charayaphan.nakorn.boon.han@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            ('deliveryrobot_gps_fleet_adapter='
                'rmf_demos_adapters.DeliveryRobot'
                '.deliveryrobot_gps_fleet_adapter:main')
        ],
    },
)
