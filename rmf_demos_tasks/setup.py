from setuptools import setup

package_name = 'rmf_demos_tasks'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/'+package_name, [package_name+'/airport_docker_config.yaml']),
        ('share/'+package_name, [package_name+'/hotel_cleaner_config.yaml']),
    ],
    install_requires=['setuptools'],
    author='Grey',
    author_email='grey@openrobotics.org',
    zip_safe=True,
    maintainer='yadu',
    maintainer_email='yadunund@openrobotics.org',
    description='A package containing scripts for demos',
    license='Apache License Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'request_loop = rmf_demos_tasks.request_loop:main',
          'request_lift = rmf_demos_tasks.request_lift:main',
          'cancel_task = rmf_demos_tasks.cancel_task:main',
          'dispatch_loop = rmf_demos_tasks.dispatch_loop:main',
          'dispatch_action = rmf_demos_tasks.dispatch_action:main',
          'dispatch_patrol = rmf_demos_tasks.dispatch_patrol:main',
          'dispatch_delivery = rmf_demos_tasks.dispatch_delivery:main',
          'dispatch_clean = rmf_demos_tasks.dispatch_clean:main',
          'dispatch_go_to_place = rmf_demos_tasks.dispatch_go_to_place:main',
          'mock_docker = rmf_demos_tasks.mock_docker:main',
          'teleop_robot = rmf_demos_tasks.teleop_robot:main',
        ],
    },
)
