import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'system_monitor'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'psutil'],
    zip_safe=True,
    maintainer='System Monitor Maintainer',
    maintainer_email='maintainer@example.com',
    description=('ROS 2 package for monitoring system resources '
                 '(CPU, memory, disk usage) and publishing them as topics'),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'system_monitor_node = system_monitor.system_monitor_node:main',
            'resource_logger_node = system_monitor.resource_logger_node:main',
        ],
    },
)
