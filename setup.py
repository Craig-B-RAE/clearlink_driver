from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'clearlink_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'pycomm3>=1.2.0',
    ],
    zip_safe=True,
    maintainer='Craig',
    maintainer_email='craig@example.com',
    description='ROS2 driver for Teknic ClearLink EtherNet/IP motor controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'clearlink_node = clearlink_driver.clearlink_node:main',
        ],
    },
)
