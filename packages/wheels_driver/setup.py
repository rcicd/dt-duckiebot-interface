from setuptools import setup
from glob import glob
import os
from utils.utils import map_recursive_files

package_name = 'wheels_driver'
packages = ['wheels_driver', 'hardware_test_wheels']

setup(
    name=package_name,
    version='1.0.0',
    packages=packages,
    package_dir={'': 'include'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch')),
        ('share/' + package_name + '/src', glob('src/*.py')),
        ('lib/' + package_name, glob('src/*.py'))
    ] + [map_recursive_files('share/' + package_name, 'include/' + x) for x in os.listdir('include')]
    ,
    description='An interface to the robot\'s wheels',
    author='Andrea F. Daniele',
    author_email='afdaniele@ttic.edu',
    install_requires=['setuptools'],
    license='GPLv3',
    entry_points={
        'console_scripts': [
            'wheels_driver_node = wheels_driver.wheels_driver_node:main',
        ],
    },
)

