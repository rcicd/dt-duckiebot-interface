from setuptools import setup
from glob import glob
import os
from utils.utils import map_recursive_files

package_name = 'wheel_encoder'
packages = ['wheel_encoder', 'hardware_test_wheel_encoder']

setup(
    name=package_name,
    version='1.0.0',
    packages=packages,
    install_requires=['setuptools'],
    package_dir={'': 'include'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch')),
        ('share/' + package_name + '/src', glob('src/*.py')),
        ('lib/' + package_name, glob('src/*.py'))
    ] + [map_recursive_files('share/' + package_name,'config/' + x) for x in os.listdir('config')]
      + [map_recursive_files('share/' + package_name, 'include/' + x) for x in os.listdir('include')]
    ,
    description='A ROS interface to the wheel encoders',
    author='Andrea F. Daniele',
    author_email='afdaniele@ttic.edu',
    license='GPLv3',
    entry_points={
        'console_scripts': [
            'wheel_encoder_node = wheel_encoder.wheel_encoder_node:main',
        ],
    },
)
