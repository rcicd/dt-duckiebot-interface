from setuptools import setup
from glob import glob
import os
from utils.utils import map_recursive_files

package_name = 'camera_driver'
packages = ['camera_driver', 'hardware_test_camera']

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
    ] + [map_recursive_files('share/' + package_name,'config/' + x) for x in os.listdir('config')]
      + [map_recursive_files('share/' + package_name, 'include/' + x) for x in os.listdir('include')]
    ,
    description='TODO: Add a description of package `pi_camera` in `package.xml`.',
    maintainer='Andrea Censi',
    maintainer_email='acensi@idsc.mavt.ethz.ch',
    license='GPLv3',
    entry_points={
        'console_scripts': [
            'jetson_nano_camera_node = camera_driver.jetson_nano_camera_node:main',
            'raspberry_pi_64_camera_node = camera_driver.raspberry_pi_64_camera_node:main',
            'raspberry_pi_camera_node = camera_driver.raspberry_pi_camera_node:main',
        ],
    },
    install_requires=['setuptools'],
)
