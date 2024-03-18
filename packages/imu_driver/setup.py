from setuptools import setup
from glob import glob
import os
from utils.utils import map_recursive_files

package_name = 'imu_driver'
packages = ['hardware_test_imu']

setup(
    name=package_name,
    version='1.0.0',
    packages=packages,
    package_dir={'': 'include'},
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch')),
        ('share/' + package_name + '/src', glob('src/*.py')),
        ('lib/' + package_name, glob('src/*.py'))
    ] + [map_recursive_files('share/' + package_name, 'config/' + x) for x in os.listdir('config')]
               + [map_recursive_files('share/' + package_name, 'include/' + x) for x in os.listdir('include')]
    ,
    description='An interface to the IMU sensor',
    author='Jason Hu',
    author_email='shhu@ethz.ch',
    license='GPLv3',
    entry_points={
        'console_scripts': [
            'imu_node = imu_driver.imu_node:main',
        ],
    },
)
