from setuptools import setup
from glob import glob
import os
from utils.utils import map_recursive_files

package_name = 'duckiebot_interface'
packages = ['dt_duckiebot_hardware_tests']
package_dir = {'': 'include'}


setup(
    name=package_name,
    version='1.0.0',
    packages=packages,
    install_requires=['setuptools'],
    package_dir={'': 'include'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/images/icons', glob('images/icons/*.png')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/launch', glob('launch/*.launch')),
    ] + [map_recursive_files('share/' + package_name, 'include/' + x) for x in os.listdir('include')]
    ,
    description='The duckiebot_interface meta package',
    maintainer='Mack',
    maintainer_email='mack@duckietown.org',
    license='GPLv3',
)

