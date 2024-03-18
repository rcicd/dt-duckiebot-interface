from setuptools import setup
from utils.utils import map_recursive_files
from glob import glob
import os

package_name = 'traffic_light'
packages = ['traffic_light']

setup(
    name=package_name,
    version='1.0.0',
    packages=[''],
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch')),
        ('share/' + package_name + '/src', glob('src/*.py')),
        ('lib/' + package_name, glob('src/*.py'))
    ] + [map_recursive_files('share/' + package_name,'config/' + x) for x in os.listdir('config')]
    ,
    description='TODO: Add a description of package `traffic_light` in `package.xml`.',
    maintainer='Mack',
    maintainer_email='mack@duckietown.or',
    license='GPLv3',
    entry_points={
        'console_scripts': [
            'traffic_light_node = traffic_light.traffic_light_node:main',
        ],
    },
)
