from setuptools import find_packages, setup
from glob import glob
import os
from utils.utils import map_recursive_files

package_name = 'joystick'

setup(
    name=package_name,
    version='1.0.0',
    install_requires=['setuptools'],
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch')),
    ] + [map_recursive_files('share/' + package_name, 'config/' + x) for x in os.listdir('config')]
    ,
    description='An interface to the display sensor',
    author='Andrea F. Daniele',
    author_email='afdaniele@ttic.edu',
    license='GPLv3',
    entry_points={
        'console_scripts': [],
    },
)
