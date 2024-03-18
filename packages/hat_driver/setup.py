from setuptools import setup
from glob import glob
import os
from utils.utils import map_recursive_files

package_name = 'hat_driver'
packages = ['hat_driver']

setup(
    name=package_name,
    version='1.0.0',
    packages=packages,
    package_dir={'': 'include'},
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]+ [map_recursive_files('share/' + package_name, 'include/' + x) for x in os.listdir('include')]
    ,
    description='An interface to the DTHUT board',
    author='Andrea F. Daniele',
    author_email='afdaniele@ttic.edu',
    license='GPLv3',
)
