from setuptools import setup
from glob import glob
import os
from utils.utils import map_recursive_files

package_name = 'button_driver'
packages = ['button_driver', 'hardware_test_button']

setup(
    name=package_name,
    version='1.0.0',
    packages=packages,
    package_dir={'': 'include'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch'))),
        (os.path.join('share', package_name, 'src'), glob(os.path.join('src', '*.py'))),
        (os.path.join('lib', package_name), glob(os.path.join('src', '*.py')))
    ] + [map_recursive_files(os.path.join('share', package_name), os.path.join('config', x)) for x in os.listdir('config')]
      + [map_recursive_files(os.path.join('share', package_name), os.path.join('include', x)) for x in os.listdir('include')]
    ,
    description='An interface to the display sensor',
    author='Andrea F. Daniele',
    author_email='afdaniele@ttic.edu',
    install_requires=['setuptools'],
    license='GPLv3',
    entry_points={
        'console_scripts': [
            'button_driver_node = button_driver.button_driver_node:main',
        ],
    },
)