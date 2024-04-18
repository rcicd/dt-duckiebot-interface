from setuptools import setup
from glob import glob
import os
from utils.utils import map_recursive_files

package_name = 'display_renderers'
packages = ['health_renderer_node', 'networking_renderer_node', 'robot_info_renderer_node']

setup(
    name=package_name,
    version='1.0.0',
    packages=[''],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch')),
        ('share/' + package_name + '/src', glob('src/*.py')),
        ('lib/' + package_name, glob('src/*.py'))
    ] + [map_recursive_files('share/' + package_name, 'config/' + x) for x in os.listdir('config')],
    install_requires=['setuptools'],
    description='An interface to the display sensor',
    author='Andrea F. Daniele',
    author_email='afdaniele@ttic.edu',
    license='GPLv3',
    entry_points={
        'console_scripts': [
            'health_renderer_node = display_renderers.health_renderer_node:main',
            'networking_renderer_node = display_renderers.networking_renderer_node:main',
            'robot_info_renderer_node = display_renderers.robot_info_renderer_node:main',
        ],
    },
)
