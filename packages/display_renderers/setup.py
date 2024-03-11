from setuptools import setup
from glob import glob
import os

package_name = 'display_renderers'
packages = ['health_renderer_node', 'networking_renderer_node', 'robot_info_renderer_node']

setup(
    name=package_name,
    version='1.0.0',
    packages=[''],
    package_data={package_name: ['package.xml', 'launch/*.launch','src/*.py', 'config/**/*.yaml']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '**/*.yaml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch'))),
        (os.path.join('share', package_name, 'src'), glob(os.path.join('src', '*.py')))
    ],
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
