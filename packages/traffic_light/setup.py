from setuptools import setup
from glob import glob
import os

package_name = 'traffic_light'
packages = ['traffic_light']

setup(
    name=package_name,
    version='1.0.0',
    packages=[''],
    package_data={package_name: ['package.xml', 'launch/*.launch','src/*.py', 'config/**/*.yaml']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '**/.launch'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch'))),
        (os.path.join('share', package_name, 'src'), glob(os.path.join('src', '*.py')))
    ],
    description='TODO: Add a description of package `traffic_light` in `package.xml`.',
    maintainer='Mack',
    maintainer_email='mack@duckietown.or',
    license='GPLv3',
)
