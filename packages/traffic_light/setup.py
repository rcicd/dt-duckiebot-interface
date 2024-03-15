from setuptools import setup
from glob import glob
import os

package_name = 'traffic_light'
packages = ['traffic_light']
def get_all_files(dest, directory):
    return (os.path.join(dest, directory), [y for x in os.walk(directory) for y in glob(os.path.join(x[0], '*'))])

setup(
    name=package_name,
    version='1.0.0',
    packages=[''],
    install_requires=['setuptools'],
    # package_dir={'': 'include'},
    package_data={package_name: ['package.xml', 'launch/*.launch','src/*.py', 'config/*/*.yaml']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '**', '*.yaml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch'))),
        (os.path.join('share', package_name, 'src'), glob(os.path.join('src', '*.py')))
    ] + [get_all_files(os.path.join('share', package_name), os.path.join('config', x)) for x in os.listdir('config')]
    ,
    description='TODO: Add a description of package `traffic_light` in `package.xml`.',
    maintainer='Mack',
    maintainer_email='mack@duckietown.or',
    license='GPLv3',
    entry_points={
        'console_scripts': [
            'traffic_light_node.py = traffic_light.traffic_light_node:main',
        ],
    },
)
