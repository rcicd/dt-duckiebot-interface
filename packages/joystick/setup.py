from setuptools import setup
from glob import glob
import os

package_name = 'joystick'
def get_all_files(dest, directory):
    return (os.path.join(dest, directory), [y for x in os.walk(directory) for y in glob(os.path.join(x[0], '*'))])

setup(
    name=package_name,
    version='1.0.0',
    install_requires=['setuptools'],
    packages=[''],
    package_data={package_name: ['package.xml', 'launch/*.launch', 'config/*/*.yaml']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch'))),
        # (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '**', '*.yaml')))
    ] + [get_all_files(os.path.join('share', package_name), os.path.join('config', x)) for x in os.listdir('config')]
    ,
    description='An interface to the display sensor',
    author='Andrea F. Daniele',
    author_email='afdaniele@ttic.edu',
    license='GPLv3',
    entry_points={
        'console_scripts': [],
    },
)
