from setuptools import setup
from glob import glob
import os

package_name = 'duckiebot_interface'
packages = ['dt_duckiebot_hardware_tests']
package_dir = {'': 'include'}

def get_all_files(dest, directory):
    return (os.path.join(dest, directory), [y for x in os.walk(directory) for y in glob(os.path.join(x[0], '*'))])

setup(
    name=package_name,
    version='1.0.0',
    packages=packages,
    install_requires=['setuptools'],
    package_dir={'': 'include'},
    package_data={package_name: ['package.xml', 'launch/*.launch', 'include/*/*.py', 'urdf/*.xacro', 'images/icons/*.png']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'images/icons'), glob(os.path.join('icons', '*.png'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch'))),
        # (os.path.join('share', package_name, 'include'), glob(os.path.join('include', '**', '*.py')))
    ] + [get_all_files(os.path.join('share', package_name), os.path.join('include', x)) for x in os.listdir('include')]
    ,
    description='The duckiebot_interface meta package',
    maintainer='Mack',
    maintainer_email='mack@duckietown.org',
    license='GPLv3',
)

