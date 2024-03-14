from setuptools import setup
from glob import glob
import os

package_name = 'duckiebot_interface'
packages = ['dt_duckiebot_hardware_tests']
package_dir = {'': 'include'}

setup(
    name=package_name,
    version='1.0.0',
    packages=packages,
    # package_dir={'': 'include'},
    package_data={package_name: ['package.xml', 'launch/*.launch', 'include/**/*.py', 'urdf/*.xacro', 'images/icons/*.png']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'images'), glob(os.path.join('images', 'icons/*.png'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch'))),
        (os.path.join('share', package_name, 'include'), glob(os.path.join('include', '**/*.py')))
    ],
    description='The duckiebot_interface meta package',
    maintainer='Mack',
    maintainer_email='mack@duckietown.org',
    license='GPLv3',
)

