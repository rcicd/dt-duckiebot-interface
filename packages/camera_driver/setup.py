from setuptools import setup
from glob import glob
import os

package_name = 'camera_driver'
packages = ['camera_driver', 'hardware_test_camera']

setup(
    name=package_name,
    version='1.0.0',
    packages=packages,
    package_dir={'': 'include'},
    package_data={package_name: ['package.xml', 'launch/*.launch', 'include/**/*.py','src/*.py', 'config/**/*.yaml']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '**/*.yaml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch'))),
        (os.path.join('share', package_name, 'include'), glob(os.path.join('include', '**/*.py'))),
        (os.path.join('share', package_name, 'src'), glob(os.path.join('src', '*.py')))
    ],
    description='TODO: Add a description of package `pi_camera` in `package.xml`.',
    maintainer='Andrea Censi',
    maintainer_email='acensi@idsc.mavt.ethz.ch',
    license='GPLv3',
)
