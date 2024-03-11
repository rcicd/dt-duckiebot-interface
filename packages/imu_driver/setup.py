from setuptools import setup
from glob import glob
import os

package_name = 'imu_driver'
packages = ['hardware_test_imu']

setup(
    name=package_name,
    version='1.0.0',
    packages=packages,
    package_dir={'': 'include'},
    package_data={package_name: ['package.xml', 'launch/*.launch', 'include/**/*.py','src/*.py', 'config/*/*.yaml']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch'))),
        (os.path.join('share', package_name, 'include'), glob(os.path.join('include', '**/*.py'))),
        (os.path.join('share', package_name, 'src'), glob(os.path.join('src', '*.py')))
    ],
    description='An interface to the IMU sensor',
    author='Jason Hu',
    author_email='shhu@ethz.ch',
    license='GPLv3',
)
