from setuptools import setup
from glob import glob
import os

package_name = 'robot_http_api'
packages = ['dt_robot_rest_api', 'hardware_test_robot_host']

setup(
    name=package_name,
    version='1.0.0',
    packages=packages,
    package_dir={'': 'include'},
    package_data={package_name: ['package.xml', 'launch/*.launch', 'include/**/*.py','src/*.py']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch'))),
        (os.path.join('share', package_name, 'include'), glob(os.path.join('include', '**/*.py'))),
        (os.path.join('share', package_name, 'src'), glob(os.path.join('src', '*.py')))
    ],
    description='The robot_http_api meta package',
    maintainer='Andrea F. Daniele',
    maintainer_email='afdaniele@ttic.edu',
    license='GPLv3',
)
