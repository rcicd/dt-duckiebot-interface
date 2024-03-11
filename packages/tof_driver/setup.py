from setuptools import setup
import os
from glob import glob

package_name = 'tof_driver'
packages = ['hardware_test_tof', 'tof_accuracy']

setup(
    name=package_name,
    version='1.0.0',
    packages=packages,
    package_dir={'': 'include'},
    package_data={package_name: ['package.xml', 'launch/*.launch', 'include/**/*.py','src/*.py', 'config/**/*.yaml']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '**/.launch'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch'))),
        (os.path.join('share', package_name, 'include'), glob(os.path.join('include', '**/*.py'))),
        (os.path.join('share', package_name, 'src'), glob(os.path.join('src', '*.py')))
    ],
    description='An interface to the ToF sensor',
    author='Andrea F. Daniele',
    author_email='afdaniele@ttic.edu',
    license='GPLv3',
)
