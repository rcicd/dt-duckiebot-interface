from setuptools import setup
from glob import glob
import os

package_name = 'imu_driver'
packages = ['hardware_test_imu']
def get_all_files(dest, directory):
    return (os.path.join(dest, directory), [y for x in os.walk(directory) for y in glob(os.path.join(x[0], '*'))])

setup(
    name=package_name,
    version='1.0.0',
    packages=packages,
    package_dir={'': 'include'},
    install_requires=['setuptools'],
    package_data={package_name: ['package.xml', 'launch/*.launch', 'include/*/*.py','src/*.py', 'config/*/*.yaml']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '**', '*.yaml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch'))),
        # (os.path.join('share', package_name, 'include'), glob(os.path.join('include', '**', '*.py'))),
        (os.path.join('share', package_name, 'src'), glob(os.path.join('src', '*.py')))
    ] + [get_all_files(os.path.join('share', package_name), os.path.join('config', x)) for x in os.listdir('config')]
               + [get_all_files(os.path.join('share', package_name), os.path.join('include', x)) for x in os.listdir('include')]
    ,
    description='An interface to the IMU sensor',
    author='Jason Hu',
    author_email='shhu@ethz.ch',
    license='GPLv3',
    entry_points={
        'console_scripts': [
            'imu_node.py = imu_driver.imu_node:main',
        ],
    },
)
