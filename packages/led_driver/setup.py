from setuptools import setup
from glob import glob
import os

package_name = 'led_driver'
packages = ['rgb_led', 'hardware_test_led']

def get_all_files(dest, directory):
    return (os.path.join(dest, directory), [y for x in os.walk(directory) for y in glob(os.path.join(x[0], '*'))])


setup(
    name=package_name,
    version='1.0.0',
    install_requires=['setuptools'],
    packages=packages,
    package_dir={'': 'include'},
    package_data={package_name: ['package.xml', 'launch/*.launch', 'include/*/*.py','src/*.py', 'config/*/*.yaml']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '**', '*.launch'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch'))),
        # (os.path.join('share', package_name, 'include'), glob(os.path.join('include', '**', '*.py'))),
        (os.path.join('share', package_name, 'src'), glob(os.path.join('src', '*.py')))
    ] + [get_all_files(os.path.join('share', package_name), os.path.join('config', x)) for x in os.listdir('config')]
               + [get_all_files(os.path.join('share', package_name), os.path.join('include', x)) for x in os.listdir('include')]
    ,
    description='TODO: description for `led_driver` package.',
    maintainer='Andrea Censi',
    maintainer_email='acensi@idsc.mavt.ethz.ch',
    license='GPLv3',
    entry_points={
        'console_scripts': [
            'led_driver_node.py = led_driver.led_driver_node:main',
        ],
    },
)
