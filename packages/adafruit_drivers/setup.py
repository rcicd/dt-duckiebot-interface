from setuptools import setup
from glob import glob
import os

package_name = 'adafruit_drivers'
packages = [
    # 'Adafruit_GPIO',
    'Adafruit_I2C',
    # 'Adafruit_MotorHAT',
    'Adafruit_PWM_Servo_Driver'
]

setup(
    name=package_name,
    version='1.0.0',
    packages=packages,
    package_dir={'': 'include'},
    package_data={package_name: ['package.xml', 'include/**/*.py']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'include'), glob(os.path.join('include', '**/*.py')))
    ],
    install_requires=['setuptools'],
    description='Module containing all used drivers.',
    author='Dmitry Yershov',
    author_email='yershov@mit.edu',
    keywords=['ROS', 'ROS2'],
    license='GPLv3',
    entry_points={
        'console_scripts': [],
    },
)