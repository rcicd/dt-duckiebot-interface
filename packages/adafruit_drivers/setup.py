from setuptools import setup

package_name = 'adafruit_drivers'
packages = [
    # 'Adafruit_GPIO',
    'Adafruit_I2C',
    # 'Adafruit_MotorHAT',
    'Adafruit_PWM_Servo_Driver'
]
package_dir = {'': 'include'}

setup(
    name=package_name,
    version='1.0.0',
    packages=packages,
    package_dir={'': 'include'},
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