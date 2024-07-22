from setuptools import setup
from glob import glob

package_name = 'virtual_joystick'

setup(
    name=package_name,
    version='1.0.0',
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch*')),
        ('share/' + package_name + '/src', glob('src/*.py')),
        ('lib/' + package_name, glob('src/*.py'))
    ],
    description='Package for communication with virtual joystick',
    author='Ilia Nechaev',
    author_email='lelikk2002@gmail.com',
    license='GPLv3',
    entry_points={
        'console_scripts': [
            'virtual_joystick = virtual_joystick.virtual_joystick_node:main',
        ],
    },
)
