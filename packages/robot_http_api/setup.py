from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'robot_http_api'
packages = ['dt_robot_rest_api', 'hardware_test_robot_host']

def get_all_files(dest, directory):
    result = []
    for root, dirs, files in os.walk(directory):
        if files:
            result += [(os.path.join(dest, root), glob(os.path.join(root, '*.*')))]
    return result


setup(
    name=package_name,
    version='1.0.0',
    packages = find_packages(where='include'),
    package_dir={
        'hardware_test_robot_host': 'include/hardware_test_robot_host',
        'dt_robot_rest_api': 'include/dt_robot_rest_api',
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.launch')),
        (f'share/{package_name}/src', glob('src/*.py')),
        (f'lib/{package_name}', glob('src/*.py'))
    ] + get_all_files(f'share/{package_name}', 'include')
    ,
    description='The robot_http_api meta package',
    maintainer='Andrea F. Daniele',
    maintainer_email='afdaniele@ttic.edu',
    license='GPLv3',
    entry_points={
        'console_scripts': [
            'robot_http_api_node = robot_http_api.robot_http_api_node:main',
        ],
    },
)
