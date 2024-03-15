from setuptools import setup
from glob import glob
import os

package_name = 'robot_http_api'
packages = ['dt_robot_rest_api', 'hardware_test_robot_host']
def get_all_files(dest, directory):
    result = []
    for x in os.walk(directory):
        for y in glob(os.path.join(x[0], '*')):
            if os.path.isfile(y):
                result += (os.path.join(dest, directory),y)
            else:
                result += get_all_files(os.path.join(dest, y), os.path.join(directory, y))
    return result

install_files = []
for x in os.listdir('include'):
    install_files += get_all_files(os.path.join('share', package_name), os.path.join('include', x))

setup(
    name=package_name,
    version='1.0.0',
    packages=packages,
    install_requires=['setuptools'],
    package_dir={'': 'include'},
    package_data={package_name: ['package.xml', 'launch/*.launch', 'include/**/*.py','src/*.py']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch'))),
        # (os.path.join('share', package_name, 'include'), glob(os.path.join('include', '**', '*.py'))),
        (os.path.join('share', package_name, 'src'), glob(os.path.join('src', '*.py')))
    ]
               # + install_files
    ,
    description='The robot_http_api meta package',
    maintainer='Andrea F. Daniele',
    maintainer_email='afdaniele@ttic.edu',
    license='GPLv3',
    entry_points={
        'console_scripts': [
            'robot_http_api_node.py = robot_http_api.robot_http_api_node:main',
        ],
    },
)
