from setuptools import setup
from glob import glob
import os

package_name = 'hat_driver'
packages = ['hat_driver']

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
    description='An interface to the DTHUT board',
    author='Andrea F. Daniele',
    author_email='afdaniele@ttic.edu',
    license='GPLv3',
)
