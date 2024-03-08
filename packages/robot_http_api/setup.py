from setuptools import setup

package_name = 'robot_http_api'
packages = ['dt_robot_rest_api', 'hardware_test_robot_host']
package_dir = {'': 'include'}

setup(
    name=package_name,
    version='1.0.0',
    packages=packages,
    package_dir=package_dir,
    package_data={package_name: ['package.xml']},
    description='The robot_http_api meta package',
    maintainer='Andrea F. Daniele',
    maintainer_email='afdaniele@ttic.edu',
    license='GPLv3',
)
