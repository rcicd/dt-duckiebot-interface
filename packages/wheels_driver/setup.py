from setuptools import setup

package_name = 'wheels_driver'
packages = ['wheels_driver', 'hardware_test_wheels']
package_dir = {'': 'include'}

setup(
    name=package_name,
    version='1.0.0',
    packages=packages,
    package_dir=package_dir,
    package_data={package_name: ['package.xml']},
    description='An interface to the robot\'s wheels',
    author='Andrea F. Daniele',
    author_email='afdaniele@ttic.edu',
    license='GPLv3',
)

