from setuptools import setup

package_name = 'wheel_encoder'
packages = ['wheel_encoder', 'hardware_test_wheel_encoder']
package_dir = {'': 'include'}

setup(
    name=package_name,
    version='1.0.0',
    packages=packages,
    package_dir=package_dir,
    package_data={package_name: ['package.xml']},
    description='A ROS interface to the wheel encoders',
    author='Andrea F. Daniele',
    author_email='afdaniele@ttic.edu',
    license='GPLv3',
)
