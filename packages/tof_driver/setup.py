from setuptools import setup

package_name = 'tof_driver'
packages = ['hardware_test_tof', 'tof_accuracy']
package_dir = {'': 'include'}

setup(
    name=package_name,
    version='1.0.0',
    packages=packages,
    package_dir=package_dir,
    package_data={package_name: ['package.xml']},
    description='An interface to the ToF sensor',
    author='Andrea F. Daniele',
    author_email='afdaniele@ttic.edu',
    license='GPLv3',
)
