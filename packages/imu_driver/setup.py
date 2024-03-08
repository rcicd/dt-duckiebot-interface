from setuptools import setup

package_name = 'imu_driver'
packages = ['hardware_test_imu']
package_dir = {'': 'include'}

setup(
    name=package_name,
    version='1.0.0',
    packages=packages,
    package_dir=package_dir,
    package_data={package_name: ['package.xml']},
    description='An interface to the IMU sensor',
    author='Jason Hu',
    author_email='shhu@ethz.ch',
    license='GPLv3',
)
