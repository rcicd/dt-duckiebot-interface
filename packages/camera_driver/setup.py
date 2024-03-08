from setuptools import setup

package_name = 'camera_driver'
packages = ['camera_driver', 'hardware_test_camera']
package_dir = {'': 'include'}

setup(
    name=package_name,
    version='1.0.0',
    packages=packages,
    package_dir=package_dir,
    package_data={package_name: ['package.xml']},
    description='TODO: Add a description of package `pi_camera` in `package.xml`.',
    maintainer='Andrea Censi',
    maintainer_email='acensi@idsc.mavt.ethz.ch',
    license='GPLv3',
)
