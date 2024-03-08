from setuptools import setup

package_name = 'led_driver'
packages = ['rgb_led', 'hardware_test_led']
package_dir = {'': 'include'}

setup(
    name=package_name,
    version='1.0.0',
    packages=packages,
    package_dir=package_dir,
    package_data={package_name: ['package.xml']},
    description='TODO: description for `led_driver` package.',
    maintainer='Andrea Censi',
    maintainer_email='acensi@idsc.mavt.ethz.ch',
    license='GPLv3',
)
