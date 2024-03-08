from setuptools import setup

package_name = 'duckiebot_interface'
packages = ['dt_duckiebot_hardware_tests']
package_dir = {'': 'include'}

setup(
    name=package_name,
    version='1.0.0',
    packages=packages,
    package_dir=package_dir,
    package_data={package_name: ['package.xml']},
    description='The duckiebot_interface meta package',
    maintainer='Mack',
    maintainer_email='mack@duckietown.org',
    license='GPLv3',
)

