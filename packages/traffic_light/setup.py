from setuptools import setup

package_name = 'traffic_light'
packages = ['traffic_light']
package_dir = {'': 'src'}

setup(
    name=package_name,
    version='1.0.0',
    packages=[''],
    package_dir=package_dir,
    package_data={package_name: ['package.xml']},
    description='TODO: Add a description of package `traffic_light` in `package.xml`.',
    maintainer='Mack',
    maintainer_email='mack@duckietown.or',
    license='GPLv3',
)
