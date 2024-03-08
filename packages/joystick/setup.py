from setuptools import setup

package_name = 'joystick'
# packages = ['health_renderer_node', 'networking_renderer_node', 'robot_info_renderer_node']
package_dir = {'': '.'}

setup(
    name=package_name,
    version='1.0.0',
    packages=[''],
    package_dir=package_dir,
    package_data={package_name: ['package.xml']},
    description='An interface to the display sensor',
    author='Andrea F. Daniele',
    author_email='afdaniele@ttic.edu',
    license='GPLv3',
    entry_points={
        'console_scripts': [],
    },
)
