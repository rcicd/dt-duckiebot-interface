from setuptools import setup

package_name = 'display_renderers'
packages = ['health_renderer_node', 'networking_renderer_node', 'robot_info_renderer_node']
package_dir = {'': 'src'}

setup(
    name=package_name,
    version='1.0.0',
    packages=[''],
    package_dir=package_dir,
    install_requires=['setuptools'],
    description='An interface to the display sensor',
    author='Andrea F. Daniele',
    author_email='afdaniele@ttic.edu',
    license='GPLv3',
    entry_points={
        'console_scripts': [
            'health_renderer_node = display_renderers.health_renderer_node:main',
            'networking_renderer_node = display_renderers.networking_renderer_node:main',
            'robot_info_renderer_node = display_renderers.robot_info_renderer_node:main',
        ],
    },
)
