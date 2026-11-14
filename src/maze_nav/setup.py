from glob import glob

from setuptools import setup

package_name = 'maze_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Lite3 maze traversal with SLAM + Nav2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_scan = maze_nav.wall_scan:main',
            'maze_explorer = maze_nav.explorer:main',
            'cmd_vel_bridge = maze_nav.cmd_vel_bridge:main',
        ],
    },
)
