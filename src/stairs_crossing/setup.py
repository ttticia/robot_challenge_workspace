from glob import glob

from setuptools import setup

package_name = 'stairs_crossing'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Stairs crossing in the AI gait (copy of the maze_explore explorer)',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'explorer_stairs = stairs_crossing.explorer_stairs:main',
        'stair_watch = stairs_crossing.stair_watch:main',
        'keyboard_teleop = stairs_crossing.keyboard_teleop:main',
    ]},
)
