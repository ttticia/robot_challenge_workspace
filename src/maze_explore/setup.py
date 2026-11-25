from glob import glob

from setuptools import setup

package_name = 'maze_explore'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/models/curtain_int8_320x256_openvino_model',
         glob('models/curtain_int8_320x256_openvino_model/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Navigation / exploration stack for the Lite3 maze run (v2)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'wall_scan = maze_explore.wall_scan:main',
        'maze_explorer = maze_explore.explorer:main',
        'curtain_node = maze_explore.curtain_node:main',
    ]},
)
