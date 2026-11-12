from glob import glob
from setuptools import find_packages, setup

package_name = 'navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Group 4',
    maintainer_email='user@example.com',
    description='SLAM Toolbox + Nav2 mapping for the Lite3, with automatic map saving.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_autosaver = navigation.map_autosaver:main',
            'cmd_vel_bridge = navigation.cmd_vel_bridge:main',
        ],
    },
)
