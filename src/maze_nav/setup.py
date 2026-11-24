from setuptools import setup

package_name = 'maze_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, 'touble_shooting'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Lite3 curtain detection: lidar gap check, lie-down inspection and a YOLO camera model',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'curtain_detector = maze_nav.curtain_detector:main',
            'curtain_yolo = maze_nav.curtain_yolo:main',
        ],
    },
)
