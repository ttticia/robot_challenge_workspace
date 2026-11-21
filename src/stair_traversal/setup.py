from setuptools import setup

setup(
    name='stair_traversal',
    version='0.0.0',
    py_modules=['stair_traversal', 'stair_geometry'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/stair_traversal']),
        ('share/stair_traversal', ['package.xml']),
        ('share/stair_traversal/launch', ['launch/scene_stairs.launch.py']),
    ],
    install_requires=['setuptools'],
    entry_points={'console_scripts': ['stair_traversal = stair_traversal:main']},
)
