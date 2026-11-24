from setuptools import setup

package_name = 'scene_understanding'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Room captures for the VLM (room_scene) and mission task reports (task_reporter)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'junction_trigger = scene_understanding.junction_trigger:main',
            'room_scene = scene_understanding.room_scene:main',
            'task_reporter = scene_understanding.task_reporter:main',
        ],
    },
)
