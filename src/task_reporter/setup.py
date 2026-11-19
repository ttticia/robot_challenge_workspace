from setuptools import setup

setup(
    name='task_reporter',
    version='0.1.0',
    packages=['task_reporter'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/task_reporter']),
        ('share/task_reporter', ['package.xml']),
    ],
    install_requires=['setuptools', 'requests'],
    entry_points={'console_scripts': [
        'task_reporter = task_reporter.node:main',
    ]},
)
