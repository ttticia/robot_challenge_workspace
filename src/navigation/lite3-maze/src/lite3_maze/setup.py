from setuptools import setup
from glob import glob
setup(name='lite3_maze', version='0.1.0', packages=['lite3_maze'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/lite3_maze']),
                ('share/lite3_maze', ['package.xml']),
                ('share/lite3_maze/config', glob('config/*.yaml')),
                ('share/lite3_maze/models', glob('models/*')),
                ('share/lite3_maze/launch', glob('launch/*.py'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Lite3 team', maintainer_email='team@example.com',
    description='Sensor-driven rough cell-maze exploration for Lite3', license='MIT',
    entry_points={'console_scripts': [
        'navigator = lite3_maze.navigator:main',
        'mode_bridge = lite3_maze.mode_bridge:main',
        'yolo = lite3_maze.yolo:main',
        'detections = lite3_maze.detections:main']})
