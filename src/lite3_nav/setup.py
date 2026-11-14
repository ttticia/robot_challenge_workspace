from glob import glob
from setuptools import find_packages, setup

setup(name='lite3_nav', version='0.1.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/lite3_nav']),
                  ('share/lite3_nav', ['package.xml', 'README.md']),
                  ('share/lite3_nav/config', glob('config/*.yaml') + glob('config/*.xml')),
                  ('share/lite3_nav/launch', glob('launch/*.py')),
                  ('share/lite3_nav/scripts', glob('scripts/*.sh'))],
      install_requires=['setuptools'], zip_safe=True, license='MIT',
      maintainer='Group 4', maintainer_email='user@example.com',
      description='Navigation workstation package on top of the unchanged lite3_mission package',
      entry_points={'console_scripts': [
          'mission = lite3_nav.mission:main',
          'velocity_gate = lite3_nav.velocity_gate:main',
      ]})
