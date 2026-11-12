from glob import glob
from setuptools import find_packages, setup

setup(name='lite3_mission', version='0.1.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/lite3_mission']),
                  ('share/lite3_mission', ['package.xml', 'README.md']),
                  ('share/lite3_mission/config', glob('config/*.yaml')),
                  ('share/lite3_mission/launch', glob('launch/*.py'))],
      install_requires=['setuptools'], zip_safe=True, license='MIT',
      maintainer='Group 4', maintainer_email='user@example.com',
      description='Mission coordination above the existing Lite3 lab navigation stack',
      entry_points={'console_scripts': [
          'mission = lite3_mission.mission:main',
          'velocity_gate = lite3_mission.velocity_gate:main',
          'observations = lite3_mission.observations:main',
          'qr = lite3_mission.qr:main',
          'scene_client = lite3_mission.scene_client:main',
          'team_qr_adapter = lite3_mission.team_qr_adapter:main',
      ]})
