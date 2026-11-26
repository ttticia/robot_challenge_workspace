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
    description='Scene understanding of room looks with the lab VLM (junction_trigger)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'junction_trigger = scene_understanding.junction_trigger:main',
        ],
    },
)
