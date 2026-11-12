from setuptools import find_packages, setup

package_name = 'scene_understanding'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'image_subscriber = scene_understanding.image_subscriber:main',
            'image_folder_publisher = '
            'scene_understanding.image_folder_publisher:main',
            'junction_trigger = scene_understanding.junction_trigger:main',
            'vlm_test = scene_understanding.vlm_client:main',
            'vlm_api_test = scene_understanding.vlm_api_client:main',
        ],
    },
)
