from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'video_pub'

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
    maintainer='byl',
    maintainer_email='byl@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_pub_node = video_pub.video_pub_node:main',
            'hello_world_publisher = video_pub.hello_world_publisher:main'
        ],
    },
)
