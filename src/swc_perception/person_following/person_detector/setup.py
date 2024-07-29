from setuptools import find_packages, setup
from glob import glob
import os


package_name = 'person_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*')),
        (os.path.join('share', package_name, 'data'), glob('resource/data/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='qyciuui@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'person_detector_node = person_detector.person_detector_node:main',
            'persons_visualize_node = person_detector.persons_visualizer_node:main',
            'person_tracker_node = person_detector.person_tracker_node:main'
        ],
    },
)
