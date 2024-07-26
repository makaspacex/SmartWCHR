import os
from glob import glob

from setuptools import setup

package_name = 'swc_nav_bringup'

setup(
    name=package_name,
    version='0.0.0',
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'map'), glob('map/**')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/**')),
        (os.path.join('share', package_name, 'PCD'), glob('PCD/**')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/**')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/**')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/**')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
