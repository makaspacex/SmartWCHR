from setuptools import find_packages, setup
from glob import glob

package_name = 'yolomix'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/weights', glob('weights/*')),
        ('share/' + package_name + '/data', glob('data/*')),
        ('share/' + package_name + '/config', glob('config/*')),
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
            'yolomix_node = yolomix.yolomix_node:main',
            'visualization_node = yolomix.VisualizationNode:main',
            'reid_node = yolomix.yolomix_reid:main'
        ],
        
    },
)
