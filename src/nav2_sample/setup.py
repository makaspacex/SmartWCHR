from setuptools import find_packages, setup
import glob

package_name = 'nav2_sample'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/launch', glob.glob('launch/*')),
        ('share/' + package_name + '/maps', glob.glob('maps/*')),
        ('share/' + package_name + '/params', glob.glob('params/*')),
        ('share/' + package_name + '/rviz', glob.glob('rviz/*')),
        ('share/' + package_name + '/urdf', glob.glob('urdf/*')),
        ('share/' + package_name + '/worlds', glob.glob('worlds/*')),
        ('share/' + package_name + '/meshes', glob.glob('meshes/*')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='makafly',
    maintainer_email='izhangxm@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
