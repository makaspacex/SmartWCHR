from setuptools import find_packages, setup

package_name = 'gesture_recognition'

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
    maintainer='jetson',
    maintainer_email='qyciuui@163.com',
    description='Gesture recognition from camera images',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_recognition_node = gesture_recognition.gesture_recognition_node:main',
            'hands_Visualization_node = gesture_recognition.handsVisualizationNode:main'
        ],
    },
)
