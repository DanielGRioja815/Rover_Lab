from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'rover_ros2'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', glob('launch/*.py')))
data_files.append(('share/' + package_name + '/worlds', glob('worlds/*.wbt')))
data_files.append(('share/' + package_name + '/resource', glob('resource/*')))
data_files.append(('share/' + package_name, ['package.xml']))
    

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='danielgrioja',
    maintainer_email='angel.galicia@ingenieria.unam.edu',
    description='Paquete ROS2 para el Rover y su brazo en Webots',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_controller = rover_ros2.rover_controller:main',
            'arm_controller = rover_ros2.arm_controller:main'
        ],
    },
)
