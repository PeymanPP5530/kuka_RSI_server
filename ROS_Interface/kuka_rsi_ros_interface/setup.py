import os 
from glob import glob
from setuptools import find_packages, setup

package_name = 'kuka_rsi_ros_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,"launch"), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vahid',
    maintainer_email='vahid@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'kuka_rsi_ros = kuka_rsi_ros_interface.kuka_rsi_ros:main',
        ],
    },
)
