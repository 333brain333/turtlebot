import os
from glob import glob

from setuptools import setup

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agrodroid',
    maintainer_email='222brain222@gmail.com',
    description='Launch files for the robot system.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
)
