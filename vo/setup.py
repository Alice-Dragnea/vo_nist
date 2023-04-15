from setuptools import setup
import os
import glob

package_name = 'vo'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'launch/vo_launch.py', 'config/rs_vio.json']),
        ('lib/' + package_name, [package_name+'/voLib.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cloeb',
    maintainer_email='cloeb@purdue.edu',
    description='Visual Odometry Package - Purdue Spring 2023',
    license='No License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom = vo.run:main',
        ],
    },
)
