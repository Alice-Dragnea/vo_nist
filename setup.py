from setuptools import setup

package_name = 'vo_nist'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'launch/vo_launch.py', 'config/rs_vio.json']),
        ('lib/' + package_name, [package_name + '/voLib.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='px4vision',
    maintainer_email='cloeb@purdue.edu',
    description='Purdue NIST Visual Odometry Package',
    license='No License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom = vo_nist.run:main',
        ],
    },
)
