from setuptools import setup
import os

package_name = 'cmdvel_to_servo_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), ['launch/cmdvel_to_servo_pkg_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AWS DeepRacer',
    maintainer_email='aws-deepracer@amazon.com',
    description='This package converts the cmd_vel received to DeepRacer supported servo msgs.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmdvel_to_servo_node = cmdvel_to_servo_pkg.cmdvel_to_servo_node:main'
        ],
    },
)
