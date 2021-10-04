from setuptools import setup
import os

package_name = 'enable_deepracer_nav_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), ['launch/enable_deepracer_nav_pkg_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AWS DeepRacer',
    maintainer_email='aws-deepracer@amazon.com',
    description='This package enables the services for nodes required for enabling ROS Navigation on DeepRacer device.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'enable_deepracer_nav_node = enable_deepracer_nav_pkg.enable_deepracer_nav_node:main'
        ],
    },
)
