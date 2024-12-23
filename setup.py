from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bad_apple'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'frames-ascii'), glob('frames-ascii/*.txt')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kadir Yavuz Kurt',
    maintainer_email='k.yavuzkurt1@gmail.com',
    description='Bad Apple ASCII animation as a ROS2 topic.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ascii_publisher = bad_apple.ascii_publisher:main',
            'ascii_subscriber = bad_apple.ascii_subscriber:main',
        ],
    },
)
