from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'april_tags'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Safina Baiocchi & Prisha Kansal',
    maintainer_email='safina.baiocchi@example.com',
    description='AprilTag subscriber node for ROS2, subscribes to Brio image topic.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'atag_subscriber = atag_subscriber:main',
        ],
    },
)