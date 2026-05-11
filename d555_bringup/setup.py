from setuptools import setup
import os
from glob import glob

package_name = 'd555_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Intel RealSense',
    maintainer_email='realsense@intel.com',
    description='Host-side bringup launch file for Intel RealSense D555 camera',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'd555_color_relay = d555_bringup.d555_color_relay:main',
        ],
    },
)
