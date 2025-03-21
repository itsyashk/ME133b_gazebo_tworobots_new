from setuptools import setup
import os
from glob import glob

package_name = 'mars_robots'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        # Config files
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        # URDF files
        (os.path.join('share', package_name, 'urdf'),
         glob('urdf/*.urdf.xacro')),
        (os.path.join('share', package_name, 'urdf'),
         glob('urdf/*.gazebo.xacro')),
        # World files
        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*.world')),
        # RViz files
        (os.path.join('share', package_name, 'rviz'),
         glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A ROS 2 package for simulating two robots in a Mars-like environment',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)