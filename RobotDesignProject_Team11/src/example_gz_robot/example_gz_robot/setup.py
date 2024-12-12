import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'example_gz_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include rviz (.rviz) files
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        # Include world (.sdf, .dae, and .stl) files
        (os.path.join('share', package_name, 'worlds'), 
        glob(os.path.join('worlds', '**', '*.sdf'), recursive=True) +
        glob(os.path.join('worlds', '**', '*.dae'), recursive=True) +
        glob(os.path.join('worlds', '**', '*.stl'), recursive=True)),
        # Include config (.yaml) files
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*yaml*'))),
	# Include map (.yaml and .pgm) files
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.[yp][ag][m]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DrAndyWest',
    maintainer_email='andrew.west@manchester.ac.uk',
    description='Package to investigate the Nav2 stack in ROS',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
