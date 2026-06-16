from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'capytown_esan'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Grupo CapyTown',
    maintainer_email='grupo@esan.edu.pe',
    description='Paquete ROS2 del proyecto CapyTown - RC-1 La Manzana del Tambo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_detector = capytown_esan.lane_detector:main',
            'lane_controller = capytown_esan.lane_controller:main',
        ],
    },
)
