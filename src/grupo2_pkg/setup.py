import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'grupo2_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='slawhs',
    maintainer_email='ignagaja@gmail.com',
    description='Rutinas del grupo 2 para la tarea del brazo AgileX Nero',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'waypoint_manager = grupo2_pkg.waypoint_manager:main',
            'interceptar_cubo = grupo2_pkg.interceptar_cubo:main',
            'recoger_cubo = grupo2_pkg.recoger_cubo:main',
            'waypoint_sender = grupo2_pkg.waypoint_sender:main',
            'pose_probe = grupo2_pkg.pose_probe:main',
            'proximity_sensor_node = grupo2_pkg.proximity_sensor_node:main',
        ],
    },
)
