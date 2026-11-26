"""P3: intercepcion, agarre y levantamiento del cubo.

simulacion_cubo.launch.py (cinta automatica) + la rutina recoger_cubo, que estima
la velocidad del cubo con /proximity, lo agarra entre los dedos y lo levanta
(brazo y pinza por /commands/waypoints).

Ejemplos:
  ros2 launch grupo2_pkg recoger_cubo.launch.py
  ros2 launch grupo2_pkg recoger_cubo.launch.py cube_speed:=0.12 rviz:=true
  ros2 launch grupo2_pkg recoger_cubo.launch.py hil:=true     # cubo real + sensor por USB
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('grupo2_pkg'), 'launch', 'simulacion_cubo.launch.py')),
        launch_arguments={
            'routine': 'recoger_cubo',
            'cube_speed': LaunchConfiguration('cube_speed'),
            'cube_delay': LaunchConfiguration('cube_delay'),
            'rviz': LaunchConfiguration('rviz'),
            'hil': LaunchConfiguration('hil'),
            'sensor_port': LaunchConfiguration('sensor_port'),
            'sensor_y': LaunchConfiguration('sensor_y'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('cube_speed', default_value='0.08',
                              description='Velocidad de la cinta en +Y (m/s)'),
        DeclareLaunchArgument('cube_delay', default_value='8.0',
                              description='Segundos antes de arrancar la cinta (brazo ya en posicion)'),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false'],
                              description='Lanzar RViz2 junto a MuJoCo'),
        DeclareLaunchArgument('hil', default_value='false', choices=['true', 'false'],
                              description='Hardware in the loop: el cubo sigue al sensor real por USB'),
        DeclareLaunchArgument('sensor_port', default_value='/dev/ttyACM0',
                              description='Puerto serie del Arduino del sensor (solo hil:=true)'),
        DeclareLaunchArgument('sensor_y', default_value='-0.90',
                              description='Posicion en la via (y mundo) del sensor real (solo hil:=true)'),
        base,
    ])
