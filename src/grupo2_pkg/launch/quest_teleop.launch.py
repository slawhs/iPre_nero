"""Simulación lista para teleoperar con el Meta Quest.

Levanta MuJoCo (nero_can_ctrl mode:=sim), ik_node y quest_teleop. Los nodos del
Quest se lanzan aparte, desde el workspace que tenga oculus_reader:

  ros2 run oculus_reader pub_pose.py --ros-args -p ros_to_arm_rpy:="[-1.5708, 0.0, 0.0]"
  ros2 run oculus_reader pub_delta_pose.py --ros-args \\
      -p feedback_tcp_pose_topic:=/feedback/pinch_pose -p gripper_max_range:=0.05

En el control derecho: mantener A para teleoperar, B para detener, gatillo para el gripper.
No usar a la vez que waypoint_manager.

Ejemplos:
  ros2 launch grupo2_pkg quest_teleop.launch.py
  ros2 launch grupo2_pkg quest_teleop.launch.py rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    simulacion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nero_ctrl'), 'launch', 'nero_can_ctrl.launch.py')),
        launch_arguments={
            'mode': 'sim',
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )

    # Reenvío de /start_running_nodes: can_module puede perderse el de main_node (ver
    # simulacion_cubo.launch.py). Se esperan 3 suscriptores porque ik_node tiene dos.
    orden_arranque = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '-w', '3', '/start_running_nodes',
             'std_msgs/msg/Bool', '{data: true}'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false'],
                              description='Lanzar RViz2 junto a MuJoCo'),
        simulacion,
        Node(package='nero_ctrl', executable='ik_node', output='screen'),
        Node(package='grupo2_pkg', executable='quest_teleop', output='screen'),
        orden_arranque,
    ])
