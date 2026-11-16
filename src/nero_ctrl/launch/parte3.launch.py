"""
nero_ctrl — parte3.launch.py   (Tarea Nero, Parte 3: interceptar, cerrar la pinza y recoger el cubo)
=====================================================================================================
Es parte2.launch.py con grasp:=true. Acepta los mismos argumentos.

  ros2 launch nero_ctrl parte3.launch.py
  ros2 launch nero_ctrl parte3.launch.py cube_speed:=0.05
  ros2 topic echo /parte3/agarrado
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

PASSTHROUGH = {
    'rviz': 'true',
    'start_delay': '2.0',
    'cinta_x': '-0.45',
    'cinta_y': '0.0',
    'cinta_yaw': '1.5708',
    'cube_speed': '0.04',
    'cube_dir': '-1',
    'cube_start_delay': '8.0',
    'tracking': 'interno',
}


def generate_launch_description():
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in PASSTHROUGH.items()]
    launch_arguments = {k: LaunchConfiguration(k) for k in PASSTHROUGH}
    launch_arguments['grasp'] = 'true'
    launch_arguments['interceptor'] = 'true'
    parte2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nero_ctrl'), 'launch', 'parte2.launch.py')
        ),
        launch_arguments=launch_arguments.items(),
    )
    return LaunchDescription(args + [parte2])
