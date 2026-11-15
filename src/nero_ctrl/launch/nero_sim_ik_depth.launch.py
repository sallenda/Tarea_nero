"""
nero_ctrl — nero_sim_ik_depth.launch.py
========================================
Lanza la simulacion IK del brazo Nero junto con el sensor ultrasonico conectado
por Arduino.

Ejemplo:
  ros2 launch nero_ctrl nero_sim_ik_depth.launch.py demo:=true
  ros2 launch nero_ctrl nero_sim_ik_depth.launch.py demo:=true serial_port:=/dev/ttyUSB0

Topicos del sensor:
  ultrasonic/distance_cm   Entrada publicada por serial_connection.
  depth_sensor/distance_cm  Salida republicada por depth_sensor.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    scene = LaunchConfiguration('scene')
    rviz = LaunchConfiguration('rviz')
    start_delay = LaunchConfiguration('start_delay')
    demo = LaunchConfiguration('demo')
    demo_mode = LaunchConfiguration('demo_mode')
    serial_port = LaunchConfiguration('serial_port')
    baudrate = LaunchConfiguration('baudrate')

    arm_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nero_ctrl'),
                'launch',
                'nero_sim_ik.launch.py',
            )
        ),
        launch_arguments={
            'scene': scene,
            'rviz': rviz,
            'start_delay': start_delay,
            'demo': demo,
            'demo_mode': demo_mode,
        }.items(),
    )

    serial_node = Node(
        package='arduino_nodes',
        executable='serial_connection',
        name='arduino_serial_node',
        output='screen',
        parameters=[{
            'port': serial_port,
            'baudrate': baudrate,
        }],
    )

    depth_sensor_node = Node(
        package='nero_ctrl',
        executable='depth_sensor',
        name='depth_sensor_node',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'scene',
            default_value='scene_nero_arm.xml',
            description='Escena MuJoCo del simulador',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'start_delay',
            default_value='2.0',
            description='Espera antes de lanzar los nodos de control',
        ),
        DeclareLaunchArgument(
            'demo',
            default_value='false',
            choices=['true', 'false'],
            description='Ejecutar el recorrido de waypoints',
        ),
        DeclareLaunchArgument(
            'demo_mode',
            default_value='ik',
            choices=['ik', 'joint'],
            description='Modo de la demo del brazo',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Puerto serial del Arduino',
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='Baudios del Arduino',
        ),
        arm_simulation,
        serial_node,
        depth_sensor_node,
    ])
