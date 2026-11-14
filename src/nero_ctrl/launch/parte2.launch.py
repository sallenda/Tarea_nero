"""
nero_ctrl — parte2.launch.py   (Tarea Nero, Parte 2: cinta + cubo + intercepcion)
===================================================================================
Levanta con un solo comando:

  1. Simulador MuJoCo con la cinta del grupo (cinta.xml) y un cubo de 4 cm que se
     mueve en linea recta a velocidad constante (mujoco_cinta_bridge.py), + RViz.
  2. main_node + can_module           (nero_can_ctrl.launch.py mode:=sim)
  3. ik_node                          (IK del curso, para movimientos grandes)
  4. interceptor_node                 (estima la trayectoria del cubo y lo intercepta)

Ejemplos:
  ros2 launch nero_ctrl parte2.launch.py
  ros2 launch nero_ctrl parte2.launch.py cube_speed:=0.06
  ros2 launch nero_ctrl parte2.launch.py cube_dir:=1            # el cubo va de -y a +y
  ros2 launch nero_ctrl parte2.launch.py interceptor:=false     # solo la escena (cubo moviendose)
  ros2 launch nero_ctrl parte2.launch.py tracking:=ik_node      # seguir con ik_node (comparacion)

En otra terminal, durante la simulacion:
  ros2 topic pub --once /conveyor/speed std_msgs/msg/Float64 "{data: 0.05}"
  ros2 topic pub --once /cube/reset std_msgs/msg/Empty "{}"
  ros2 topic echo /parte2/estado
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    script = os.path.join(get_package_share_directory('agx_arm_description'),
                          'scripts', 'mujoco_cinta_bridge.py')
    viewer_args = ' '.join([
        '--cinta-pos', arg('cinta_x'), arg('cinta_y'), '0.0',
        '--cinta-yaw', arg('cinta_yaw'),
        '--cube-speed', arg('cube_speed'),
        '--cube-dir', arg('cube_dir'),
        '--cube-start-delay', arg('cube_start_delay'),
    ])
    start_delay = float(arg('start_delay'))

    sim_and_core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nero_ctrl'), 'launch', 'nero_can_ctrl.launch.py')
        ),
        launch_arguments={
            'mode': 'sim',
            'scene': 'scene_nero_arm.xml',
            'rviz': arg('rviz'),
            'start_delay': str(start_delay),
            'viewer_script': script,
            'viewer_args': viewer_args,
        }.items(),
    )

    ik_node = Node(package='nero_ctrl', executable='ik_node',
                   name='inverse_kinematics_node', output='screen')

    interceptor = Node(
        package='nero_ctrl', executable='interceptor_node', name='interceptor_node',
        output='screen',
        parameters=[{
            'tracking': arg('tracking'),
            'standby_xyz': [float(arg('cinta_x')), float(arg('cinta_y')), 0.17],
        }],
        condition=IfCondition(arg('interceptor')),
    )

    return [
        sim_and_core,
        TimerAction(period=start_delay, actions=[ik_node]),
        TimerAction(period=start_delay + 1.0, actions=[interceptor]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_delay', default_value='2.0',
                              description='Espera (s) antes de lanzar los nodos de control'),
        DeclareLaunchArgument('cinta_x', default_value='-0.45',
                              description='Posicion x de la cinta (zona de trabajo en -x)'),
        DeclareLaunchArgument('cinta_y', default_value='0.0'),
        DeclareLaunchArgument('cinta_yaw', default_value='1.5708',
                              description='Giro de la cinta (rad); 1.5708 = a lo largo del eje y'),
        DeclareLaunchArgument('cube_speed', default_value='0.04', description='Velocidad del cubo (m/s)'),
        DeclareLaunchArgument('cube_dir', default_value='-1', choices=['-1', '1'],
                              description='Sentido del cubo sobre la banda'),
        DeclareLaunchArgument('cube_start_delay', default_value='8.0',
                              description='Segundos que el cubo espera antes de partir'),
        DeclareLaunchArgument('interceptor', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('tracking', default_value='interno', choices=['interno', 'ik_node'],
                              description='IK para seguir al cubo: interno (tracking_ik) o ik_node'),
        OpaqueFunction(function=launch_setup),
    ])
