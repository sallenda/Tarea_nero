"""
nero_ctrl — nero_sim_ik.launch.py   (Tarea Nero, Parte 1)
==========================================================
Levanta TODA la arquitectura ROS 2 en simulacion con un solo comando:

  1. Simulador MuJoCo + puente ROS 2 (mujoco_bridge) + robot_state_publisher + RViz
     (via nero_can_ctrl.launch.py mode:=sim -> bringup_nero.launch.py)
  2. main_node   (coordinador: /model_path, start_running_nodes, relay de objetivos)
  3. can_module  (pose inicial, puente ik/ik_result -> control/move_j)
  4. ik_node     (cinematica inversa por Jacobiano; publica ik/ik_result)
  5. parte1_demo (opcional, demo:=true) recorre waypoints para mostrar el brazo moviendose

Ejemplos:
  ros2 launch nero_ctrl nero_sim_ik.launch.py                       # todo, sin demo
  ros2 launch nero_ctrl nero_sim_ik.launch.py demo:=true            # + recorrido por IK
  ros2 launch nero_ctrl nero_sim_ik.launch.py demo:=true demo_mode:=joint
  ros2 launch nero_ctrl nero_sim_ik.launch.py rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    scene = LaunchConfiguration('scene')
    rviz = LaunchConfiguration('rviz')
    start_delay = LaunchConfiguration('start_delay')
    demo = LaunchConfiguration('demo')
    demo_mode = LaunchConfiguration('demo_mode')

    # Simulador + main_node + can_module (mismo launch que usa el README del repo base).
    sim_and_core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nero_ctrl'), 'launch',
                         'nero_can_ctrl.launch.py')
        ),
        launch_arguments={
            'mode': 'sim',
            'scene': scene,
            'rviz': rviz,
            'start_delay': start_delay,
        }.items(),
    )

    ik_node = Node(
        package='nero_ctrl',
        executable='ik_node',
        name='inverse_kinematics_node',
        output='screen',
    )

    demo_node = Node(
        package='nero_ctrl',
        executable='parte1_demo',
        name='parte1_demo',
        output='screen',
        parameters=[{'mode': demo_mode}],
        condition=IfCondition(demo),
    )

    return LaunchDescription([
        DeclareLaunchArgument('scene', default_value='scene_nero_arm.xml',
                              description='Escena MuJoCo del simulador'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_delay', default_value='2.0',
                              description='Espera (s) antes de lanzar los nodos de control'),
        DeclareLaunchArgument('demo', default_value='false', choices=['true', 'false'],
                              description='Lanzar parte1_demo (recorrido de waypoints)'),
        DeclareLaunchArgument('demo_mode', default_value='ik', choices=['ik', 'joint'],
                              description='ik: objetivos cartesianos via ik_node | joint: move_j directo'),
        sim_and_core,
        # ik_node se lanza junto con main_node/can_module (cuando el simulador ya escucha).
        TimerAction(period=PythonExpression([start_delay]), actions=[ik_node]),
        # La demo parte un poco despues; ademas ella misma espera feedback e ik listo.
        TimerAction(period=PythonExpression([start_delay, ' + 1.0']), actions=[demo_node]),
    ])
