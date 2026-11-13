#!/usr/bin/env python3
"""
parte1_demo.py — Tarea Nero, Parte 1
=====================================
Mueve el brazo Nero simulado en MuJoCo usando SOLO la arquitectura de ROS 2
(no toca MuJoCo directamente). Recorre una lista de waypoints y, para cada uno,
espera a que el brazo se detenga leyendo el feedback del simulador.

Dos modos (parametro `mode`):

  ik    (default)  Publica objetivos cartesianos [x, y, z, roll, pitch, yaw] en
                   `input_obj_position`. El flujo completo es:
                     parte1_demo -> main_node -> /obj_point -> ik_node
                     -> ik/ik_result -> can_module -> control/move_j
                     -> mujoco_bridge (simulador) -> feedback/joint_states ...
  joint            Publica directamente objetivos articulares en `control/move_j`
                   (incluye abrir/cerrar el gripper). Sirve para probar la
                   comunicacion con el simulador sin pasar por la IK.

Topicos:
  Publica   input_obj_position  std_msgs/Float64MultiArray   (modo ik)
            control/move_j      sensor_msgs/JointState       (modo joint)
            parte1/target       visualization_msgs/Marker    (esfera del objetivo, para RViz)
  Suscribe  feedback/joint_states  sensor_msgs/JointState    (para saber si el brazo se detuvo)
            feedback/tcp_pose      geometry_msgs/PoseStamped (pose de link7 -> posicion pinch_site)
            inverse_kinematics_ready std_msgs/Bool (latched) (modo ik: espera a que la IK este lista)

NOTA: la zona de trabajo del brazo esta en x negativo (-x); todos los waypoints
cartesianos estan en esa zona.

Uso (con el simulador + main_node + can_module + ik_node corriendo):
  ros2 run nero_ctrl parte1_demo
  ros2 run nero_ctrl parte1_demo --ros-args -p mode:=joint
  ros2 run nero_ctrl parte1_demo --ros-args -p loop:=false -p timeout:=15.0
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from visualization_msgs.msg import Marker

ARM_JOINTS = [f'joint{i}' for i in range(1, 8)]

# Orientaciones (roll, pitch, yaw) del sitio `pinch_site`, convencion R = Rz*Ry*Rx
# (la misma de ik_node.rpy_to_rotation_matrix).
HORIZONTAL = (-math.pi / 2, 0.0, math.pi)       # pinza apuntando hacia -x (pose de inicio)
VERTICAL = (0.0, math.pi / 2, -math.pi / 2)     # pinza apuntando hacia abajo (-z)

# Waypoints cartesianos [x, y, z] + rpy, en el marco del mundo (= base_link).
# Verificados offline con el mismo algoritmo de IK de ik_node (convergen < 1 mm).
CARTESIAN_WAYPOINTS = [
    ((-0.35, 0.00, 0.40), HORIZONTAL),
    ((-0.35, 0.15, 0.40), HORIZONTAL),
    ((-0.35, -0.15, 0.40), HORIZONTAL),
    ((-0.35, 0.00, 0.40), HORIZONTAL),
    ((-0.40, 0.00, 0.20), VERTICAL),
    ((-0.40, 0.12, 0.20), VERTICAL),
    ((-0.40, -0.12, 0.20), VERTICAL),
    ((-0.45, 0.00, 0.15), VERTICAL),
    ((-0.40, 0.00, 0.20), VERTICAL),
]

# Waypoints articulares: [joint1..joint7] + ancho del gripper (m, 0.0 cerrado - 0.1 abierto).
JOINT_WAYPOINTS = [
    [0.0, -0.40, 0.0, 2.00, 0.0, 0.0, 0.0, 0.05],    # pose inicial de can_module
    [0.6, -0.20, 0.0, 1.80, 0.0, 0.3, 0.0, 0.10],    # gira a un lado, abre pinza
    [-0.6, -0.20, 0.0, 1.80, 0.0, -0.3, 0.0, 0.00],  # gira al otro lado, cierra pinza
    [0.0, 0.45, 0.0, 1.29, 0.0, 0.0, 1.41, 0.10],    # baja con la pinza vertical
    [0.0, -0.40, 0.0, 2.00, 0.0, 0.0, 0.0, 0.05],    # vuelve al inicio
]

# Posicion de pinch_site en el marco de link7 (ver models_for_simulation/agilex_nero/nero.xml).
PINCH_OFFSET_LINK7 = np.array([0.1733, 0.0, -0.0235])


def quat_to_mat(w, x, y, z):
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Parte1Demo(Node):
    def __init__(self):
        super().__init__('parte1_demo')

        self.mode = self.declare_parameter('mode', 'ik').value
        self.loop = bool(self.declare_parameter('loop', True).value)
        self.startup_delay = float(self.declare_parameter('startup_delay', 4.0).value)
        self.min_wait = float(self.declare_parameter('min_wait', 1.5).value)
        self.timeout = float(self.declare_parameter('timeout', 12.0).value)
        self.settle_vel = float(self.declare_parameter('settle_vel', 0.02).value)
        self.settle_time = float(self.declare_parameter('settle_time', 0.8).value)
        if self.mode not in ('ik', 'joint'):
            raise ValueError("El parametro 'mode' debe ser 'ik' o 'joint'")

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # ---------------- publicadores
        self.target_pub = self.create_publisher(Float64MultiArray, 'input_obj_position', 10)
        self.move_j_pub = self.create_publisher(JointState, 'control/move_j', 10)
        self.marker_pub = self.create_publisher(Marker, 'parte1/target', latched)

        # ---------------- suscriptores
        self.create_subscription(JointState, 'feedback/joint_states', self._on_joint_states, 10)
        self.create_subscription(PoseStamped, 'feedback/tcp_pose', self._on_tcp_pose, 10)
        self.create_subscription(Bool, 'inverse_kinematics_ready', self._on_ik_ready, latched)

        # ---------------- estado
        self.ik_ready = False
        self.q = None
        self.qd = None
        self.pinch_pos = None
        self.first_feedback_t = None
        self.idx = -1
        self.sent_t = None
        self.still_since = None
        self.finished = False

        self.waypoints = CARTESIAN_WAYPOINTS if self.mode == 'ik' else JOINT_WAYPOINTS
        self.create_timer(0.1, self._tick)  # maquina de estados a 10 Hz
        self.get_logger().info(
            f'parte1_demo en modo "{self.mode}" con {len(self.waypoints)} waypoints '
            f'(loop={self.loop}). Esperando feedback del simulador...')

    # ------------------------------------------------------------- callbacks
    def _on_ik_ready(self, msg: Bool):
        if msg.data and not self.ik_ready:
            self.ik_ready = True
            self.get_logger().info('ik_node listo.')

    def _on_joint_states(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        vel = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
        if not all(n in pos for n in ARM_JOINTS):
            return
        self.q = np.array([pos[n] for n in ARM_JOINTS])
        self.qd = np.array([vel.get(n, 0.0) for n in ARM_JOINTS])
        if self.first_feedback_t is None:
            self.first_feedback_t = self._now()
            self.get_logger().info('Recibiendo feedback/joint_states del simulador.')

    def _on_tcp_pose(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        R = quat_to_mat(o.w, o.x, o.y, o.z)
        self.pinch_pos = np.array([p.x, p.y, p.z]) + R @ PINCH_OFFSET_LINK7

    # ------------------------------------------------------------- logica
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _ready(self):
        if self.first_feedback_t is None:
            return False
        if self._now() - self.first_feedback_t < self.startup_delay:
            return False  # deja que can_module lleve el brazo a la pose inicial
        return self.mode == 'joint' or self.ik_ready

    def _tick(self):
        if self.finished or not self._ready():
            return
        now = self._now()

        if self.idx < 0:
            self._send_next()
            return

        elapsed = now - self.sent_t
        moving = self.qd is None or float(np.max(np.abs(self.qd))) > self.settle_vel
        if moving:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = now

        settled = (elapsed > self.min_wait and self.still_since is not None
                   and now - self.still_since > self.settle_time)
        if settled or elapsed > self.timeout:
            self._report(settled)
            self._send_next()

    def _report(self, settled):
        tag = 'alcanzado' if settled else 'TIMEOUT'
        if self.mode == 'ik' and self.pinch_pos is not None:
            target = np.array(self.waypoints[self.idx][0])
            err = float(np.linalg.norm(target - self.pinch_pos)) * 1000.0
            self.get_logger().info(
                f'[{self.idx + 1}/{len(self.waypoints)}] {tag}: pinch_site = '
                f'{np.round(self.pinch_pos, 3).tolist()}  (error {err:.1f} mm)')
        elif self.q is not None:
            self.get_logger().info(
                f'[{self.idx + 1}/{len(self.waypoints)}] {tag}: q = {np.round(self.q, 3).tolist()}')

    def _send_next(self):
        self.idx += 1
        if self.idx >= len(self.waypoints):
            if not self.loop:
                self.finished = True
                self.get_logger().info('Recorrido terminado.')
                return
            self.idx = 0
            self.get_logger().info('Reiniciando el recorrido.')

        wp = self.waypoints[self.idx]
        if self.mode == 'ik':
            (x, y, z), (r, p, yw) = wp
            msg = Float64MultiArray()
            msg.data = [float(x), float(y), float(z), float(r), float(p), float(yw)]
            self.target_pub.publish(msg)
            self._publish_marker(x, y, z)
            self.get_logger().info(
                f'[{self.idx + 1}/{len(self.waypoints)}] objetivo cartesiano -> '
                f'input_obj_position: xyz=({x:.2f}, {y:.2f}, {z:.2f}) '
                f'rpy=({r:.2f}, {p:.2f}, {yw:.2f})')
        else:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ARM_JOINTS + ['gripper']
            msg.position = [float(v) for v in wp]
            msg.effort = [0.0] * 7 + [1.0]
            self.move_j_pub.publish(msg)
            self.get_logger().info(
                f'[{self.idx + 1}/{len(self.waypoints)}] objetivo articular -> '
                f'control/move_j: {wp}')

        self.sent_t = self._now()
        self.still_since = None

    def _publish_marker(self, x, y, z):
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'parte1'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.03
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.2, 0.2, 0.9
        self.marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = Parte1Demo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
