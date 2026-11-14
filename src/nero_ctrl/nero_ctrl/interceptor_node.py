#!/usr/bin/env python3
"""
interceptor_node.py — Tarea Nero, Parte 2: interceptar el cubo sobre la cinta
==============================================================================
Lee la pose del cubo que se mueve en linea recta a velocidad constante, estima su
velocidad, predice su trayectoria y lleva la pinza a encontrarse con el cubo en un
punto de esa trayectoria, usando la arquitectura ROS 2 del repo:

  * Movimientos grandes (ir a la pose de espera sobre la cinta) con la IK del curso:
      interceptor_node -> input_obj_position -> main_node -> /obj_point -> ik_node
                       -> ik/ik_result -> can_module -> control/move_j -> simulador
  * Seguimiento del cubo (objetivo en movimiento) con una IK rapida propia
    (tracking_ik.py, mismo modelo /model_path y mismo pinch_site), publicada a 50 Hz en
    el MISMO topico que usa ik_node:
      interceptor_node -> ik/ik_result -> can_module -> control/move_j -> simulador
    ik_node no sirve para seguir objetivos que se mueven: parte cada calculo desde el
    estado medido del brazo (que va atrasado) y el brazo quedaba a mas de 7 cm del cubo.
    (Con el parametro tracking:=ik_node se puede probar ese modo para comparar.)

Por que se "acompana" al cubo en vez de esperarlo quieto: con la pinza vertical
(rpy = 0, pi/2, -pi/2) los dedos abren a lo largo del eje y, que es justo la direccion
de la cinta. Si la pinza esperara quieta a la altura del cubo, el cubo chocaria con el
dedo de entrada. Por eso la pinza se alinea ENCIMA del cubo, baja mientras lo sigue
(misma velocidad) y queda con un dedo delante y otro detras del cubo: eso es la
intercepcion (posicion y velocidad iguales a las del cubo). La orientacion girada en
90 grados (dedos a lo largo de x) casi no tiene solucion de IK por los limites de
joint6/joint7, se verifico offline.

Maquina de estados:
  INICIO       espera ik_node listo, feedback y cube/pose; abre la pinza
  ESPERA       pinza sobre la cinta (pose de espera); espera que el cubo entre a la ventana
  ALINEANDO    sigue al cubo desde arriba (z_above), apuntando a su posicion predicha
  BAJANDO      baja en t_descend segundos sin dejar de seguir al cubo
  INTERCEPTADO pinza alrededor del cubo; lo acompana t_hold segundos (Parte 3: cerrar aqui)
  SUBIENDO     vuelve a z_above y despues a ESPERA; espera la siguiente vuelta del cubo

Topicos:
  Suscribe  cube/pose               geometry_msgs/PoseStamped (posicion del cubo, base_link)
            feedback/joint_states   sensor_msgs/JointState    (brazo quieto / en movimiento)
            feedback/tcp_pose       geometry_msgs/PoseStamped (pose link7 -> pinch_site)
            inverse_kinematics_ready std_msgs/Bool (latched)
            /model_path             std_msgs/String (latched)  (modelo para la IK de seguimiento)
  Publica   input_obj_position      std_msgs/Float64MultiArray [x, y, z, roll, pitch, yaw]
            ik/ik_result            sensor_msgs/JointState     (seguimiento a 50 Hz, joint1..7 + gripper)
            control/move_j          sensor_msgs/JointState     (solo 'gripper': abrir)
            parte2/estado           std_msgs/String            (estado de la maquina)
            parte2/interceptado     std_msgs/Bool              (True al interceptar)
            parte2/markers          visualization_msgs/MarkerArray (prediccion + punto de intercepcion)

Uso:
  ros2 run nero_ctrl interceptor_node
  ros2 run nero_ctrl interceptor_node --ros-args -p lead:=0.5 -p window_y_min:=-0.2
"""

import collections
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray

from nero_ctrl.parte1_demo import PINCH_OFFSET_LINK7, VERTICAL, quat_to_mat
from nero_ctrl.tracking_ik import TrackingIK, rpy_to_matrix

ARM_JOINTS = [f'joint{i}' for i in range(1, 8)]


class CubeEstimator:
    """Ajuste lineal p(t) = p0 + v (t - t_ref) sobre una ventana de tiempo reciente."""

    def __init__(self, window=1.0, jump=0.03):
        self.window = window
        self.jump = jump
        self.samples = collections.deque()
        self.resets = 0

    def add(self, t, p):
        if self.samples and np.linalg.norm(p - self.samples[-1][1]) > self.jump:
            self.samples.clear()          # el cubo "salto" (reinicio de la cinta)
            self.resets += 1
        self.samples.append((t, p))
        while self.samples and t - self.samples[0][0] > self.window:
            self.samples.popleft()

    def last(self):
        return self.samples[-1] if self.samples else (None, None)

    def fit(self):
        """Devuelve (t_ref, p_ref, v) o None si no hay datos suficientes."""
        if len(self.samples) < 10:
            return None
        t = np.array([s[0] for s in self.samples])
        P = np.array([s[1] for s in self.samples])
        if t[-1] - t[0] < 0.3:
            return None
        t_ref = t[-1]
        A = np.vstack([np.ones_like(t), t - t_ref]).T
        coef, *_ = np.linalg.lstsq(A, P, rcond=None)
        return t_ref, coef[0], coef[1]


class InterceptorNode(Node):
    def __init__(self):
        super().__init__('interceptor_node')
        dp = self.declare_parameter
        self.rpy = list(dp('rpy', list(VERTICAL)).value)
        self.z_above = float(dp('z_above', 0.17).value)         # altura del pinch_site al alinear
        self.grasp_dz = float(dp('grasp_dz', -0.004).value)     # pinch_site respecto al centro del cubo
        self.tracking = str(dp('tracking', 'interno').value)    # 'interno' (tracking_ik) o 'ik_node'
        self.lead = float(dp('lead', 0.15).value)               # s: adelanto para compensar el retardo
        self.rate = float(dp('rate', 50.0).value)               # Hz: objetivos de seguimiento
        self.t_descend = float(dp('t_descend', 2.0).value)
        self.t_hold = float(dp('t_hold', 2.0).value)
        self.t_align_max = float(dp('t_align_max', 5.0).value)
        self.align_tol = float(dp('align_tol', 0.012).value)
        self.intercept_tol = float(dp('intercept_tol', 0.008).value)
        self.window_y_min = float(dp('window_y_min', -0.22).value)
        self.window_y_max = float(dp('window_y_max', 0.22).value)
        self.min_speed = float(dp('min_speed', 0.005).value)
        # Correccion integral en x-y (usa feedback/tcp_pose): compensa el error
        # estacionario de los actuadores de la muneca (kp=10 y frictionloss=1 en el MJCF).
        self.ki = float(dp('ki', 1.5).value)
        self.corr_max = float(dp('corr_max', 0.025).value)
        self.gripper_open = float(dp('gripper_open', 0.1).value)
        self.standby = list(dp('standby_xyz', [-0.45, 0.0, 0.17]).value)
        self.startup_delay = float(dp('startup_delay', 3.0).value)
        self.loop = bool(dp('loop', True).value)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.target_pub = self.create_publisher(Float64MultiArray, 'input_obj_position', 10)
        self.move_j_pub = self.create_publisher(JointState, 'control/move_j', 10)
        self.state_pub = self.create_publisher(String, 'parte2/estado', latched)
        self.hit_pub = self.create_publisher(Bool, 'parte2/interceptado', latched)
        self.marker_pub = self.create_publisher(MarkerArray, 'parte2/markers', 10)

        self.create_subscription(PoseStamped, 'cube/pose', self._on_cube, 10)
        self.create_subscription(JointState, 'feedback/joint_states', self._on_js, 10)
        self.create_subscription(PoseStamped, 'feedback/tcp_pose', self._on_tcp, 10)
        self.create_subscription(Bool, 'inverse_kinematics_ready', self._on_ik_ready, latched)
        self.create_subscription(String, '/model_path', self._on_model_path, latched)
        self.ik_result_pub = self.create_publisher(JointState, 'ik/ik_result', 10)
        if self.tracking == 'ik_node':
            # ik_node tarda ~0.3 s por objetivo: menos objetivos por segundo y mas adelanto.
            self.rate = min(self.rate, 5.0)
            self.lead = max(self.lead, 0.4)
        self.R_target = rpy_to_matrix(self.rpy)
        self.tik = None
        self.q = None
        self.corr = np.zeros(3)
        self.t_corr = None

        self.est = CubeEstimator()
        self.ik_ready = False
        self.qd = None
        self.pinch = None
        self.first_fb = None
        self.state = None
        self.t_state = 0.0
        self.t_last_cmd = -1e9
        self.aligned_since = None
        self.lap_resets = None         # contador de reinicios del cubo al terminar una vuelta
        self.z_from = self.z_above
        self.hit_point = None
        self.hit_errors = []
        self.n_intercepts = 0

        self._set_state('INICIO')
        self.create_timer(0.05, self._tick)   # 20 Hz
        self.get_logger().info(
            f'interceptor_node: ventana y=[{self.window_y_min}, {self.window_y_max}], '
            f'z_above={self.z_above}, lead={self.lead}s, rpy={np.round(self.rpy, 3).tolist()}')

    # ------------------------------------------------------------------ callbacks
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_ik_ready(self, msg):
        self.ik_ready = self.ik_ready or bool(msg.data)

    def _on_model_path(self, msg):
        if msg.data and self.tik is None:
            try:
                self.tik = TrackingIK(msg.data)
                self.get_logger().info(f'IK de seguimiento con el modelo {msg.data}')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'No se pudo cargar el modelo para la IK de seguimiento: {exc}')

    def _on_js(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(n in pos for n in ARM_JOINTS):
            self.q = np.array([pos[n] for n in ARM_JOINTS])
        vel = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
        self.qd = np.array([vel.get(n, 0.0) for n in ARM_JOINTS])
        if self.first_fb is None:
            self.first_fb = self._now()

    def _on_tcp(self, msg):
        p, o = msg.pose.position, msg.pose.orientation
        R = quat_to_mat(o.w, o.x, o.y, o.z)
        self.pinch = np.array([p.x, p.y, p.z]) + R @ PINCH_OFFSET_LINK7

    def _on_cube(self, msg):
        p = msg.pose.position
        self.est.add(self._now(), np.array([p.x, p.y, p.z]))

    # ------------------------------------------------------------------ helpers
    def _set_state(self, s):
        if s != self.state:
            self.get_logger().info(f'Estado: {self.state} -> {s}')
            self.state = s
            self.t_state = self._now()
            self.aligned_since = None
            self.state_pub.publish(String(data=s))

    def _predict(self, fit, t):
        t_ref, p_ref, v = fit
        return p_ref + v * (t - t_ref)

    def _in_window(self, p):
        return self.window_y_min <= p[1] <= self.window_y_max

    def _send(self, xyz, force=False):
        """Objetivo grande (pose de espera): a traves de main_node + ik_node."""
        now = self._now()
        if not force and now - self.t_last_cmd < 1.0 / self.rate:
            return
        self.t_last_cmd = now
        msg = Float64MultiArray()
        msg.data = [float(xyz[0]), float(xyz[1]), float(xyz[2])] + [float(a) for a in self.rpy]
        self.target_pub.publish(msg)

    def _track(self, xyz):
        """Objetivo de seguimiento (cubo en movimiento)."""
        if self.tracking == 'ik_node':
            self._send(xyz)
            return
        now = self._now()
        if now - self.t_last_cmd < 1.0 / self.rate:
            return
        self.t_last_cmd = now
        xyz = np.asarray(xyz, dtype=float)
        if self.ki > 0.0 and self.pinch is not None and self.t_corr is not None:
            dt = min(now - self.t_corr, 0.1)
            e = self._last_goal - self.pinch
            e[2] = 0.0   # solo x-y: en z el error es transitorio (al bajar) y se acumularia
            self.corr = np.clip(self.corr + self.ki * dt * e, -self.corr_max, self.corr_max)
        self.t_corr = now
        self._last_goal = xyz.copy()
        xyz = xyz + self.corr
        if self.tik.q is None:
            self.tik.reset(self.q)
        q, _err = self.tik.solve(np.asarray(xyz, dtype=float), self.R_target)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ARM_JOINTS + ['gripper']
        js.position = [float(v) for v in q] + [self.gripper_open]
        js.effort = [0.0] * 7 + [1.0]
        self.ik_result_pub.publish(js)

    def _start_tracking(self):
        if self.tik is not None:
            self.tik.reset(self.q)   # semilla = estado actual del brazo
        self.corr = np.zeros(3)
        self.t_corr = None

    def _open_gripper(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['gripper']
        js.position = [self.gripper_open]
        js.effort = [1.0]
        self.move_j_pub.publish(js)

    def _arm_still(self):
        return self.qd is not None and float(np.max(np.abs(self.qd))) < 0.02

    # ------------------------------------------------------------------ logica
    def _tick(self):
        now = self._now()
        el = now - self.t_state
        fit = self.est.fit()
        moving = fit is not None and np.linalg.norm(fit[2][:2]) > self.min_speed

        if self.state == 'INICIO':
            ready = (self.ik_ready and self.first_fb is not None and self.pinch is not None
                     and self.q is not None and now - self.first_fb > self.startup_delay
                     and (self.tik is not None or self.tracking == 'ik_node'))
            if ready:
                self.get_logger().info('Abriendo la pinza y yendo a la pose de espera sobre la cinta.')
                self._open_gripper()
                self._set_state('ABRIENDO')
            return

        if self.state == 'ABRIENDO':
            if el > 1.0:   # deja que el feedback ya traiga la pinza abierta antes de pedir IK
                self._send(self.standby, force=True)
                self._set_state('ESPERA')
            return

        if self.state == 'ESPERA':
            if el < 2.0 or not self._arm_still() or not moving:
                return
            if self.lap_resets is not None and self.est.resets == self.lap_resets:
                return     # ya se intercepto en esta vuelta: esperar a que el cubo reinicie
            v = fit[2]
            p_lead = self._predict(fit, now + self.lead)
            T_total = 2.0 + self.t_descend + self.t_hold
            p_end = self._predict(fit, now + self.lead + T_total)
            if self._in_window(p_lead) and self._in_window(p_end):
                self.get_logger().info(
                    f'Cubo detectado: v = {np.round(v, 4).tolist()} m/s '
                    f'(|v| = {np.linalg.norm(v):.3f} m/s). Alineando encima del cubo.')
                self._start_tracking()
                self._set_state('ALINEANDO')
            return

        if fit is None:
            # Se perdio la estimacion (reinicio del cubo a mitad de maniobra): abortar.
            if self.state in ('ALINEANDO', 'BAJANDO', 'INTERCEPTADO'):
                self.get_logger().warn('Se perdio el cubo: abortando y subiendo.')
                self.z_from = self.pinch[2] if self.pinch is not None else self.z_above
                self._set_state('SUBIENDO')
            elif self.state == 'SUBIENDO' and el > 1.5:
                self._send(self.standby, force=True)
                self._set_state('ESPERA')
            return

        p_cmd = self._predict(fit, now + self.lead)   # hacia donde apuntar (adelantado)
        p_now = self._predict(fit, now)               # donde esta ahora
        self._publish_markers(fit, p_now)

        if self.state in ('ALINEANDO', 'BAJANDO') and not self._in_window(p_cmd):
            self.get_logger().warn('El cubo salio de la ventana alcanzable: abortando.')
            self.z_from = self.pinch[2]
            self._set_state('SUBIENDO')
            return

        z_grasp = p_now[2] + self.grasp_dz

        if self.state == 'ALINEANDO':
            self._track([p_cmd[0], p_cmd[1], self.z_above])
            err_xy = np.linalg.norm(self.pinch[:2] - p_now[:2])
            if err_xy < self.align_tol:
                self.aligned_since = self.aligned_since or now
                if now - self.aligned_since > 0.4:
                    self.get_logger().info(f'Alineado sobre el cubo (error xy {err_xy * 1000:.1f} mm). Bajando.')
                    self._set_state('BAJANDO')
            else:
                self.aligned_since = None
            if el > self.t_align_max:
                self.get_logger().warn('No se logro alinear a tiempo: abortando.')
                self.z_from = self.pinch[2]
                self._set_state('SUBIENDO')
            return

        if self.state == 'BAJANDO':
            f = min(1.0, el / self.t_descend)
            f = f * f * (3 - 2 * f)          # suavizado (smoothstep)
            z = self.z_above + (z_grasp - self.z_above) * f
            self._track([p_cmd[0], p_cmd[1], z])
            grasp_pt = np.array([p_now[0], p_now[1], z_grasp])
            err = np.linalg.norm(self.pinch - grasp_pt)
            if f >= 1.0 and (err < self.intercept_tol or el > self.t_descend + 1.5):
                self.hit_point = self.pinch.copy()
                self.n_intercepts += 1
                tag = 'INTERCEPTADO' if err < self.intercept_tol else 'intercepcion con error alto'
                self.get_logger().info(
                    f'*** {tag} #{self.n_intercepts}: cubo en {np.round(p_now, 3).tolist()}, '
                    f'pinch_site en {np.round(self.pinch, 3).tolist()} '
                    f'(error {err * 1000:.1f} mm; xy {np.linalg.norm(self.pinch[:2] - p_now[:2]) * 1000:.1f} mm) ***')
                self.hit_pub.publish(Bool(data=True))
                self.hit_errors = []
                self._set_state('INTERCEPTADO')
            return

        if self.state == 'INTERCEPTADO':
            self._track([p_cmd[0], p_cmd[1], z_grasp])
            self.hit_errors.append(np.linalg.norm(self.pinch - np.array([p_now[0], p_now[1], z_grasp])))
            if el > self.t_hold:
                e = np.array(self.hit_errors) * 1000
                self.get_logger().info(
                    f'Acompanando al cubo {self.t_hold:.1f} s: error medio {e.mean():.1f} mm, '
                    f'maximo {e.max():.1f} mm. Subiendo.')
                self.hit_pub.publish(Bool(data=False))
                self.z_from = self.pinch[2]
                self.lap_resets = self.est.resets
                self._set_state('SUBIENDO')
            return

        if self.state == 'SUBIENDO':
            f = min(1.0, el / 1.0)
            z = self.z_from + (self.z_above - self.z_from) * f
            self._track([p_cmd[0], p_cmd[1], z] if self._in_window(p_cmd)
                        else [self.pinch[0], self.pinch[1], z])
            if el > 1.5:
                self._send(self.standby, force=True)
                if not self.loop and self.n_intercepts > 0:
                    self._set_state('FIN')
                else:
                    self._set_state('ESPERA')
            return

    # ------------------------------------------------------------------ RViz
    def _publish_markers(self, fit, p_now):
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()

        line = Marker()
        line.header.stamp = stamp
        line.header.frame_id = 'base_link'
        line.ns, line.id, line.type, line.action = 'prediccion', 0, Marker.LINE_STRIP, Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.004
        line.color.r, line.color.g, line.color.b, line.color.a = 0.1, 0.6, 1.0, 0.9
        t_ref = fit[0]
        for k in range(0, 21):
            p = self._predict(fit, t_ref + 0.5 * k)
            pt = Point()
            pt.x, pt.y, pt.z = float(p[0]), float(p[1]), float(p[2])
            line.points.append(pt)
        arr.markers.append(line)

        if self.hit_point is not None:
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = 'base_link'
            m.ns, m.id, m.type, m.action = 'intercepcion', 0, Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, self.hit_point)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.025
            m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 1.0, 0.2, 0.9
            arr.markers.append(m)

        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = InterceptorNode()
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
