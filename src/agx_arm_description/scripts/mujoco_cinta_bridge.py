#!/usr/bin/env python3
"""
mujoco_cinta_bridge.py — Puente ROS2 <-> MuJoCo: brazo Nero + cinta + cubo (Tarea, Parte 2)
==========================================================================================
Extiende `MujocoBridge` (mujoco_viewer.py) SIN cambiar su interfaz: el brazo se sigue
comandando con control/move_j (y move_p/l/c) y publica feedback/joint_states y
feedback/tcp_pose igual que antes. Ademas:

  * Compone la escena con MjSpec (igual que el notebook del grupo, "Codigo 1.ipynb"):
      scene_nero_arm.xml  +  cinta.xml (adjuntada con prefijo "cinta_" en --cinta-pos/--cinta-yaw)
      +  un cubo de 4 cm con freejoint (cuerpo "cubo") apoyado sobre la banda.
  * Gravedad REAL para el cubo; el brazo usa gravcomp por cuerpo (body.gravcomp = 1),
    que reproduce la compensacion de gravedad del brazo real (antes se anulaba la
    gravedad de todo el mundo, lo que dejaria el cubo flotando).
  * Mueve el cubo en linea recta a velocidad constante a lo largo de la banda. El
    movimiento es cinematico (se impone la coordenada a lo largo de la cinta y la lateral,
    con su velocidad), la altura la resuelve la fisica (apoyo sobre la banda). Si el cubo
    deja de estar sobre la banda (lo levantan) o lo agarran los DOS dedos, la cinta deja
    de imponerle el movimiento (preparado para la Parte 3).
  * Al llegar al final de la banda el cubo vuelve al inicio (--no-loop lo desactiva).

Topicos nuevos:
  Publica   cube/pose         geometry_msgs/PoseStamped    pose del cubo en base_link (100 Hz)
            cube/twist        geometry_msgs/TwistStamped   velocidad real del cubo (100 Hz, referencia)
            scene/markers     visualization_msgs/MarkerArray  cinta + cubo para RViz (10 Hz)
  Suscribe  conveyor/speed    std_msgs/Float64             cambia la velocidad de la cinta (m/s, >= 0)
            cube/reset        std_msgs/Empty               devuelve el cubo al inicio de la banda

Uso (lo invoca display_mujoco.launch.py con viewer_script:=.../mujoco_cinta_bridge.py):
  python3 mujoco_cinta_bridge.py <ruta>/scene_nero_arm.xml [--cinta-pos -0.45 0 0]
          [--cinta-yaw 1.5708] [--cube-speed 0.04] [--cube-dir -1] [--cube-start-delay 8]
"""

import math
import os
import sys
import threading

import numpy as np

import mujoco

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Empty, Float64
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mujoco_viewer as mv  # noqa: E402  (MujocoBridge, build_arg_parser, run_bridge)

CUBE_BODY = 'cubo'
CUBE_JOINT = 'cubo_libre'
CUBE_GEOM = 'cubo_geom'
BELT_GEOM = 'cinta_banda_superior'
FINGER_BODIES = ('gripper_link1', 'gripper_link2')


class CintaBridge(mv.MujocoBridge):
    def __init__(self, args):
        self._args = args
        # gravcomp=False: NO anular la gravedad global; el brazo se compensa por cuerpo.
        super().__init__(args.scene, args.control_topic, args.feedback_topic, args.rate,
                         args.move_j_topic, args.joint_speed, args.lin_speed,
                         False, args.flange_offset, args.flange_zero_identity)

        m = self.model
        # El freejoint del cubo no es un joint del robot: no se publica en feedback/joint_states.
        self._joint_info.pop(CUBE_JOINT, None)

        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, CUBE_JOINT)
        self._cube_qadr = int(m.jnt_qposadr[jid])
        self._cube_dadr = int(m.jnt_dofadr[jid])
        self._cube_bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, CUBE_BODY)
        self._cube_gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, CUBE_GEOM)
        self._cube_half = float(m.geom_size[self._cube_gid][0])
        self._finger_bids = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b) for b in FINGER_BODIES}

        # Geometria de la banda (en el mundo): eje a lo largo, eje lateral, altura superior.
        mujoco.mj_forward(m, self.data)
        g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, BELT_GEOM)
        if g < 0:
            raise RuntimeError(f'No se encontro la banda "{BELT_GEOM}" en la escena')
        R = self.data.geom_xmat[g].reshape(3, 3)
        size = m.geom_size[g]
        self._belt_c = self.data.geom_xpos[g].copy()
        self._belt_u = R[:, 0].copy()                 # direccion a lo largo de la banda
        self._belt_w = R[:, 1].copy()                 # direccion lateral
        self._belt_half_len = float(size[0])
        self._belt_half_wid = float(size[1])
        self._belt_top = float(self._belt_c[2] + size[2])
        self._belt_size = size.copy()
        self._belt_R = R.copy()

        # Estado de la cinta
        self._speed = max(0.0, float(args.cube_speed))
        self._dir = 1.0 if args.cube_dir >= 0 else -1.0
        margin = self._cube_half + 0.01
        self._s_start = -self._dir * (self._belt_half_len - margin)   # extremo de entrada
        self._s_end = self._dir * (self._belt_half_len - margin)      # extremo de salida
        self._lat = float(args.cube_lateral)
        self._loop = not args.no_loop
        self._start_delay = float(args.cube_start_delay)
        self._s = self._s_start
        self._driving = True
        self._wait_until = self.data.time + self._start_delay
        self._cube_lock = threading.Lock()  # protege speed / reset (callbacks ROS)
        self._reset_request = False
        self._place_cube_at(self._s)

        self.cube_pose_pub = self.create_publisher(PoseStamped, 'cube/pose', 10)
        self.cube_twist_pub = self.create_publisher(TwistStamped, 'cube/twist', 10)
        self.markers_pub = self.create_publisher(MarkerArray, 'scene/markers', 10)
        self.create_subscription(Float64, 'conveyor/speed', self._on_speed, 10)
        self.create_subscription(Empty, 'cube/reset', self._on_reset, 10)
        self.create_timer(0.01, self._publish_cube)
        self.create_timer(0.1, self._publish_markers)

        self.get_logger().info(
            f'Cinta: centro={np.round(self._belt_c, 3).tolist()} eje={np.round(self._belt_u, 3).tolist()} '
            f'largo={2 * self._belt_half_len:.3f} m, superficie z={self._belt_top:.3f} m. '
            f'Cubo de {2 * self._cube_half * 100:.0f} cm a {self._speed:.3f} m/s '
            f'(sentido {"+" if self._dir > 0 else "-"}, loop={self._loop}, '
            f'parte en {self._start_delay:.1f} s).')

    # ----------------------------------------------------------- composicion de escena
    def _load_model(self, scene_path):
        a = self._args
        spec = mujoco.MjSpec.from_file(scene_path)

        # Los keyframes de nero_arm.xml tienen el tamano de qpos SIN el cubo: se eliminan
        # (el bridge parte en qpos = 0, igual que el keyframe 'home').
        for key in list(spec.keys):
            if hasattr(spec, 'delete'):
                spec.delete(key)
            else:              # versiones antiguas de MuJoCo
                key.delete()

        # Compensacion de gravedad por cuerpo para todo el brazo (incluye la pinza).
        for body in spec.bodies:
            if body.name and body.name != 'world':
                body.gravcomp = 1.0

        # Cinta del grupo (cinta.xml), en el mismo directorio que la escena.
        cinta_path = a.cinta_xml or os.path.join(os.path.dirname(scene_path), 'cinta.xml')
        spec_cinta = mujoco.MjSpec.from_file(cinta_path)
        frame = spec.worldbody.add_frame(pos=list(a.cinta_pos), euler=[0.0, 0.0, a.cinta_yaw])
        spec.attach(spec_cinta, prefix='cinta_', frame=frame)

        # Cubo de 4 cm con freejoint (la posicion real se fija despues sobre la banda).
        half = a.cube_size / 2.0
        cube = spec.worldbody.add_body(name=CUBE_BODY, pos=[0.0, 0.0, 0.3])
        cube.add_freejoint(name=CUBE_JOINT)
        cube.add_geom(name=CUBE_GEOM, type=mujoco.mjtGeom.mjGEOM_BOX, size=[half, half, half],
                      mass=a.cube_mass, rgba=[0.9, 0.15, 0.15, 1.0],
                      friction=[1.0, 0.005, 0.0001], condim=4)

        model = spec.compile()
        return model

    # --------------------------------------------------------------- utilidades
    def _place_cube_at(self, s):
        """Pone el cubo quieto sobre la banda en la coordenada s (a lo largo)."""
        p = (self._belt_c + self._belt_u * s + self._belt_w * self._lat)
        p[2] = self._belt_top + self._cube_half + 1e-4
        qa, da = self._cube_qadr, self._cube_dadr
        self.data.qpos[qa:qa + 3] = p
        yaw = math.atan2(self._belt_u[1], self._belt_u[0])
        self.data.qpos[qa + 3:qa + 7] = [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
        self.data.qvel[da:da + 6] = 0.0

    def _cube_grasped(self):
        """True si el cubo toca los DOS dedos de la pinza."""
        touched = set()
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            b1 = self.model.geom_bodyid[c.geom1]
            b2 = self.model.geom_bodyid[c.geom2]
            if b1 == self._cube_bid and b2 in self._finger_bids:
                touched.add(b2)
            elif b2 == self._cube_bid and b1 in self._finger_bids:
                touched.add(b1)
        return len(touched) == 2

    def _on_belt(self, p):
        d = p - self._belt_c
        s = float(d @ self._belt_u)
        lat = float(d @ self._belt_w)
        bottom = p[2] - self._cube_half
        return (abs(s) <= self._belt_half_len and abs(lat) <= self._belt_half_wid
                and bottom < self._belt_top + 0.005), s

    # -------------------------------------------------------------------- fisica
    def step(self):
        with self._data_lock:
            self._advance_trajectory()
            mujoco.mj_step(self.model, self.data)
            self._drive_cube()

    def _drive_cube(self):
        dt = float(self.model.opt.timestep)
        qa, da = self._cube_qadr, self._cube_dadr
        with self._cube_lock:
            speed = self._speed
            reset = self._reset_request
            self._reset_request = False
        if reset:
            self._s = self._s_start
            self._driving = True
            self._wait_until = self.data.time + 0.5
            self._place_cube_at(self._s)
            return

        p = self.data.qpos[qa:qa + 3].copy()
        on_belt, s_now = self._on_belt(p)

        if self._driving:
            if not on_belt or self._cube_grasped():
                # Lo levantaron o lo agarraron: la cinta ya no lo mueve.
                self._driving = False
                self.get_logger().info('Cubo fuera de la cinta / agarrado: la cinta deja de moverlo.')
                return
            v = 0.0 if self.data.time < self._wait_until else speed
            self._s += self._dir * v * dt
            if self._dir * (self._s - self._s_end) >= 0.0:
                # Llego al final de la banda.
                self._s = self._s_end
                if self._loop:
                    self._s = self._s_start
                    self._wait_until = self.data.time + 1.0
                    self._place_cube_at(self._s)
                    self.get_logger().info('Cubo llego al final de la cinta: vuelve al inicio.')
                    return
                v = 0.0
            # Movimiento cinematico en el plano de la banda; z y la rotacion los resuelve la fisica.
            target = self._belt_c + self._belt_u * self._s + self._belt_w * self._lat
            self.data.qpos[qa + 0] = target[0]
            self.data.qpos[qa + 1] = target[1]
            vel = self._belt_u * (self._dir * v)
            self.data.qvel[da + 0] = vel[0]
            self.data.qvel[da + 1] = vel[1]
            self.data.qvel[da + 3:da + 6] = 0.0  # sin giros
            yaw = math.atan2(self._belt_u[1], self._belt_u[0])
            self.data.qpos[qa + 3:qa + 7] = [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
        else:
            # Si el cubo vuelve a quedar suelto sobre la banda, la cinta lo retoma.
            if on_belt and not self._cube_grasped() and abs(self.data.qvel[da + 2]) < 1e-3:
                self._s = s_now
                self._driving = True
                self.get_logger().info('Cubo de nuevo sobre la cinta: la cinta lo mueve.')
            elif p[2] < 0.0 or (not on_belt and p[2] < self._belt_top - 0.02):
                # Se cayo de la cinta: reiniciar.
                with self._cube_lock:
                    self._reset_request = True

    # --------------------------------------------------------------- callbacks ROS
    def _on_speed(self, msg: Float64):
        with self._cube_lock:
            self._speed = max(0.0, float(msg.data))
        self.get_logger().info(f'Velocidad de la cinta: {self._speed:.3f} m/s')

    def _on_reset(self, _msg: Empty):
        with self._cube_lock:
            self._reset_request = True
        self.get_logger().info('Reinicio del cubo solicitado.')

    def _publish_cube(self):
        stamp = self.get_clock().now().to_msg()
        with self._data_lock:
            qa, da = self._cube_qadr, self._cube_dadr
            p_w = self.data.qpos[qa:qa + 3].copy()
            q_w = self.data.qpos[qa + 3:qa + 7].copy()
            v_w = self.data.qvel[da:da + 3].copy()
        p_b = self._base_R.T @ (p_w - self._base_p)
        q_b = mv.mat_to_quat(self._base_R.T @ mv.quat_to_mat(q_w))
        v_b = self._base_R.T @ v_w

        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = mv.BASE_BODY
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, p_b)
        ps.pose.orientation.w, ps.pose.orientation.x = float(q_b[0]), float(q_b[1])
        ps.pose.orientation.y, ps.pose.orientation.z = float(q_b[2]), float(q_b[3])
        self.cube_pose_pub.publish(ps)

        tw = TwistStamped()
        tw.header.stamp = stamp
        tw.header.frame_id = mv.BASE_BODY
        tw.twist.linear.x, tw.twist.linear.y, tw.twist.linear.z = map(float, v_b)
        self.cube_twist_pub.publish(tw)

    def _publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        with self._data_lock:
            qa = self._cube_qadr
            p_c = self.data.qpos[qa:qa + 3].copy()
            q_c = self.data.qpos[qa + 3:qa + 7].copy()
        arr = MarkerArray()

        belt = Marker()
        belt.header.stamp = stamp
        belt.header.frame_id = mv.BASE_BODY
        belt.ns, belt.id, belt.type, belt.action = 'cinta', 0, Marker.CUBE, Marker.ADD
        c = self._base_R.T @ (self._belt_c - self._base_p)
        belt.pose.position.x, belt.pose.position.y, belt.pose.position.z = map(float, c)
        qb = mv.mat_to_quat(self._base_R.T @ self._belt_R)
        belt.pose.orientation.w, belt.pose.orientation.x = float(qb[0]), float(qb[1])
        belt.pose.orientation.y, belt.pose.orientation.z = float(qb[2]), float(qb[3])
        belt.scale.x, belt.scale.y, belt.scale.z = map(float, 2.0 * self._belt_size)
        belt.color.r, belt.color.g, belt.color.b, belt.color.a = 0.15, 0.15, 0.15, 1.0
        arr.markers.append(belt)

        cube = Marker()
        cube.header.stamp = stamp
        cube.header.frame_id = mv.BASE_BODY
        cube.ns, cube.id, cube.type, cube.action = 'cubo', 0, Marker.CUBE, Marker.ADD
        p = self._base_R.T @ (p_c - self._base_p)
        cube.pose.position.x, cube.pose.position.y, cube.pose.position.z = map(float, p)
        qq = mv.mat_to_quat(self._base_R.T @ mv.quat_to_mat(q_c))
        cube.pose.orientation.w, cube.pose.orientation.x = float(qq[0]), float(qq[1])
        cube.pose.orientation.y, cube.pose.orientation.z = float(qq[2]), float(qq[3])
        cube.scale.x = cube.scale.y = cube.scale.z = float(2.0 * self._cube_half)
        cube.color.r, cube.color.g, cube.color.b, cube.color.a = 0.9, 0.15, 0.15, 1.0
        arr.markers.append(cube)

        self.markers_pub.publish(arr)


def main():
    parser = mv.build_arg_parser('Puente ROS2 <-> MuJoCo: Nero + cinta transportadora + cubo')
    parser.add_argument('--cinta-xml', default='', help='Ruta a cinta.xml (default: junto a la escena)')
    parser.add_argument('--cinta-pos', type=float, nargs=3, default=[-0.45, 0.0, 0.0],
                        metavar=('X', 'Y', 'Z'), help='Posicion de la cinta en el mundo (m)')
    parser.add_argument('--cinta-yaw', type=float, default=1.5708,
                        help='Giro de la cinta en torno a z (rad). 1.5708 = a lo largo del eje y')
    parser.add_argument('--cube-size', type=float, default=0.04, help='Arista del cubo (m)')
    parser.add_argument('--cube-mass', type=float, default=0.05, help='Masa del cubo (kg)')
    parser.add_argument('--cube-speed', type=float, default=0.04, help='Velocidad de la cinta (m/s)')
    parser.add_argument('--cube-dir', type=float, default=-1.0,
                        help='Sentido sobre la banda: -1 (default, con yaw=90 va de +y a -y) o +1')
    parser.add_argument('--cube-lateral', type=float, default=0.0,
                        help='Desplazamiento lateral del cubo respecto al centro de la banda (m)')
    parser.add_argument('--cube-start-delay', type=float, default=8.0,
                        help='Segundos (tiempo simulado) que el cubo espera antes de partir')
    parser.add_argument('--no-loop', action='store_true',
                        help='No devolver el cubo al inicio al llegar al final')
    args, _ros_args = parser.parse_known_args()

    rclpy.init(args=sys.argv)
    bridge = CintaBridge(args)
    mv.run_bridge(bridge)


if __name__ == '__main__':
    main()
