"""
tracking_ik.py — IK rapida para seguir objetivos en movimiento (Tarea Nero, Parte 2)
====================================================================================
Mismo modelo (`/model_path`, sitio `pinch_site`) y misma idea que ik_node (Jacobiano
del sitio + error de posicion y orientacion), pero pensada para llamarse a 50 Hz:

  * Parte desde la ULTIMA SOLUCION (no desde el feedback). ik_node parte desde el estado
    medido del brazo y publica cada iteracion; con un objetivo que se mueve, cada nuevo
    calculo "tira" el comando hacia atras (hacia donde esta el brazo, que va con retraso)
    y el brazo termina siguiendo al cubo a ~la mitad de su velocidad. Se verifico en
    simulacion: con ik_node el error de seguimiento crecia hasta > 7 cm.
  * Minimos cuadrados amortiguados (damped least squares) para ser robusta cerca de
    singularidades, con pocas iteraciones por llamada (el objetivo cambia poco entre llamadas).
  * Respeta los limites articulares del modelo.
"""

import mujoco
import numpy as np


def rpy_to_matrix(rpy):
    """R = Rz(yaw) @ Ry(pitch) @ Rx(roll), igual que ik_node.rpy_to_rotation_matrix."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def rotation_error(R_target, R_current):
    """Vector eje*angulo (marco mundo) que lleva R_current a R_target."""
    Re = R_target @ R_current.T
    angle = np.arccos(np.clip((np.trace(Re) - 1.0) / 2.0, -1.0, 1.0))
    if angle < 1e-9:
        return np.zeros(3)
    axis = np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
    n = np.linalg.norm(axis)
    if n < 1e-9:
        return np.zeros(3)
    return axis / n * angle


class TrackingIK:
    def __init__(self, model_path, site='pinch_site', n_arm=7, damping=0.02):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site)
        if self.site < 0:
            raise RuntimeError(f'El modelo {model_path} no tiene el sitio "{site}"')
        self.n = n_arm
        self.lam2 = damping ** 2
        self.lo = self.model.jnt_range[:n_arm, 0].copy()
        self.hi = self.model.jnt_range[:n_arm, 1].copy()
        self.jacp = np.zeros((3, self.model.nv))
        self.jacr = np.zeros((3, self.model.nv))
        self.q = None  # ultima solucion (semilla)

    def reset(self, q_seed):
        self.q = np.array(q_seed[:self.n], dtype=float)

    def fk(self, q):
        self.data.qpos[:self.n] = q
        mujoco.mj_forward(self.model, self.data)
        return self.data.site_xpos[self.site].copy(), self.data.site_xmat[self.site].reshape(3, 3).copy()

    def solve(self, pos, R, iters=15, tol=1e-4, max_dq=0.08):
        """Una actualizacion: devuelve (q, error) partiendo de la solucion anterior."""
        if self.q is None:
            raise RuntimeError('TrackingIK.reset(q) antes de solve()')
        q = self.q.copy()
        err = np.inf
        for _ in range(iters):
            p, Rc = self.fk(q)
            e = np.concatenate([pos - p, rotation_error(R, Rc)])
            err = float(np.linalg.norm(e))
            if err < tol:
                break
            mujoco.mj_jacSite(self.model, self.data, self.jacp, self.jacr, self.site)
            J = np.vstack([self.jacp, self.jacr])[:, :self.n]
            dq = J.T @ np.linalg.solve(J @ J.T + self.lam2 * np.eye(6), e)
            nd = np.linalg.norm(dq)
            if nd > max_dq:
                dq *= max_dq / nd
            q = np.clip(q + dq, self.lo, self.hi)
        self.q = q
        return q, err
