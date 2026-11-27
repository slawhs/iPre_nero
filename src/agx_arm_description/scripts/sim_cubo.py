"""
sim_cubo.py — Cinta transportadora del cubo para el puente MuJoCo
==================================================================
Mueve el cubo "cubo_movil" (joint libre "cubo_free") e informa su estado.

  escucha  /control/cubo            Float64MultiArray  [x, y, z, vx, vy, vz, duracion]
  publica  /feedback/cubo_pose      PoseStamped        pose del cubo en base_link
  publica  /feedback/cubo_contacto  Bool               el cubo toca el robot
  publica  /feedback/cubo_agarre    Bool               ambos dedos tocan el cubo

/control/cubo coloca el cubo en (x, y, z) (marco mundo) y pone la cinta a
velocidad (vx, vy, vz) durante "duracion" segundos de simulacion; despues la cinta
se para (velocidad 0) y retiene el cubo.

La cinta actua por fuerzas, como una real: mientras el cubo esta a la altura de la
cinta, el rozamiento lleva su velocidad a la de la cinta, un resorte lo mantiene a
esa altura y derecho, y se amortigua su giro. Si algo lo bloquea, la cinta lo sigue
empujando contra el obstaculo. Al levantarlo (mas de ALTURA_CONTACTO) queda libre.
"""

import numpy as np

import mujoco
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64MultiArray

CUBE_BODY = 'cubo_movil'
CUBE_JOINT = 'cubo_free'
BASE_BODY = 'base_link'
FINGER_BODIES = ('gripper_link1', 'gripper_link2')

ROZAMIENTO = 2.0          # N/(m/s), rozamiento con la cinta
RIGIDEZ_ALTURA = 20.0     # N/m, mantiene el cubo a la altura de la cinta
AMORT_ALTURA = 0.5        # N/(m/s)
RIGIDEZ_GIRO = 0.01       # N·m/rad, mantiene el cubo derecho
AMORT_GIRO = 5e-5         # N·m/(rad/s)
ALTURA_CONTACTO = 0.01    # m, por encima la cinta no toca el cubo


class SimCubo:
    def __init__(self, node, model, data, lock, rate_hz=100.0):
        self.node = node
        self.model = model
        self.data = data
        self.lock = lock
        self.cinta = None   # (z_cinta, v_cinta, t_fin) en tiempo de simulacion

        self.body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, CUBE_BODY)
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, CUBE_JOINT)
        if self.body < 0 or joint < 0:
            node.get_logger().info('Cinta del cubo: sin cubo en la escena, desactivada.')
            return

        self.qadr = int(model.jnt_qposadr[joint])
        self.vadr = int(model.jnt_dofadr[joint])

        # Cuerpos del robot: base_link y todos sus descendientes.
        base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, BASE_BODY)
        self.robot_bodies = {b for b in range(model.nbody) if self._desciende(b, base)}
        self.fingers = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n)
                        for n in FINGER_BODIES}

        mujoco.mj_forward(model, data)
        self.base_p = data.xpos[base].copy()
        self.base_R = data.xmat[base].reshape(3, 3).copy()

        self.contacto = False
        self.agarre = False

        node.create_subscription(Float64MultiArray, '/control/cubo', self._on_cmd, 10)
        self.pub_pose = node.create_publisher(PoseStamped, '/feedback/cubo_pose', 10)
        self.pub_contacto = node.create_publisher(Bool, '/feedback/cubo_contacto', 10)
        self.pub_agarre = node.create_publisher(Bool, '/feedback/cubo_agarre', 10)
        node.create_timer(1.0 / rate_hz, self._publish)
        node.get_logger().info('Cinta del cubo: escuchando /control/cubo.')

    def _desciende(self, b, ancestro):
        while b > 0:
            if b == ancestro:
                return True
            b = int(self.model.body_parentid[b])
        return False

    def _on_cmd(self, msg):
        if len(msg.data) != 7:
            self.node.get_logger().warn('/control/cubo espera [x, y, z, vx, vy, vz, duracion].')
            return
        p0 = np.array(msg.data[0:3])
        v = np.array(msg.data[3:6])
        with self.lock:
            q, qd = self.data.qpos, self.data.qvel
            q[self.qadr:self.qadr + 3] = p0
            q[self.qadr + 3:self.qadr + 7] = (1.0, 0.0, 0.0, 0.0)
            qd[self.vadr:self.vadr + 3] = v
            qd[self.vadr + 3:self.vadr + 6] = 0.0
            self.cinta = (float(p0[2]), v, self.data.time + float(msg.data[6]))
            self.contacto = False
            self.agarre = False
        self.node.get_logger().info(
            f'Cinta del cubo: desde {np.round(p0, 3)} a {np.round(v, 3)} m/s '
            f'durante {msg.data[6]:.1f} s.')

    def step(self):
        """Fuerzas de la cinta sobre el cubo. Llamar antes de mj_step, con el lock tomado."""
        if self.cinta is None:
            return
        f = self.data.xfrc_applied[self.body]
        f[:] = 0.0
        z_cinta, v, t_fin = self.cinta
        z = self.data.xpos[self.body][2]
        if z - z_cinta > ALTURA_CONTACTO:
            return  # levantado: la cinta no lo toca
        if self.data.time >= t_fin:
            v = np.zeros(3)  # cinta parada: retiene el cubo
        vel = self.data.qvel[self.vadr:self.vadr + 3]
        w = self.data.qvel[self.vadr + 3:self.vadr + 6]   # marco local del cubo
        R = self.data.xmat[self.body].reshape(3, 3)

        f[0:2] = ROZAMIENTO * (v[0:2] - vel[0:2])
        f[2] = RIGIDEZ_ALTURA * (z_cinta - z) - AMORT_ALTURA * vel[2]
        f[3:6] = (RIGIDEZ_GIRO * np.cross(R[:, 2], [0.0, 0.0, 1.0])
                  - AMORT_GIRO * (R @ w))

    def _leer_contactos(self):
        robot = False
        dedos = set()
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            b1 = int(self.model.geom_bodyid[c.geom1])
            b2 = int(self.model.geom_bodyid[c.geom2])
            if self.body not in (b1, b2):
                continue
            otro = b2 if b1 == self.body else b1
            if otro in self.robot_bodies:
                robot = True
            if otro in self.fingers:
                dedos.add(otro)
        return robot, dedos == self.fingers

    def _publish(self):
        with self.lock:
            robot, agarre = self._leer_contactos()
            if robot and not self.contacto:
                self.node.get_logger().info('Cinta del cubo: el cubo toca el robot.')
            self.contacto = self.contacto or robot
            self.agarre = agarre
            p_w = self.data.xpos[self.body].copy()
            R_w = self.data.xmat[self.body].reshape(3, 3).copy()

        p_b = self.base_R.T @ (p_w - self.base_p)
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, (self.base_R.T @ R_w).flatten())

        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = BASE_BODY
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, p_b)
        msg.pose.orientation.w, msg.pose.orientation.x = float(q[0]), float(q[1])
        msg.pose.orientation.y, msg.pose.orientation.z = float(q[2]), float(q[3])
        self.pub_pose.publish(msg)
        self.pub_contacto.publish(Bool(data=self.contacto))
        self.pub_agarre.publish(Bool(data=self.agarre))
