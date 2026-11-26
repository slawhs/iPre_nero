#!/usr/bin/env python3

"""Puente de teleoperación: conecta el Meta Quest (QuestArmTeleop) con la IK de nero_ctrl.

Del QuestArmTeleop solo se usan pub_pose.py y pub_delta_pose.py. Aquí no hay
cinemática: el nodo traduce la pose objetivo del Quest al formato de ik_node.
La pose que se controla es la del pinch_site (centro del gripper), no link7.

  Quest -> /delta_pose -> quest_teleop -> /obj_point -> ik_node -> /ik/ik_result
        -> can_module -> /control/move_j

Tópicos usados:

  publica  /obj_point              std_msgs/Float64MultiArray -> [x, y, z, roll, pitch, yaw, gripper]
  publica  /feedback/pinch_pose    geometry_msgs/PoseStamped  -> pose actual del pinch_site
  escucha  /delta_pose             geometry_msgs/PoseStamped  -> pose objetivo del pinch_site
  escucha  /feedback/tcp_pose      geometry_msgs/PoseStamped  -> pose actual de link7
  escucha  /control/joint_states   sensor_msgs/JointState     -> apertura pedida con el gatillo

pub_delta_pose.py tiene que usar /feedback/pinch_pose como referencia:

  ros2 run oculus_reader pub_delta_pose.py --ros-args \\
      -p feedback_tcp_pose_topic:=/feedback/pinch_pose -p gripper_max_range:=0.05

Solo probado en simulación.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


# ============================================================
# CONSTANTES
# ============================================================

# Posición del pinch_site en el marco de link7 (misma orientación que link7).
# Sale de models_for_simulation/agilex_nero/nero.xml, el modelo que usa ik_node.
PINCH_OFFSET = [0.1733, 0.0, -0.0235]

# ik_node recorta la apertura a este rango
GRIPPER_MIN = 0.0
GRIPPER_MAX = 0.05

# Etapas de la máquina de estados
WAIT_FEEDBACK = "wait_feedback"
FOLLOWING = "following"


class QuestTeleop(Node):
    def __init__(self):
        super().__init__("quest_teleop")

        #? --- Atributos ---

        self.stage = WAIT_FEEDBACK

        self.pinch_pos = None           # pose actual del pinch_site
        self.target = None              # último objetivo sin enviar (los viejos se descartan)
        self.gripper_width = GRIPPER_MAX

        self.setup_parameters()
        self.init_communications()

        self.get_logger().info("Puente del Quest listo. Esperando feedback del controlador...")

    def setup_parameters(self):
        self.declare_parameter("rate_hz", 20.0)   # Hz, máximo de objetivos enviados a la IK
        # La IK publica un resultado aunque no converja: no se le mandan saltos grandes
        self.declare_parameter("max_jump", 0.15)  # m, distancia máxima entre pinch_site y objetivo

        self.rate_hz = self.get_parameter("rate_hz").value
        self.max_jump = self.get_parameter("max_jump").value

    def init_communications(self):
        # Publishers
        # ik_node se suscribe a /obj_point como TRANSIENT_LOCAL
        obj_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_obj = self.create_publisher(Float64MultiArray, "/obj_point", obj_qos)
        self.pub_pinch = self.create_publisher(PoseStamped, "/feedback/pinch_pose", 10)

        # Subscribers
        self.create_subscription(PoseStamped, "/delta_pose", self.target_cb, 10)
        self.create_subscription(PoseStamped, "/feedback/tcp_pose", self.tcp_cb, 10)
        self.create_subscription(JointState, "/control/joint_states", self.gripper_cb, 10)

        # Timers
        self.timer = self.create_timer(1.0 / self.rate_hz, self.routine_cb)


    # ======= Callbacks feedback =======
    def tcp_cb(self, msg):           #? Pose de link7 -> pose del pinch_site
        o = msg.pose.orientation
        offset = Rotation.from_quat([o.x, o.y, o.z, o.w]).apply(PINCH_OFFSET)

        p = msg.pose.position
        self.pinch_pos = (p.x + offset[0], p.y + offset[1], p.z + offset[2])

        pinch = PoseStamped()
        pinch.header = msg.header

        (pinch.pose.position.x,
         pinch.pose.position.y,
         pinch.pose.position.z) = map(float, self.pinch_pos)
        pinch.pose.orientation = o

        self.pub_pinch.publish(pinch)
    # ==================================


    # ======= Callbacks comandos =======
    def target_cb(self, msg):        #? Objetivo del Quest: solo interesa el último
        self.target = msg.pose

    def gripper_cb(self, msg):       #? Apertura del gatillo, para que la IK no la pise
        if "gripper" in msg.name:
            i = msg.name.index("gripper")
            if i < len(msg.position):
                self.gripper_width = min(max(float(msg.position[i]), GRIPPER_MIN), GRIPPER_MAX)
    # ==================================


    def send_target(self, pose):
        p = pose.position
        o = pose.orientation
        position = (p.x, p.y, p.z)
        quat = (o.x, o.y, o.z, o.w)

        if not all(map(math.isfinite, position + quat)) or math.hypot(*quat) < 1e-6:
            self.get_logger().warn("Objetivo inválido: descartado.", throttle_duration_sec=1.0)
            return

        jump = math.dist(position, self.pinch_pos)

        if jump > self.max_jump:
            self.get_logger().warn(
                f"Objetivo a {100 * jump:.0f} cm del gripper (máximo {100 * self.max_jump:.0f} cm): descartado.",
                throttle_duration_sec=1.0
            )
            return

        # ik_node compone la rotación como Rz @ Ry @ Rx: ejes fijos "xyz"
        roll, pitch, yaw = Rotation.from_quat(quat).as_euler("xyz")

        msg = Float64MultiArray()
        msg.data = [float(v) for v in (*position, roll, pitch, yaw, self.gripper_width)]

        self.pub_obj.publish(msg)

    def routine_cb(self):
        # ----------------------------------------------------
        # Esperar el feedback y que la IK esté escuchando
        # ----------------------------------------------------
        if self.stage == WAIT_FEEDBACK:
            controller_ready = (
                self.pinch_pos is not None
                and self.pub_obj.get_subscription_count() > 0
            )

            if controller_ready:
                self.get_logger().info("IK detectada. Siguiendo los objetivos de /delta_pose.")
                self.target = None
                self.stage = FOLLOWING

            return

        # ----------------------------------------------------
        # Enviar el objetivo más reciente, si hay uno nuevo
        # ----------------------------------------------------
        if self.stage == FOLLOWING:
            if self.target is None:
                return

            self.send_target(self.target)
            self.target = None

            return


def main(args=None):
    rclpy.init(args=args)              #? Inicializa ROS
    node = QuestTeleop()               #? Instancia de un nodo de clase QuestTeleop

    try:
        rclpy.spin(node)               #? Permite ejecución continua de un nodo
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()           #? Cierra el nodo al terminar la ejecución


if __name__ == "__main__":
    main()
