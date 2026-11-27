#!/usr/bin/env python3
#? ^ Le dice a bash "ejecuta esto con python3"  ^

"""Herramienta de terminal para alimentar a waypoint_manager.py.

Publica rutinas de prueba en /commands/waypoints (y cancelaciones o e-stop) sin
tener que escribir mensajes a mano. No es parte del control: es solo el emisor
que hace de "nodo orquestador" mientras se prueba el ejecutor.

Uso:
    ros2 run grupo2_pkg waypoint_sender <escenario>

Escenarios:
    happy   rutina completa: mover, abrir, mover, cerrar, mover
            (incluye un id duplicado y un comando inválido, que deben rechazarse)
    cancel  encola 3 comandos y cancela uno en cola, uno inexistente y luego todo
    abort   manda una pose inalcanzable: el ejecutor debe abortar y volver a casa
    estop   encola una rutina y dispara el e-stop a mitad del primer movimiento
    home    devuelve el brazo a la pose inicial del launch (sin pasar por la cola)

Las poses están verificadas contra el simulador: son desplazamientos de ±5 cm y
giros de ±45° sobre Z respecto de la pose inicial. Para explorar otras, usa
`ros2 run grupo2_pkg pose_probe`.
"""

import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from grupo2_interfaces.msg import Waypoint
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, UInt16


# ============================================================
# POSES VERIFICADAS (brida en marco base_link)
# ============================================================

ORI_HOME = (0.010325, 0.707031, -0.707031, 0.010325)   # orientación de la pose inicial
ORI_P45 = (-0.261030, 0.657163, -0.649261, 0.280108)   # +45° sobre Z de la base
ORI_M45 = (0.280108, 0.649261, -0.657163, -0.261030)   # -45° sobre Z de la base

POS_BACK = (-0.1992, -0.0235, 0.4156)   # 5 cm hacia atrás
POS_SIDE = (-0.1492, -0.0735, 0.4156)   # 5 cm a un lado
POS_UP = (-0.1492, -0.0235, 0.4656)     # 5 cm arriba
POS_UNREACHABLE = (-0.1492, -0.0235, 1.50)

GRIPPER_OPEN = 0.08
GRIPPER_CLOSED = 0.04

# Pose inicial que publica nero_ctrl/can_module.py al arrancar
ARM_JOINTS = [f"joint{i}" for i in range(1, 8)]
INIT_POSE = [0.0, -0.40, 0.0, 2.0, 0.0, 0.0, 0.0, 0.05]
INIT_EFFORT = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

SCENARIOS = ("happy", "cancel", "abort", "estop", "home")


class WaypointSender(Node):
    def __init__(self):
        super().__init__("waypoint_sender")

        self.init_communications()

    def init_communications(self):
        # Publishers
        self.pub_waypoint = self.create_publisher(Waypoint, "/commands/waypoints", 50)
        self.pub_cancel = self.create_publisher(UInt16, "/commands/cancel_waypoint", 10)
        self.pub_estop = self.create_publisher(Empty, "/commands/estop", 10)
        self.pub_joints = self.create_publisher(JointState, "/control/move_j", 10)

    def wait(self, seconds):
        deadline = time.monotonic() + seconds

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def send_move(self, waypoint_id, pos, ori):
        msg = Waypoint()
        msg.id = waypoint_id
        msg.command = Waypoint.COMMAND_MOVE

        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pos

        (msg.pose.orientation.x,
         msg.pose.orientation.y,
         msg.pose.orientation.z,
         msg.pose.orientation.w) = ori

        self.pub_waypoint.publish(msg)

        self.get_logger().info(f"-> id={waypoint_id} move pos={pos}")

    def send_gripper(self, waypoint_id, width):
        msg = Waypoint()
        msg.id = waypoint_id
        msg.command = Waypoint.COMMAND_GRIPPER
        msg.gripper_width = float(width)

        self.pub_waypoint.publish(msg)

        self.get_logger().info(f"-> id={waypoint_id} gripper {100 * width:.1f} cm")

    def send_raw(self, waypoint_id, command, pos, ori):
        """Mensaje deliberadamente inválido, para comprobar que el ejecutor lo rechaza."""
        msg = Waypoint()
        msg.id = waypoint_id
        msg.command = command

        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pos

        (msg.pose.orientation.x,
         msg.pose.orientation.y,
         msg.pose.orientation.z,
         msg.pose.orientation.w) = ori

        self.pub_waypoint.publish(msg)

        self.get_logger().info(f"-> id={waypoint_id} command='{command}' (debe rechazarse)")

    def send_cancel(self, waypoint_id):
        self.pub_cancel.publish(UInt16(data=waypoint_id))

        self.get_logger().info(f"-> cancel id={waypoint_id}" + (" (todo)" if waypoint_id == 0 else ""))

    def send_estop(self):
        self.pub_estop.publish(Empty())

        self.get_logger().info("-> E-STOP")

    def send_init_pose(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ARM_JOINTS + ["gripper"]
        msg.position = list(INIT_POSE)
        msg.effort = list(INIT_EFFORT)

        self.pub_joints.publish(msg)

        self.get_logger().info("-> pose inicial")

    # ========================================================
    # ESCENARIOS
    # ========================================================

    def happy(self):
        self.send_move(1, POS_BACK, ORI_P45)
        self.send_gripper(2, GRIPPER_OPEN)
        self.send_move(3, POS_SIDE, ORI_M45)
        self.send_gripper(4, GRIPPER_CLOSED)
        self.send_move(5, POS_UP, ORI_HOME)

        # Los dos siguientes deben rechazarse sin tocar la cola
        self.send_move(5, POS_UP, ORI_HOME)
        self.send_raw(6, "mover", POS_UP, ORI_HOME)

    def cancel(self):
        self.send_move(10, POS_BACK, ORI_P45)
        self.send_gripper(11, GRIPPER_OPEN)
        self.send_move(12, POS_SIDE, ORI_M45)

        self.wait(0.3)
        self.send_cancel(11)   # está en la cola: se elimina sin más

        self.wait(1.0)
        self.send_cancel(99)   # no existe: warning

        self.wait(1.0)
        self.send_cancel(0)    # todo, incluido el activo

    def abort(self):
        self.send_move(20, POS_UNREACHABLE, ORI_HOME)
        self.send_gripper(21, GRIPPER_OPEN)

    def estop(self):
        self.send_move(30, POS_BACK, ORI_P45)
        self.send_gripper(31, GRIPPER_OPEN)
        self.send_move(32, POS_SIDE, ORI_M45)

        self.wait(0.5)
        self.send_estop()

    def home(self):
        self.send_init_pose()

    def run(self, scenario):
        self.wait(1.5)   # dar tiempo al descubrimiento de DDS

        getattr(self, scenario)()

        self.wait(2.0)   # asegurar que los últimos mensajes salen


def main(args=None):
    scenario = sys.argv[1] if len(sys.argv) > 1 else "happy"

    if scenario not in SCENARIOS:
        print(f"Escenario desconocido '{scenario}'. Opciones: {', '.join(SCENARIOS)}")
        return 1

    rclpy.init(args=args)              #? Inicializa ROS
    node = WaypointSender()            #? Instancia de un nodo de clase WaypointSender

    try:
        node.run(scenario)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()           #? Cierra el nodo al terminar la ejecución

    return 0


if __name__ == "__main__":
    sys.exit(main())
