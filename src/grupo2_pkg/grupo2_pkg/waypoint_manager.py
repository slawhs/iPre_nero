#!/usr/bin/env python3

"""Ejecutor de waypoints: recibe comandos sueltos, los encola y los ejecuta en orden.

No hay cinemática ni planificación: el nodo solo traduce cada comando de la cola
a un setpoint en los tópicos del controlador y espera a que el feedback confirme
que se cumplió antes de pasar al siguiente.

Tópicos usados (los que levanta `ros2 launch nero_ctrl nero_can_ctrl.launch.py
mode:=sim rviz:=false`):

  publica  /control/move_p        geometry_msgs/PoseStamped  -> pose objetivo
  publica  /control/move_j        sensor_msgs/JointState     -> vuelta a la pose inicial
  publica  /control/joint_states  sensor_msgs/JointState     -> apertura gripper
  escucha  /feedback/tcp_pose     geometry_msgs/PoseStamped  -> pose alcanzada
  escucha  /feedback/joint_states sensor_msgs/JointState     -> articulaciones y apertura reales

Comandos propios:

  escucha  /commands/waypoints       grupo2_interfaces/Waypoint  -> encolar un comando
  escucha  /commands/cancel_waypoint std_msgs/UInt16             -> cancelar por id (0 = todo)
  escucha  /commands/estop           std_msgs/Empty              -> parada: vacia la cola y clava el brazo
"""

import math
import time
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from grupo2_interfaces.msg import Waypoint
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, UInt16


# ============================================================
# CONSTANTES DEL CONTROLADOR
# ============================================================

# Comandos admitidos en grupo2_interfaces/Waypoint
COMMAND_MOVE = Waypoint.COMMAND_MOVE
COMMAND_GRIPPER = Waypoint.COMMAND_GRIPPER

# Pose inicial que publica nero_ctrl/can_module.py al arrancar (joint1..7 + gripper).
# El nodo vuelve aquí si un comando falla.
ARM_JOINTS = [f"joint{i}" for i in range(1, 8)]
INIT_POSE = [0.0, -0.40, 0.0, 2.0, 0.0, 0.0, 0.0, 0.05]
INIT_EFFORT = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

# Etapas de la máquina de estados
WAIT_FEEDBACK = "wait_feedback"
IDLE = "idle"
SEND_WAYPOINT = "send_waypoint"
MOVING = "moving"
GRIPPER = "gripper"
GOING_HOME = "going_home"


class WaypointManager(Node):
    def __init__(self):
        super().__init__("waypoint_manager")

        #? --- Atributos ---

        self.waypoints_queue = deque()  # comandos pendientes, se ejecuta siempre el primero
        self.active_waypoint = None     # comando en ejecución (ya salió de la cola)

        self.stage = WAIT_FEEDBACK
        self.t0 = None
        self.stage_timeout = None

        self.tcp = None
        self.gripper_width = None
        self.joint_positions = None

        self.pos_setpoint = None
        self.gripper_setpoint = None

        self.setup_parameters()
        self.init_communications()

        self.get_logger().info("Ejecutor de waypoints listo. Esperando feedback del controlador...")

    def setup_parameters(self):
        # Medido en simulación: un move_p se asienta en 0-1,8 mm (el residuo es la
        # tolerancia de la IK del puente) en 2-4 s. El gripper es exponencial con
        # tau ~2 s: llega exacto, pero tarda ~6 s en entrar en 2 mm y ~12 s en clavarlo.
        self.declare_parameter("pos_tolerance", 0.004)      # m
        self.declare_parameter("gripper_tolerance", 0.002)  # m
        self.declare_parameter("joint_tolerance", 0.02)     # rad
        self.declare_parameter("move_timeout", 15.0)        # s
        self.declare_parameter("gripper_timeout", 10.0)     # s

        self.pos_tolerance = self.get_parameter("pos_tolerance").value
        self.gripper_tolerance = self.get_parameter("gripper_tolerance").value
        self.joint_tolerance = self.get_parameter("joint_tolerance").value
        self.move_timeout = self.get_parameter("move_timeout").value
        self.gripper_timeout = self.get_parameter("gripper_timeout").value

    def init_communications(self):
        # Publishers
        self.pub_pose = self.create_publisher(PoseStamped, "/control/move_p", 10)
        self.pub_joints = self.create_publisher(JointState, "/control/move_j", 10)
        self.pub_direct = self.create_publisher(JointState, "/control/joint_states", 10)

        # Subscribers
        self.create_subscription(PoseStamped, "/feedback/tcp_pose", self.tcp_cb, 10)
        self.create_subscription(JointState, "/feedback/joint_states", self.joint_states_cb, 10)
        self.create_subscription(Waypoint, "/commands/waypoints", self.waypoint_cb, 50)
        self.create_subscription(UInt16, "/commands/cancel_waypoint", self.cancel_waypoint_cb, 10)
        self.create_subscription(Empty, "/commands/estop", self.estop_cb, 10)

        # Timers
        self.timer = self.create_timer(0.1, self.routine_cb)


    # ======= Callbacks feedback =======
    def tcp_cb(self, msg):          #? Pose alcanzada por el centro del gripper 
        p = msg.pose.position
        self.tcp = (p.x, p.y, p.z)

    def joint_states_cb(self, msg):  #? Articulaciones y apertura reales
        if "gripper" in msg.name:
            i = msg.name.index("gripper")
            if i < len(msg.position):
                self.gripper_width = float(msg.position[i])

        positions = []
        for joint in ARM_JOINTS:
            if joint not in msg.name:
                return
            positions.append(float(msg.position[msg.name.index(joint)]))

        self.joint_positions = positions
    # ==================================


    # ======= Callbacks comandos =======
    def waypoint_cb(self, msg):      #? Encolar un comando nuevo
        command = msg.command.strip().lower()

        if command not in (COMMAND_MOVE, COMMAND_GRIPPER):
            self.get_logger().warn(f"Comando desconocido '{msg.command}': descartado. Usa '{COMMAND_MOVE}' o '{COMMAND_GRIPPER}'.")
            return

        # El id tiene que ser único mientras esté pendiente, o cancelar sería ambiguo
        if msg.id != 0 and self.find_waypoint(msg.id) is not None:
            self.get_logger().warn(f"El id {msg.id} ya está pendiente: comando descartado.")
            return

        waypoint = {"id": int(msg.id), "command": command}

        if command == COMMAND_MOVE:
            p = msg.pose.position
            o = msg.pose.orientation

            waypoint["pos"] = (p.x, p.y, p.z)
            waypoint["ori"] = (o.x, o.y, o.z, o.w)

            detail = f"pos={waypoint['pos']} ori={waypoint['ori']}"
        else:
            if msg.gripper_width < 0.0:
                self.get_logger().warn(f"Apertura negativa ({msg.gripper_width}): comando descartado.")
                return

            waypoint["gripper_width"] = float(msg.gripper_width)

            detail = f"{100 * waypoint['gripper_width']:.1f} cm"

        self.waypoints_queue.append(waypoint)

        self.get_logger().info(f"Encolado id={waypoint['id']} {command}: {detail} (cola: {len(self.waypoints_queue)})")

    def cancel_waypoint_cb(self, msg):  #? Cancelar por id (0 = cancelar todo)
        waypoint_id = int(msg.data)

        if waypoint_id == 0:
            pending = len(self.waypoints_queue) + (1 if self.active_waypoint is not None else 0)

            self.waypoints_queue.clear()

            self.get_logger().info(f"Cancelación total: {pending} comandos descartados.")

            if self.active_waypoint is not None:
                self.drop_active_waypoint()

            return

        # En la cola: se elimina y nadie se enteró
        for waypoint in list(self.waypoints_queue):
            if waypoint["id"] == waypoint_id:
                self.waypoints_queue.remove(waypoint)

                self.get_logger().info(f"Cancelado id={waypoint_id} (estaba en la cola).")

                return

        # En ejecución: solo se deja de esperar, el brazo sigue su trayectoria
        if self.active_waypoint is not None and self.active_waypoint["id"] == waypoint_id:
            self.drop_active_waypoint()

            return

        self.get_logger().warn(f"No hay ningún comando pendiente con id={waypoint_id}.")

    def estop_cb(self, msg):         #? Parada: vacía la cola y clava el brazo donde está
        pending = len(self.waypoints_queue) + (1 if self.active_waypoint is not None else 0)

        self.waypoints_queue.clear()
        self.active_waypoint = None

        held = self.hold_current_position()

        self.get_logger().error(f"E-STOP: {pending} comandos descartados." + ("" if held else " Sin feedback articular: NO se pudo frenar el brazo."))

        self.next_stage(IDLE)
    # ==================================


    def find_waypoint(self, waypoint_id):
        if self.active_waypoint is not None and self.active_waypoint["id"] == waypoint_id:
            return self.active_waypoint

        for waypoint in self.waypoints_queue:
            if waypoint["id"] == waypoint_id:
                return waypoint

        return None

    def send_pose_waypoint(self, waypoint):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = waypoint["pos"]

        (msg.pose.orientation.x,
         msg.pose.orientation.y,
         msg.pose.orientation.z,
         msg.pose.orientation.w) = waypoint["ori"]

        self.pub_pose.publish(msg)

        self.pos_setpoint = waypoint["pos"]

    def send_gripper_waypoint(self, width):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["gripper"]
        msg.position = [float(width)]

        self.pub_direct.publish(msg)

        self.gripper_setpoint = float(width)

    def hold_current_position(self):
        """Clava el brazo donde está.

        El controlador no tiene setpoint de velocidad: la única forma de frenar un
        movimiento en curso es mandarle como comando directo las articulaciones que
        acabamos de medir. Al ver nombres de joint1..7 en /control/joint_states el
        puente descarta la trayectoria en vuelo (_traj y _jgoal a None) y se queda
        en esa posición.

        Ojo: es una parada por software (pasa por DDS y por este nodo), no un
        enclavamiento de seguridad. En el brazo real existe además el servicio
        emergency_stop de agx_arm_ctrl, que frena con amortiguamiento.
        """
        if self.joint_positions is None:
            return False

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ARM_JOINTS)
        msg.position = list(self.joint_positions)

        # El gripper es otro actuador: si no se le manda su ancho actual, sigue su comando
        if self.gripper_width is not None:
            msg.name.append("gripper")
            msg.position.append(self.gripper_width)

        self.pub_direct.publish(msg)

        return True

    def send_init_pose(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ARM_JOINTS + ["gripper"]
        msg.position = list(INIT_POSE)
        msg.effort = list(INIT_EFFORT)

        self.pub_joints.publish(msg)


    def next_stage(self, stage, timeout=None):
        self.stage = stage
        self.t0 = time.monotonic()
        self.stage_timeout = timeout

    def timed_out(self):
        timer_running = self.t0 is not None and self.stage_timeout is not None
        elapsed = time.monotonic() - self.t0 if timer_running else 0.0
        time_exceeded = elapsed > self.stage_timeout

        return timer_running and time_exceeded

    def finish_active_waypoint(self):
        self.active_waypoint = None

        self.next_stage(IDLE)

    def drop_active_waypoint(self):
        waypoint_id = self.active_waypoint["id"]

        self.get_logger().warn(f"Cancelado id={waypoint_id} en ejecución: se deja de esperarlo, pero el brazo sigue la trayectoria que ya tiene el controlador.")

        self.finish_active_waypoint()

    def abort(self, reason):
        """Descarta toda la cola y vuelve a la pose inicial del launch."""
        self.get_logger().warn(reason)
        self.get_logger().warn(f"Rutina abortada: {len(self.waypoints_queue)} comandos descartados. Volviendo a la pose inicial.")

        self.waypoints_queue.clear()
        self.active_waypoint = None

        self.send_init_pose()

        self.next_stage(GOING_HOME, self.move_timeout)

    def routine_cb(self):
        # ----------------------------------------------------
        # Esperar a que el controlador esté escuchando
        # ----------------------------------------------------
        if self.stage == WAIT_FEEDBACK:
            controller_ready = (
                self.tcp is not None
                and self.pub_pose.get_subscription_count() > 0
                and self.pub_direct.get_subscription_count() > 0
            )

            if controller_ready:
                self.get_logger().info("Controlador detectado. Esperando comandos en /commands/waypoints.")
                self.next_stage(IDLE)

            return

        # ----------------------------------------------------
        # Cola vacía: esperando comandos
        # ----------------------------------------------------
        if self.stage == IDLE:
            if self.waypoints_queue:
                self.active_waypoint = self.waypoints_queue.popleft()

                self.next_stage(SEND_WAYPOINT)

            return

        # ----------------------------------------------------
        # Publicar el comando activo
        # ----------------------------------------------------
        if self.stage == SEND_WAYPOINT:
            waypoint = self.active_waypoint

            if waypoint["command"] == COMMAND_MOVE:
                self.send_pose_waypoint(waypoint)

                self.get_logger().info(f"Ejecutando id={waypoint['id']} move: pos={waypoint['pos']} ori={waypoint['ori']}")

                self.next_stage(MOVING, self.move_timeout)
            else:
                self.send_gripper_waypoint(waypoint["gripper_width"])

                self.get_logger().info(f"Ejecutando id={waypoint['id']} gripper: {100 * waypoint['gripper_width']:.1f} cm")

                self.next_stage(GRIPPER, self.gripper_timeout)

            return

        # ----------------------------------------------------
        # Esperar la llegada del brazo
        # ----------------------------------------------------
        if self.stage == MOVING:
            error = math.dist(self.tcp, self.pos_setpoint)

            if error < self.pos_tolerance:
                self.get_logger().info(f"Completado id={self.active_waypoint['id']} move (error {1000 * error:.1f} mm).")

                self.finish_active_waypoint()

            elif self.timed_out():
                self.abort(
                    f"id={self.active_waypoint['id']} move no alcanzado en {self.move_timeout:.0f} s "
                    f"(error {1000 * error:.1f} mm). Revisa si el controlador rechazó la IK."
                )

            return

        # ----------------------------------------------------
        # Esperar que el gripper llegue a la apertura pedida
        # ----------------------------------------------------
        if self.stage == GRIPPER:
            if self.gripper_width is None:
                return

            error = abs(self.gripper_width - self.gripper_setpoint)

            if error < self.gripper_tolerance:
                self.get_logger().info(f"Completado id={self.active_waypoint['id']} gripper ({100 * self.gripper_width:.1f} cm).")

                self.finish_active_waypoint()

            elif self.timed_out():
                self.abort(
                    f"id={self.active_waypoint['id']} gripper no alcanzó {100 * self.gripper_setpoint:.1f} cm "
                    f"en {self.gripper_timeout:.0f} s (está en {100 * self.gripper_width:.1f} cm)."
                )

            return

        # ----------------------------------------------------
        # Volviendo a la pose inicial tras un fallo
        # ----------------------------------------------------
        if self.stage == GOING_HOME:
            if self.joint_positions is None:
                return

            error = max(abs(q - q_init) for q, q_init in zip(self.joint_positions, INIT_POSE))

            if error < self.joint_tolerance:
                self.get_logger().info("Pose inicial recuperada. Esperando comandos en /commands/waypoints.")

                self.next_stage(IDLE)

            elif self.timed_out():
                self.get_logger().error(f"No se recuperó la pose inicial en {self.move_timeout:.0f} s (error {error:.3f} rad). Se aceptan comandos de nuevo igualmente.")

                self.next_stage(IDLE)

            return


def main(args=None):
    rclpy.init(args=args)              #? Inicializa ROS
    node = WaypointManager()           #? Instancia de un nodo de clase WaypointManager

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
