#!/usr/bin/env python3
#? ^ Le dice a bash "ejecuta esto con python3"  ^

"""Sonda de alcanzabilidad: prueba poses en /control/move_p y mira si el brazo llega.

Sirve para elegir waypoints antes de escribir una rutina. El puente de MuJoCo
rechaza en silencio (solo un warning en SU terminal) las poses sin solución de
cinemática inversa, así que desde fuera un waypoint inalcanzable se ve igual que
uno lento: esta herramienta distingue los dos midiendo /feedback/tcp_pose.

Parte de la pose actual del brazo y prueba desplazamientos y giros alrededor,
volviendo a la pose de partida entre candidatos.

Uso:
    ros2 run grupo2_pkg pose_probe              # ±5 cm y ±20°/±45° sobre Z
    ros2 run grupo2_pkg pose_probe 0.08         # con otro desplazamiento (m)
"""

import math
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


TOLERANCE = 0.004   # m, igual que el pos_tolerance del ejecutor
TIMEOUT = 8.0       # s por candidato


def quat_mul(a, b):
    """Producto de cuaterniones (x, y, z, w)."""
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b

    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def rotate_z(q, degrees):
    """Gira q sobre el eje Z de la base."""
    half = math.radians(degrees) / 2
    out = quat_mul((0.0, 0.0, math.sin(half), math.cos(half)), q)
    norm = math.sqrt(sum(c * c for c in out))

    return tuple(round(c / norm, 6) for c in out)


class PoseProbe(Node):
    def __init__(self, step):
        super().__init__("pose_probe")

        self.step = step
        self.tcp = None
        self.ori = None

        self.pub_pose = self.create_publisher(PoseStamped, "/control/move_p", 10)
        self.create_subscription(PoseStamped, "/feedback/tcp_pose", self.tcp_cb, 10)

    def tcp_cb(self, msg):
        p = msg.pose.position
        o = msg.pose.orientation

        self.tcp = (p.x, p.y, p.z)
        self.ori = (o.x, o.y, o.z, o.w)

    def wait_for_feedback(self):
        deadline = time.monotonic() + 10.0

        while self.tcp is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        return self.tcp is not None

    def send(self, pos, ori):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pos

        (msg.pose.orientation.x,
         msg.pose.orientation.y,
         msg.pose.orientation.z,
         msg.pose.orientation.w) = ori

        self.pub_pose.publish(msg)

    def try_pose(self, pos, ori):
        """Devuelve (alcanzada, error final en m)."""
        self.send(pos, ori)

        deadline = time.monotonic() + TIMEOUT
        error = float("inf")

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

            error = math.dist(self.tcp, pos)

            if error < TOLERANCE:
                return True, error

        return False, error

    def run(self):
        if not self.wait_for_feedback():
            self.get_logger().error("No llega /feedback/tcp_pose: ¿está el controlador arriba?")
            return

        base_pos = self.tcp
        base_ori = self.ori

        print(f"\nPose de partida: pos=({base_pos[0]:.4f}, {base_pos[1]:.4f}, {base_pos[2]:.4f}) "
              f"ori=({base_ori[0]:.4f}, {base_ori[1]:.4f}, {base_ori[2]:.4f}, {base_ori[3]:.4f})")
        print(f"Paso: {100 * self.step:.1f} cm   tolerancia: {1000 * TOLERANCE:.0f} mm\n")

        candidates = []

        for axis, name in enumerate("xyz"):
            for sign in (-1, 1):
                pos = list(base_pos)
                pos[axis] += sign * self.step
                candidates.append((tuple(pos), base_ori, f"{name}{sign * self.step:+.3f}"))

        for degrees in (20, -20, 45, -45):
            candidates.append((base_pos, rotate_z(base_ori, degrees), f"rotZ{degrees:+d}"))

        for pos, ori, label in candidates:
            reached, error = self.try_pose(pos, ori)

            verdict = "alcanzable" if reached else f"NO alcanzable (quedó a {1000 * error:.0f} mm)"

            print(f"  {label:12s} pos=({pos[0]:+.4f}, {pos[1]:+.4f}, {pos[2]:+.4f}) -> {verdict}")

            # Volver a la pose de partida antes del siguiente candidato
            self.try_pose(base_pos, base_ori)

        print("\nLas que salen alcanzables se pueden usar como waypoints en waypoint_sender.py.\n")


def main(args=None):
    step = float(sys.argv[1]) if len(sys.argv) > 1 else 0.05

    rclpy.init(args=args)              #? Inicializa ROS
    node = PoseProbe(step)             #? Instancia de un nodo de clase PoseProbe

    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()           #? Cierra el nodo al terminar la ejecución


if __name__ == "__main__":
    main()
