#!/usr/bin/env python3
"""Hardware in the loop: el cubo simulado sigue al cubo real que ve el sensor.

Convierte cada lectura del sensor real (proximity_sensor_node, URM37 + Arduino)
en la posicion del cubo en la cinta con la misma formula que las rutinas:

    y_cubo = sensor_y + range + medio_cubo

estima su velocidad con un ajuste lineal de las ultimas lecturas y manda el
estado al puente MuJoCo, que mueve el cubo simulado con la cinta (sim_cubo.py,
modo seguimiento). Asi interceptar_cubo / recoger_cubo reaccionan a la lectura
real y el brazo simulado interactua con un cubo que esta donde esta el real.

  escucha  /proximity            sensor_msgs/Range           sensor real
  publica  /control/cubo_seguir  std_msgs/Float64MultiArray  [x, y, z, vx, vy, vz] mundo

Sin eco (range fuera de [min_range, max_range]) no publica: el puente retiene el
cubo donde estaba.
"""

from collections import deque

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Range
from std_msgs.msg import Float64MultiArray


class CuboHil(Node):
    def __init__(self):
        super().__init__('cubo_hil')
        p = {
            'proximity_topic': '/proximity',
            'sensor_y': -0.90,     # m, posicion del sensor en la via (mismo valor que la rutina)
            'medio_cubo': 0.02,    # m, del frente al centro del cubo
            'cubo_x': -0.55,       # m, via de la cinta (marco mundo)
            'cubo_z': 0.12,        # m, altura de la cinta
            'ventana': 0.8,        # s de lecturas para estimar la velocidad
            'v_max': 0.5,          # m/s, estimaciones mayores se recortan
        }
        for k, v in p.items():
            setattr(self, k, self.declare_parameter(k, v).value)

        self.muestras = deque()
        self.pub = self.create_publisher(Float64MultiArray, '/control/cubo_seguir', 10)
        self.create_subscription(Range, self.proximity_topic, self.proximity_cb, 10)
        self.get_logger().info(
            f'HIL: {self.proximity_topic} -> /control/cubo_seguir, '
            f'y_cubo = {self.sensor_y:.3f} + range + {self.medio_cubo:.3f}.')

    def proximity_cb(self, msg):
        if not (msg.min_range < msg.range < msg.max_range):
            return  # sin eco
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        y = self.sensor_y + msg.range + self.medio_cubo
        self.muestras.append((t, y))
        while self.muestras and t - self.muestras[0][0] > self.ventana:
            self.muestras.popleft()

        v = 0.0
        if len(self.muestras) >= 3:
            ts, ys = np.array(self.muestras).T
            if ts[-1] - ts[0] > 1e-3:
                v = float(np.clip(np.polyfit(ts - ts[-1], ys, 1)[0], -self.v_max, self.v_max))

        self.pub.publish(Float64MultiArray(
            data=[float(self.cubo_x), float(y), float(self.cubo_z), 0.0, v, 0.0]))


def main(args=None):
    rclpy.init(args=args)
    node = CuboHil()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
