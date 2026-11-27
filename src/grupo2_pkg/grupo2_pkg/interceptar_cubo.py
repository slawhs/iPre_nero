#!/usr/bin/env python3
"""P2: intercepcion del cubo en la cinta con el sensor de proximidad.

El brazo espera sobre la cinta con la pinza abierta y los dedos alineados con
la via. El sensor (al inicio de la cinta, mirando en +Y) da la posicion del cubo;
con un ajuste lineal de las ultimas lecturas se estima su velocidad y se predice
cuando llega a y_encuentro, donde lo para el dedo delantero. La bajada se ordena
t_bajada + margen segundos antes, para que el dedo ya este en la via.

recoger_cubo.py (P3) reutiliza esta clase con otro punto de encuentro.

  publica  /commands/waypoints       grupo2_interfaces/Waypoint  pose de link7 o apertura
  publica  /commands/cancel_waypoint std_msgs/UInt16             cancelar un comando por id
  escucha  /proximity                sensor_msgs/Range           distancia al cubo
  escucha  /feedback/tcp_pose        geometry_msgs/PoseStamped   pose alcanzada
  escucha  /feedback/joint_states    sensor_msgs/JointState      apertura real

waypoint_manager ejecuta los comandos en orden, uno tras otro.

Fases: ESPERA -> SOBREVOLAR (abre la pinza a la vez) -> MEDIR -> BAJAR -> VERIFICAR -> FIN
"""

import math
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from grupo2_interfaces.msg import Waypoint
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import UInt16


# Orientacion de link7 (x, y, z, w): pinza 30 grados inclinada hacia fuera,
# apertura a lo largo de la via (+Y). Alcanzable por la IK del brazo.
Q_LINK7 = (0.348125, 0.624359, -0.597027, 0.364065)

# Pose de link7 con el centro de los dedos sobre el cubo en y = 0 (cubo a
# [-0.55, 0.0, 0.12] de base_link).
OBJETIVO_LINK7 = (-0.4889, -0.0235, 0.2409)

ESPERA = 'espera'
SOBREVOLAR = 'sobrevolar'
MEDIR = 'medir'
BAJAR = 'bajar'
VERIFICAR = 'verificar'
FIN = 'fin'


class InterceptarCubo(Node):
    def __init__(self, nombre='interceptar_cubo', **defaults):
        super().__init__(nombre)

        self.setup_parameters(defaults)
        self.init_communications()

        self.fase = ESPERA
        self.t0 = None

        self.tcp = None
        self.apertura = None
        self.muestras = deque()        # (t, y_cubo) de las lecturas validas
        self.estimacion = None         # (y, v, t) del ultimo ajuste
        self.t_orden_bajada = None

        self.get_logger().info(
            f'Encuentro en y = {self.y_int:.3f} m. Bajada de {self.altura:.2f} m, '
            f'T_bajada = {self.t_bajada:.2f} s. Waypoints a "{self.waypoint_topic}".')

    # ========================================================
    # PARAMETROS Y COMUNICACIONES
    # ========================================================

    def setup_parameters(self, defaults):
        p = {
            'waypoint_topic': '/commands/waypoints',
            'objetivo_link7': list(OBJETIVO_LINK7),   # m, pose de bajada
            'orientacion_link7': list(Q_LINK7),       # x, y, z, w
            'altura_sobrevuelo': 0.08,                # m sobre el objetivo
            'sensor_y': -0.90,                        # m, posicion del sensor en Y
            'medio_cubo': 0.02,                       # m, del frente al centro del cubo
            'y_max_medida': 1.0,                      # m, lecturas mas alla se ignoran
            't_bajada': 1.0,                          # s, lo que tarda la bajada
            'margen': 0.2,                            # s, adelanto extra
            'y_encuentro': -0.09,                     # m, donde el dedo delantero para el cubo
            'ventana': 1.0,                           # s de lecturas para el ajuste
            'min_muestras': 4,
            'v_min': 0.01,                            # m/s, por debajo el cubo esta quieto
            'atipico': 0.03,                          # m, residuo para descartar lectura
            'apertura_abierta': 0.10,                 # m
            'umbral_abierto': 0.085,                  # m
            'tol_pos': 0.015,                         # m
            'timeout': 15.0,                          # s
            'margen_pasado': 0.05,                    # m tras y_int para darlo por perdido
            't_quieto': 1.0,                          # s quieto para darlo por interceptado
            'tol_bloqueo': 0.03,                      # m alrededor de y_encuentro
        }
        p.update(defaults)
        for k, v in p.items():
            setattr(self, k, self.declare_parameter(k, v).value)

        self.objetivo = np.array(self.objetivo_link7, dtype=float)
        self.altura = float(self.altura_sobrevuelo)
        self.sobrevuelo = self.objetivo + np.array([0.0, 0.0, self.altura])
        self.y_int = float(self.y_encuentro)

    def init_communications(self):
        self.pub_waypoint = self.create_publisher(Waypoint, self.waypoint_topic, 10)
        self.pub_cancelar = self.create_publisher(UInt16, '/commands/cancel_waypoint', 10)
        self.ultimo_id = 0

        self.create_subscription(Range, '/proximity', self.proximity_cb, 10)
        self.create_subscription(PoseStamped, '/feedback/tcp_pose', self.tcp_cb, 10)
        self.create_subscription(JointState, '/feedback/joint_states', self.joint_states_cb, 10)

        self.timer = self.create_timer(0.05, self.rutina_cb)

    # ========================================================
    # CALLBACKS
    # ========================================================

    def tcp_cb(self, msg):
        p = msg.pose.position
        self.tcp = np.array([p.x, p.y, p.z])

    def joint_states_cb(self, msg):
        if 'gripper' in msg.name:
            i = msg.name.index('gripper')
            if i < len(msg.position):
                self.apertura = float(msg.position[i])

    def proximity_cb(self, msg):
        if not (msg.min_range < msg.range < msg.max_range):
            return  # sin eco
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        y = self.sensor_y + msg.range + self.medio_cubo
        if y > self.y_max_medida:
            return  # el dedo tapa el cubo: se sigue con la ultima estimacion
        self.muestras.append((t, y))
        while self.muestras and t - self.muestras[0][0] > self.ventana:
            self.muestras.popleft()
        self.estimacion = self.ajustar()

    # ========================================================
    # ESTIMACION DEL CUBO
    # ========================================================

    def ajustar(self):
        """Ajuste lineal y(t) sobre la ventana: devuelve (y, v, t) en la ultima muestra."""
        if len(self.muestras) < self.min_muestras:
            return None
        t, y = np.array(self.muestras).T
        v, y0 = np.polyfit(t - t[-1], y, 1)
        buenas = np.abs(y - (y0 + v * (t - t[-1]))) < self.atipico
        if buenas.sum() < self.min_muestras:
            return None
        if not buenas.all():
            v, y0 = np.polyfit(t[buenas] - t[-1], y[buenas], 1)
        return float(y0), float(v), float(t[-1])

    def tiempo_hasta_intercepcion(self):
        """Segundos hasta que el cubo llega a y_int, contados desde ahora."""
        y, v, t = self.estimacion
        edad = self.get_clock().now().nanoseconds * 1e-9 - t
        return (self.y_int - y) / v - edad

    def y_actual(self):
        y, v, t = self.estimacion
        return y + v * (self.get_clock().now().nanoseconds * 1e-9 - t)

    # ========================================================
    # ORDENES
    # ========================================================

    def nuevo_waypoint(self, comando):
        """Waypoint con un id nuevo (1..65535) para poder cancelarlo."""
        self.ultimo_id = self.ultimo_id % 65535 + 1
        msg = Waypoint()
        msg.id = self.ultimo_id
        msg.command = comando
        return msg

    def enviar_pose(self, pos):
        """Encola un movimiento de link7 a pos; devuelve su id."""
        msg = self.nuevo_waypoint(Waypoint.COMMAND_MOVE)
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, pos)
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = map(float, self.orientacion_link7)
        self.pub_waypoint.publish(msg)
        return msg.id

    def enviar_gripper(self, apertura):
        """Encola una apertura del gripper; devuelve su id."""
        msg = self.nuevo_waypoint(Waypoint.COMMAND_GRIPPER)
        msg.gripper_width = float(apertura)
        self.pub_waypoint.publish(msg)
        return msg.id

    def cancelar(self, waypoint_id):
        """Deja de esperar un comando: el siguiente de la cola se ejecuta ya."""
        self.pub_cancelar.publish(UInt16(data=waypoint_id))

    # ========================================================
    # RUTINA
    # ========================================================

    def cambiar_fase(self, fase):
        self.fase = fase
        self.t0 = time.monotonic()

    def expirado(self):
        return time.monotonic() - self.t0 > self.timeout

    def cerca_de(self, objetivo):
        return self.tcp is not None and math.dist(self.tcp, objetivo) < self.tol_pos

    def al_bajar(self):
        """Siguiente fase con la pinza abajo."""
        self.cambiar_fase(VERIFICAR)

    def terminar(self, mensaje, aviso=False):
        (self.get_logger().warn if aviso else self.get_logger().info)(mensaje)
        self.fase = FIN

    def rutina_cb(self):
        if self.fase == ESPERA:
            if (self.tcp is not None
                    and self.pub_waypoint.get_subscription_count() > 0
                    and self.pub_cancelar.get_subscription_count() > 0):
                self.enviar_gripper(self.apertura_abierta)
                self.enviar_pose(self.sobrevuelo)
                self.get_logger().info(
                    f'Abriendo pinza a {100 * self.apertura_abierta:.0f} cm y sobrevolando '
                    f'{np.round(self.sobrevuelo, 3)}.')
                self.cambiar_fase(SOBREVOLAR)

        elif self.fase == SOBREVOLAR:
            abierta = self.apertura is not None and self.apertura >= self.umbral_abierto
            if abierta and self.cerca_de(self.sobrevuelo):
                self.get_logger().info('En posicion. Midiendo el cubo con el sensor...')
                self.cambiar_fase(MEDIR)
            elif self.expirado():
                self.terminar(
                    'No se alcanzo el sobrevuelo con la pinza abierta (IK rechazada?).', aviso=True)

        elif self.fase == MEDIR:
            if self.estimacion is None:
                return
            y, v, _ = self.estimacion
            if v < self.v_min:
                return  # cubo quieto o sin medir aun
            restante = self.tiempo_hasta_intercepcion()
            if restante < 0.0:
                self.terminar(
                    f'Tarde: el cubo ya paso y_encuentro (y = {self.y_actual():.3f} m) antes de '
                    f'tener el brazo listo.', aviso=True)
                return
            if restante <= self.t_bajada + self.margen:
                self.enviar_pose(self.objetivo)
                self.t_orden_bajada = time.monotonic()
                self.get_logger().info(
                    f'Bajando: cubo en y = {self.y_actual():.3f} m a {v:.3f} m/s, '
                    f'llega en {restante:.2f} s.')
                self.cambiar_fase(BAJAR)

        elif self.fase == BAJAR:
            if self.cerca_de(self.objetivo):
                bajada = time.monotonic() - self.t_orden_bajada
                self.get_logger().info(
                    f'Pinza abajo. Bajada real: {bajada:.2f} s (t_bajada = {self.t_bajada:.2f} s).')
                self.al_bajar()
            elif self.expirado():
                self.terminar('No se alcanzo el punto de bajada.', aviso=True)

        elif self.fase == VERIFICAR:
            if time.monotonic() - self.t_orden_bajada > self.timeout:
                self.terminar('Sin conclusion: el cubo no se detuvo ni paso de largo.', aviso=True)
                return
            if self.estimacion is None:
                return
            y, v, _ = self.estimacion
            if y > self.y_int + self.margen_pasado:
                self.terminar(
                    f'NO INTERCEPTADO: el cubo siguio de largo (y = {y:.3f} m). '
                    f'Sube t_bajada.', aviso=True)
            elif abs(v) < self.v_min and abs(y - self.y_int) < self.tol_bloqueo:
                if time.monotonic() - self.t0 >= self.t_quieto:
                    self.terminar(f'INTERCEPTADO: cubo detenido en y = {y:.3f} m.')
            else:
                self.t0 = time.monotonic()  # sigue moviendose: reinicia el plazo quieto


def main(args=None):
    rclpy.init(args=args)
    node = InterceptarCubo()
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
