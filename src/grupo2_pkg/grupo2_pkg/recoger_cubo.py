#!/usr/bin/env python3
"""P3: interceptar el cubo en la cinta, agarrarlo y levantarlo.

Misma estimacion que P2 (interceptar_cubo.py): el sensor de proximidad da la
posicion y la velocidad del cubo, y la bajada se ordena para que la pinza llegue
cuando el cubo esta entre los dedos (y_encuentro = centro de la pinza). Despues
se cierra la pinza, se confirma el agarre por la apertura medida (se detiene en
el ancho del cubo) y se levanta.

  publica  /commands/waypoints       grupo2_interfaces/Waypoint  pose de link7 o apertura
  publica  /commands/cancel_waypoint std_msgs/UInt16             cancelar un comando por id
  escucha  /proximity                sensor_msgs/Range           distancia al cubo
  escucha  /feedback/tcp_pose        geometry_msgs/PoseStamped   pose alcanzada
  escucha  /feedback/joint_states    sensor_msgs/JointState      apertura real

El cierre sobre el cubo nunca llega a la apertura pedida (el cubo lo frena), asi
que al confirmar el agarre se cancela por id: la pinza sigue apretando y
waypoint_manager pasa al levantamiento.

Fases: ESPERA -> SOBREVOLAR -> MEDIR -> BAJAR -> CERRAR -> LEVANTAR -> FIN
"""

import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException

from grupo2_pkg.interceptar_cubo import FIN, InterceptarCubo

CERRAR = 'cerrar'
LEVANTAR = 'levantar'


class RecogerCubo(InterceptarCubo):
    def __init__(self):
        super().__init__(
            'recoger_cubo',
            y_encuentro=0.0,    # m, centro de la pinza
            t_bajada=0.65,      # s, bajada medida en simulacion
            margen=0.1,         # s
            y_max_medida=-0.06,  # m, desde aqui el dedo delantero tapa el cubo
        )
        p = {
            'apertura_cerrada': 0.0,      # m, orden de cierre
            'ancho_min': 0.03,            # m, apertura minima con el cubo entre los dedos
            'ancho_max': 0.05,            # m
            'estable': 0.005,             # m, variacion maxima de apertura en t_estable
            't_estable': 0.5,             # s
            'altura_levantar': 0.10,      # m
        }
        for k, v in p.items():
            setattr(self, k, self.declare_parameter(k, v).value)
        self.levantado = self.objetivo + np.array([0.0, 0.0, self.altura_levantar])
        self.historial_apertura = []

    def al_bajar(self):
        self.id_cierre = self.enviar_gripper(self.apertura_cerrada)
        self.historial_apertura = []
        self.get_logger().info('Cerrando la pinza...')
        self.cambiar_fase(CERRAR)

    def agarre_estable(self):
        """True si la apertura lleva t_estable segundos quieta; devuelve tambien la apertura."""
        ahora = time.monotonic()
        self.historial_apertura.append((ahora, self.apertura))
        self.historial_apertura = [(t, a) for t, a in self.historial_apertura
                                   if ahora - t <= self.t_estable]
        if ahora - self.t0 < self.t_estable:
            return False
        aperturas = [a for _, a in self.historial_apertura]
        return max(aperturas) - min(aperturas) < self.estable

    def rutina_cb(self):
        if self.fase == CERRAR:
            if self.apertura is None:
                return
            if self.apertura < self.ancho_min:
                self.terminar(
                    f'NO AGARRADO: la pinza cerro hasta {100 * self.apertura:.1f} cm sin cubo.',
                    aviso=True)
            elif self.agarre_estable() and self.apertura <= self.ancho_max:
                self.get_logger().info(
                    f'Cubo agarrado (apertura {100 * self.apertura:.1f} cm). Levantando...')
                self.cancelar(self.id_cierre)
                self.enviar_pose(self.levantado)
                self.cambiar_fase(LEVANTAR)
            elif self.expirado():
                self.terminar('La pinza no termino de cerrar.', aviso=True)

        elif self.fase == LEVANTAR:
            if self.apertura is not None and self.apertura < self.ancho_min:
                self.terminar('NO LEVANTADO: el cubo se solto al subir.', aviso=True)
            elif self.cerca_de(self.levantado):
                self.terminar(
                    f'LEVANTADO: cubo sujeto {100 * self.altura_levantar:.0f} cm arriba '
                    f'(apertura {100 * self.apertura:.1f} cm).')
            elif self.expirado():
                self.terminar('No se alcanzo la altura de levantamiento.', aviso=True)

        elif self.fase != FIN:
            super().rutina_cb()


def main(args=None):
    rclpy.init(args=args)
    node = RecogerCubo()
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
