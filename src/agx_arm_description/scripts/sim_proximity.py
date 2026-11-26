"""
sim_proximity.py — Sensor de proximidad simulado (P4)
======================================================
Publica sensor_msgs/Range en /proximity, igual que grupo2_pkg/proximity_sensor_node con el URM37.

Lanza rayos (mj_ray) desde el site "proximity_site" a lo largo de su eje +Z, en un
cono de 15 grados, contra el estado en vivo de la simulacion. Publica el eco mas
cercano; sin eco publica max_range. Los planos (suelo) solo dan eco si el rayo
incide a menos de plane_echo_angle de la normal, como un ultrasonido real.

Parametros ROS (nodo mujoco_bridge, prefijo "proximity."):
  site            proximity_site
  topic           proximity
  frame_id        proximity_link
  rate            5.0    Hz
  min_range       0.04   m
  max_range       5.0    m
  fov             0.26   rad, apertura total del cono
  rings           2      anillos de rayos en el cono
  rays_per_ring   8
  noise_std       0.0    m, ruido gaussiano
  plane_echo_angle 0.52  rad
"""

import math

import numpy as np

import mujoco
from sensor_msgs.msg import Range


class SimProximitySensor:
    def __init__(self, node, model, data, lock):
        self.node = node
        self.model = model
        self.data = data
        self.lock = lock

        p = {
            'site': 'proximity_site',
            'topic': 'proximity',
            'frame_id': 'proximity_link',
            'rate': 5.0,
            'min_range': 0.04,
            'max_range': 5.0,
            'fov': 0.26,
            'rings': 2,
            'rays_per_ring': 8,
            'noise_std': 0.0,
            'plane_echo_angle': 0.52,
        }
        for k, v in p.items():
            p[k] = node.declare_parameter(f'proximity.{k}', v).value
        self.p = p

        self.site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, p['site'])
        if self.site_id < 0:
            node.get_logger().info(f'Sensor de proximidad: sin site "{p["site"]}", desactivado.')
            self.pub = None
            return

        # Excluye la carcasa del sensor (el cuerpo 0 es el mundo/suelo, no se excluye).
        body = int(model.site_bodyid[self.site_id])
        self.body_exclude = body if body != 0 else -1

        self.dirs_local = self._cone_directions(p['fov'] / 2.0, int(p['rings']),
                                                int(p['rays_per_ring']))
        self._geomid = np.zeros(1, dtype=np.int32)
        self._rng = np.random.default_rng()
        self._cos_plane = math.cos(float(p['plane_echo_angle']))

        self.pub = node.create_publisher(Range, p['topic'], 10)
        node.create_timer(1.0 / float(p['rate']), self._publish)
        node.get_logger().info(
            f'Sensor de proximidad: "{p["site"]}" -> "{p["topic"]}" a {p["rate"]} Hz.')

    @staticmethod
    def _cone_directions(half_angle, rings, per_ring):
        """Direcciones unitarias del cono en el marco del site (eje +Z + anillos)."""
        dirs = [np.array([0.0, 0.0, 1.0])]
        for r in range(1, rings + 1):
            theta = half_angle * r / rings
            for k in range(per_ring):
                phi = 2.0 * math.pi * k / per_ring
                dirs.append(np.array([
                    math.sin(theta) * math.cos(phi),
                    math.sin(theta) * math.sin(phi),
                    math.cos(theta),
                ]))
        return np.array(dirs)

    def measure(self):
        """Distancia (m) al eco mas cercano, o None."""
        with self.lock:
            origin = self.data.site_xpos[self.site_id].copy()
            R = self.data.site_xmat[self.site_id].reshape(3, 3).copy()
            best = None
            for d_local in self.dirs_local:
                d = R @ d_local
                dist = mujoco.mj_ray(self.model, self.data, origin, d, None, 1,
                                     self.body_exclude, self._geomid)
                if dist < 0.0 or (best is not None and dist >= best):
                    continue
                g = int(self._geomid[0])
                if self.model.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE:
                    normal = self.data.geom_xmat[g].reshape(3, 3)[:, 2]
                    if abs(float(normal @ d)) < self._cos_plane:
                        continue
                best = dist
        return best

    def _publish(self):
        p = self.p
        dist = self.measure()
        if dist is None or dist > p['max_range']:
            dist = p['max_range']
        else:
            if p['noise_std'] > 0.0:
                dist += float(self._rng.normal(0.0, p['noise_std']))
            dist = min(max(dist, p['min_range']), p['max_range'])

        msg = Range()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = p['frame_id']
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = float(p['fov'])
        msg.min_range = float(p['min_range'])
        msg.max_range = float(p['max_range'])
        msg.range = float(dist)
        self.pub.publish(msg)
