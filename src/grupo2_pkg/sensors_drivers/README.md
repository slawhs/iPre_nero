# P4 — Sensor de proximidad

Real y simulado publican en `/proximity` (`sensor_msgs/Range`, metros, frame `proximity_link`,
ULTRASOUND, cono 0.26 rad, 0.04–5.0 m).

## Real: URM37 + Arduino

Cargar `Sensor_proximidad_arduino_r4.ino` en el Arduino y lanzar el puente serie:

```bash
ros2 run grupo2_pkg proximity_sensor_node --ros-args -p port:=/dev/ttyACM0 -p baud:=9600
```

## Simulado: MuJoCo

El puente de MuJoCo publica `/proximity` al cargar `scene_nero_cubo.xml`.

- Código: `src/agx_arm_description/scripts/sim_proximity.py`. Lanza rayos en un cono de 15°
  contra la simulación en vivo y publica el eco más cercano.
- Montaje: body `proximity_sensor` en `scene_nero_cubo.xml`, al inicio de la cinta, mirando en +Y.
  `y_cubo = -0.90 + range + 0.02`.
- Parámetros: `proximity.*` del nodo `mujoco_bridge`, listados en `sim_proximity.py`.

```bash
ros2 topic echo /proximity
```
