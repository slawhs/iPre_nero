# Tarea de Robótica — Robot Nero

## Pregunta 1: Comunicación y actualización de datos en ROS 2 (10 puntos)

### 1. Comunicación mediante publicadores y suscriptores

ROS 2 permite la comunicación entre distintos programas, llamados **nodos**, mediante un sistema de publicación y suscripción de mensajes.

Los elementos principales son:

- **Nodo:** programa que realiza una función específica dentro del sistema.
- **Tópico:** canal de comunicación por el que se intercambian mensajes.
- **Publicador (Publisher):** envía mensajes a un tópico.
- **Suscriptor (Subscriber):** recibe los mensajes publicados en un tópico.
- **Mensaje:** estructura que contiene los datos transmitidos.

En la simulación del robot Nero, esta comunicación permite enviar instrucciones de movimiento y recibir información sobre el estado del robot.

Por ejemplo, al ejecutar una instrucción de movimiento articular:

1. Se publica un mensaje de tipo `sensor_msgs/msg/JointState` en el tópico `/control/move_j`.
2. El nodo `mujoco_bridge` está suscrito a ese tópico, por lo que recibe la instrucción.
3. El mensaje contiene las posiciones objetivo de las articulaciones.
4. El nodo procesa la instrucción y actualiza los comandos de control utilizados por MuJoCo.

La comunicación puede representarse como:

```text
      Nodo de control
             |
             | Publica JointState
             v
       /control/move_j
             |
             | Recibe el mensaje
             v
        mujoco_bridge
             |
             v
       Simulación MuJoCo
```

De esta manera, los nodos pueden intercambiar información sin necesitar una conexión directa entre sus programas.

### 2. Actualización de los datos de la simulación

La actualización de los datos se realiza continuamente durante la ejecución de MuJoCo.

El proceso funciona de la siguiente manera:

**1. Recepción de instrucciones**

El nodo `mujoco_bridge` recibe los mensajes de control enviados mediante ROS 2.

Estos mensajes indican las posiciones objetivo de las articulaciones o del gripper.

**2. Actualización de la simulación**

MuJoCo ejecuta sucesivos pasos de simulación mediante la función:

`mujoco.mj_step(model, data)`

En cada paso se actualiza el estado físico del robot, incluyendo las posiciones y velocidades de sus articulaciones.

**3. Publicación de los datos actualizados**

El nodo `mujoco_bridge` obtiene las posiciones y velocidades actuales desde MuJoCo y las publica mediante el tópico:

`/feedback/joint_states`

Este tópico utiliza mensajes de tipo `sensor_msgs/msg/JointState`.

En el código del profesor, la frecuencia de publicación está configurada por defecto en **200 Hz**, es decir, se programa una publicación cada 0,005 segundos.

Además, se publica la posición y orientación del extremo del brazo mediante:

`/feedback/tcp_pose`

**4. Recepción y utilización de los datos**

El nodo `can_module` está suscrito a `/feedback/joint_states`, por lo que puede recibir la información actualizada de las articulaciones.

Esta información también se utiliza para actualizar la representación del robot en RViz, mediante `robot_state_publisher`.

Por lo tanto, existe un ciclo continuo de comunicación:

```text
       Instrucción de movimiento
                  |
                  v
           /control/move_j
                  |
                  v
            mujoco_bridge
                  |
                  v
         Actualización MuJoCo
                  |
                  v
       /feedback/joint_states
                  |
                  v
        Otros nodos de ROS 2
```

Es importante distinguir entre las **posiciones objetivo**, que indican dónde queremos mover el robot, y las **posiciones actuales**, que corresponden al estado calculado por la simulación.

Esta diferencia permite utilizar la retroalimentación para conocer cómo está respondiendo el robot.

### 3. Visualización de la simulación

MuJoCo se encarga de calcular el estado físico del robot.

Su ventana gráfica se actualiza aproximadamente a **60 Hz**, mientras que los datos de retroalimentación se publican a una frecuencia configurada de 200 Hz.

Por su parte, RViz permite visualizar el robot utilizando la información publicada mediante ROS 2.

Por lo tanto, la simulación física, la publicación de datos y la visualización gráfica son procesos relacionados, pero no necesariamente funcionan a la misma frecuencia.

### 4. Resumen para la explicación oral

ROS 2 permite comunicar los nodos del robot mediante publicadores, suscriptores y tópicos.

Por ejemplo, cuando enviamos un movimiento articular, publicamos un mensaje en `/control/move_j`. El nodo `mujoco_bridge` recibe esa instrucción y la utiliza para controlar el robot en MuJoCo.

La simulación actualiza continuamente las posiciones y velocidades articulares mediante pasos de simulación.

Después, `mujoco_bridge` publica los datos actualizados en `/feedback/joint_states`, con una frecuencia configurada de 200 Hz.

Otros nodos pueden suscribirse a ese tópico para conocer el estado del robot, mientras que RViz permite visualizarlo.

De esta manera, ROS 2 permite mantener la comunicación entre las instrucciones de control, la simulación y la información de retroalimentación.