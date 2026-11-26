# Un comando para el ejecutor de waypoints (grupo2_pkg/waypoint_manager.py).
#
# Cada mensaje es UN comando: o mueve el brazo, o mueve el gripper, nunca los
# dos. Van separados a propósito: el ejecutor solo acciona el gripper después de
# confirmar la llegada, así que un comando mixto con una pose inalcanzable se
# comería el timeout sin llegar a tocar la pinza.

# Comandos admitidos (string para que se lea directo en `ros2 topic echo`)
string COMMAND_MOVE=move
string COMMAND_GRIPPER=gripper

# Identificador asignado por quien publica, para poder cancelarlo por id.
# 0 = sin identificador (se ejecuta igual, pero no se puede cancelar por id).
uint16 id

# COMMAND_MOVE o COMMAND_GRIPPER
string command

# Pose de la brida en el marco base_link. Solo se usa si command == COMMAND_MOVE.
# Misma convención que /control/move_p (ver src/agx_arm_ros/test/nero/test_move_p.yaml).
geometry_msgs/Pose pose

# Apertura total del gripper en metros. Solo se usa si command == COMMAND_GRIPPER.
float64 gripper_width
