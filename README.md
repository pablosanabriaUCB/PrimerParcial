# PrimerParcial

EXAMEN-1ER-PARCIAL-ROB-TICA-
Sanabria Pablo, Ivan Campos

PARTE-1
colcon build
source install/setup.bash
ros2 topic pub /target_position geometry_msgs/msg/Point "{x: 1.2, y: 0.5, z: 4.0}" -1
ros2 run visual_pubsub inverse_kinematics


PARTE-2
Sensor1:
colcon build
source install/setup.bash
ros2 run mypkg nodo1

Sensor2:
colcon build
source install/setup.bash
ros2 run mypkg nodo2

Sensor3:
colcon build
source install/setup.bash
ros2 run mypkg nodo3

Nodo4:
colcon build
source install/setup.bash
ros2 run mypkg nodo4

Nodo5:
colcon build
source install/setup.bash
ros2 run mypkg nodo5

Para graficar la comunicacion de todos los nodos con sus correspondientes topicos: rqt_graph



