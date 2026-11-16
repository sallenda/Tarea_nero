# Comandos para grabar el video de la entrega

Antes de empezar, en cada terminal:

```bash
cd ~/Tarea_nero
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Para compilar, una vez, después de cambiar de rama o hacer `git pull`:

```bash
colcon build --packages-select agx_arm_description nero_ctrl arduino_nodes
source install/setup.bash
```

> Las Partes 1, 2 y 3 están en `feature/parte2-cinta-intercepcion` (o en `develop`/`main` cuando se haga el merge). La Parte 4 (Arduino) está en `main`. Para grabar todo desde una sola rama, primero hay que hacer el merge.

---

## Parte 1: el robot se mueve usando ROS 2

**Terminal 1**: simulador, RViz, main_node, can_module, ik_node y la demo:

```bash
ros2 launch nero_ctrl nero_sim_ik.launch.py demo:=true
```

**Terminal 2**: para explicar la comunicación (mostrar mientras el brazo se mueve):

```bash
ros2 node list
ros2 topic list
rqt_graph                                   # grafo de publicadores y suscriptores
ros2 topic info /control/move_j -v          # quién publica y quién se suscribe
ros2 topic hz /feedback/joint_states        # aprox. 200 Hz: así se actualizan los datos del simulador
ros2 topic echo /input_obj_position         # los objetivos que manda la demo
```

Para mover el brazo "a mano" con un objetivo (sin `demo:=true`):

```bash
ros2 topic pub --once /input_obj_position std_msgs/msg/Float64MultiArray \
  "{data: [-0.40, 0.10, 0.20, 0.0, 1.5708, -1.5708]}"
```

Qué decir en el video: [`Parte1_arquitectura_ROS2.md`](Parte1_arquitectura_ROS2.md), secciones 2 a 4.

---

## Parte 2: cubo de 4 cm a velocidad constante e intercepción

**Terminal 1:**

```bash
ros2 launch nero_ctrl parte2.launch.py
```

**Terminal 2:**

```bash
ros2 topic echo /parte2/estado              # ESPERA → ALINEANDO → BAJANDO → INTERCEPTADO → SUBIENDO
ros2 topic hz /cube/pose                    # 100 Hz
ros2 topic echo /cube/twist --field twist.linear   # velocidad constante (0.04 m/s en -y)
```

Opcional, para mostrar que la velocidad es configurable:

```bash
ros2 topic pub --once /conveyor/speed std_msgs/msg/Float64 "{data: 0.05}"
```

Para mostrar primero solo el cubo moviéndose, sin el brazo: `ros2 launch nero_ctrl parte2.launch.py interceptor:=false`.

En el log hay que mostrar la línea `*** INTERCEPTADO #1 ... (error X mm) ***`. En RViz se ven la línea azul (trayectoria predicha) y la esfera verde (punto de intercepción).

---

## Parte 3: cerrar la pinza y recoger el cubo

**Terminal 1:**

```bash
ros2 launch nero_ctrl parte3.launch.py
```

**Terminal 2:**

```bash
ros2 topic echo /parte3/agarrado            # pasa a true con el cubo arriba
ros2 topic echo /cube/pose --field pose.position   # la z del cubo sube de 0.084 a unos 0.215 m
```

En el log hay que mostrar `Pinza cerrada: ancho medido ~37 mm` y `*** CUBO RECOGIDO #1 ... ***`.

---

## Parte 4: sensor de distancia con Arduino (rama `main`)

1. Cargar `Arduino_codes/ultrasonic_code/ultrasonic_code.ino` en el Arduino y conectarlo por USB.
2. Ver qué puerto tomó: `ls /dev/ttyACM* /dev/ttyUSB*`. Si da error de permisos: `sudo usermod -aG dialout $USER` y volver a iniciar sesión.

**Terminal 1**: simulación con IK más el nodo serial y el nodo del sensor:

```bash
ros2 launch nero_ctrl nero_sim_ik_depth.launch.py demo:=true serial_port:=/dev/ttyACM0
```

**Terminal 2:**

```bash
ros2 topic echo /depth_sensor/distance_cm   # mover la mano frente al sensor y ver cómo cambia
ros2 topic hz /ultrasonic/distance_cm
rqt_graph                                   # arduino_serial_node → depth_sensor_node dentro del grafo
```

---

## Consejos para grabar

- Antes de grabar, deja correr cada launch una vez: la primera carga de MuJoCo es la más lenta.
- En Parte 2 y Parte 3 el cubo parte 8 s después de abrir el simulador, y el ciclo se repite en cada vuelta (unos 19 s por vuelta a 0.04 m/s).
- Para el video, pon lado a lado la ventana de MuJoCo, la de RViz y una terminal con el log o con `ros2 topic echo`.
- Para cerrar todo: `Ctrl+C` en la terminal del launch.
