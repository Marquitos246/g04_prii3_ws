# Proyecto RII 3 — Grupo 04

Workspace de ROS 2 para las prácticas de Proyecto RII 3.

- Sprint 1: dibujo del número 04 con turtlesim.
- Sprint 2: dibujo del 04, detección y rodeo de obstáculos con TurtleBot3 en Gazebo.

## Requisitos

Entorno utilizado para las pruebas:

- Ubuntu 22.04.
- ROS 2 Humble.
- Python 3 y colcon.
- turtlesim.
- Gazebo Classic y los paquetes de simulación de TurtleBot3.
- RViz2.
- Modelo TurtleBot3 Burger.

Los paquetes de Python utilizan rclpy, geometry_msgs, nav_msgs,
sensor_msgs y std_srvs.

## Descargar y compilar

Clonar el repositorio:

```bash
git clone https://github.com/Marquitos246/g04_prii3_ws.git
cd g04_prii3_ws
```

Cargar ROS 2 y compilar:

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Los ejemplos siguientes suponen que el workspace está en
`~/g04_prii3_ws`. Si se clonó en otra ubicación, adaptar las rutas.

En cada terminal nueva que vaya a ejecutar nuestros paquetes:

```bash
source /opt/ros/humble/setup.bash
source ~/g04_prii3_ws/install/setup.bash
```

## Sprint 1 — turtlesim

### Ejecutar

```bash
ros2 launch g04_prii3_turtlesim dibujo_4.launch.py
```

Este launch inicia turtlesim y el nodo `dibujo_4`.
La tortuga dibuja automáticamente el número 04.

### Servicios

Detener el dibujo:

```bash
ros2 service call /dibujo_4/pausar std_srvs/srv/SetBool "{data: true}"
```

Reanudar el dibujo:

```bash
ros2 service call /dibujo_4/pausar std_srvs/srv/SetBool "{data: false}"
```

Reiniciar el dibujo:

```bash
ros2 service call /dibujo_4/reiniciar std_srvs/srv/Trigger "{}"
```

### Funcionamiento

El nodo publica mensajes geometry_msgs/msg/Twist en
`/turtle1/cmd_vel` y recibe la posición y orientación de la tortuga
mediante `/turtle1/pose`.

## Sprint 2 — TurtleBot3 en Gazebo

### Estado del desarrollo

Probado en simulación:

- Dibujo autónomo del 04.
- Servicios para detener, reanudar y reiniciar.
- Visualización del recorrido en RViz.
- Parada ante un obstáculo frontal y continuación automática al retirarlo.
- Rodeo de un obstáculo aislado y continuación del dibujo.

El paquete `g04_prii3_move_jetbot` contiene la estructura inicial.
La implementación y las pruebas con el robot real están pendientes.

La parte de cámara y la documentación de sus resultados quedan
pendientes de incorporar a este repositorio.

### Terminal 1 — iniciar Gazebo

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

Esperar a que aparezca el robot y comprobar que la simulación está en Play.

### Terminal 2 — ejecutar un programa

Preparar el entorno:

```bash
source /opt/ros/humble/setup.bash
source ~/g04_prii3_ws/install/setup.bash
```

Ejecutar solo uno de los tres programas siguientes.
Antes de cambiar de programa, cerrar el anterior con Ctrl+C.
Tampoco debe ejecutarse teleoperación al mismo tiempo.

#### Dibujar el 04

```bash
ros2 run g04_prii3_move_turtlebot draw_number --ros-args -p use_sim_time:=true
```

El robot sigue nueve puntos utilizando la odometría.
Cada número mide aproximadamente 0,6 m de ancho y 1,2 m de alto.

Primero se orienta hacia el siguiente punto y después avanza,
reduciendo la velocidad al acercarse. Considera alcanzado el
punto cuando queda a menos de 3 cm.

Al finalizar muestra:

```text
Punto 9/9 alcanzado.
Dibujo terminado. Recorrido disponible en RViz.
```

#### Detenerse ante obstáculos

```bash
ros2 run g04_prii3_move_turtlebot collision_avoidance --ros-args -r __node:=collision_avoidance -p use_sim_time:=true
```

El robot dibuja el 04 y comprueba las medidas del láser en un
sector frontal de ±30 grados.

- Se detiene si detecta un obstáculo a 0,40 m o menos del láser.
- Continúa automáticamente cuando la distancia frontal supera 0,50 m.
- Conserva el punto del dibujo que tenía pendiente.

Los dos umbrales evitan alternar continuamente entre parar y avanzar.

Para comprobarlo en Gazebo:

1. Colocar una caja delante del robot sin que se toquen.
2. Comprobar que se detiene y aparece el mensaje `OBSTACULO`.
3. Retirar la caja.
4. Comprobar que continúa automáticamente y termina el dibujo.

#### Rodear obstáculos

```bash
ros2 run g04_prii3_move_turtlebot obstacle_avoidance --ros-args -r __node:=obstacle_avoidance -p use_sim_time:=true
```

El robot intenta pasar por la izquierda del obstáculo y mantenerlo
a su derecha. Cuando ha progresado y detecta un camino despejado
hacia el punto pendiente, vuelve al controlador de dibujo.

Para comprobarlo:

1. Colocar un obstáculo aislado en la trayectoria, con espacio alrededor.
2. Evitar que el obstáculo cubra un punto objetivo del dibujo.
3. Mantener el obstáculo colocado durante la prueba.
4. Comprobar que el robot lo rodea y continúa hacia el punto pendiente.

Durante el rodeo, la trayectoria se desvía de la forma original del 04.

Esta implementación se ha probado con un obstáculo aislado en Gazebo;
no garantiza resolver entornos arbitrarios. Puede girar sobre el sitio
si está demasiado cerca de un obstáculo. Si recorre más de 12 m
durante un rodeo sin resolverlo, se detiene y solicita revisar la prueba.

### Terminal 3 — visualizar el recorrido

Desde la raíz del workspace:

```bash
source /opt/ros/humble/setup.bash
cd ~/g04_prii3_ws
rviz2 -d recorrido_04.rviz --ros-args -p use_sim_time:=true
```

Configuración utilizada:

- Fixed Frame: `odom`.
- Visualización: `Path`.
- Topic: `/recorrido_04`.
- Vista superior: `TopDownOrtho`.

La línea representa el recorrido registrado mediante odometría.
Incluye el desplazamiento entre el 0 y el 4.

El nodo sigue publicando la línea después de terminar el dibujo,
mientras permanece abierto.

### Terminal 4 — servicios de control

Preparar el entorno:

```bash
source /opt/ros/humble/setup.bash
source ~/g04_prii3_ws/install/setup.bash
```

Los tres programas del Sprint 2 ofrecen los mismos servicios.

Detener:

```bash
ros2 service call /detener_dibujo std_srvs/srv/Empty "{}"
```

Reanudar:

```bash
ros2 service call /reanudar_dibujo std_srvs/srv/Empty "{}"
```

Reiniciar:

```bash
ros2 service call /reiniciar_dibujo std_srvs/srv/Empty "{}"
```

Detener mantiene el punto pendiente. Reanudar continúa el dibujo;
si ya había terminado, se debe utilizar reiniciar.

Reiniciar espera a que el robot se detenga, borra la traza anterior
y comienza otro 04 desde su posición y orientación actuales.
No devuelve el robot al origen de Gazebo.

En `collision_avoidance`, la reanudación manual no anula la parada
por obstáculo.

### Topics utilizados

| Topic | Tipo | Uso |
|---|---|---|
| `/cmd_vel` | geometry_msgs/msg/Twist | Enviar velocidades al robot |
| `/odom` | nav_msgs/msg/Odometry | Recibir posición y orientación |
| `/scan` | sensor_msgs/msg/LaserScan | Recibir medidas del láser |
| `/recorrido_04` | nav_msgs/msg/Path | Mostrar el recorrido en RViz |

`collision_avoidance` y `obstacle_avoidance` reutilizan la clase
DrawNumber de `draw_number.py`, que contiene el dibujo, la odometría,
la traza y los servicios.

### Repetir una prueba

Cada nueva ejecución toma la posición y orientación actuales como
referencia. Por ello, el dibujo puede aparecer desplazado o girado.

Para repetir desde la posición inicial de la simulación:

1. Cerrar el programa de movimiento con Ctrl+C.
2. Cerrar Gazebo desde la terminal 1 con Ctrl+C.
3. Volver a iniciar Gazebo.
4. Ejecutar el programa elegido.

### Cerrar

Cerrar primero el programa de movimiento con Ctrl+C.
El programa intenta publicar velocidad cero antes de finalizar.

Después se pueden cerrar RViz y Gazebo.

## Grupo 04