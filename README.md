# Sprint 1 - Proyecto RII 3 - Grupo 04

Proyecto realizado para la asignatura Proyecto RII 3.

El programa utiliza ROS2 Humble y turtlesim para dibujar automáticamente
el número del grupo 04.

## Requisitos

- Ubuntu 22.04
- ROS2 Humble
- turtlesim
- Python 3
- colcon

## Descargar el proyecto

Clonar el repositorio:

```bash
git clone https://github.com/Marquitos246/g04_prii3_ws
```

Entrar en el workspace:

```bash
cd g04_prii3_ws
```

## Compilar

```bash
colcon build
```

Cargar el workspace:

```bash
source install/setup.bash
```

## Ejecutar

El proyecto se ejecuta desde un único fichero launch:

```bash
ros2 launch g04_prii3_turtlesim dibujo_4.launch.py
```

Al ejecutarlo:

- Se inicia turtlesim.
- Se inicia el nodo `dibujo_4`.
- La tortuga dibuja automáticamente el número 04.

## Servicios

### Detener el dibujo

```bash
ros2 service call /dibujo_4/pausar std_srvs/srv/SetBool "{data: true}"
```

### Reanudar el dibujo

```bash
ros2 service call /dibujo_4/pausar std_srvs/srv/SetBool "{data: false}"
```

### Reiniciar el dibujo

```bash
ros2 service call /dibujo_4/reiniciar std_srvs/srv/Trigger "{}"
```

## Funcionamiento

El nodo `dibujo_4` publica mensajes de tipo `Twist` en:

```text
/turtle1/cmd_vel
```

para controlar el movimiento de la tortuga.

También se suscribe a:

```text
/turtle1/pose
```

para conocer su posición y orientación.

## Grupo 04
