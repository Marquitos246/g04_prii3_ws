import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Empty


def normalizar(angulo):
    """Devuelve un angulo entre -pi y pi."""
    return math.atan2(math.sin(angulo), math.cos(angulo))


class DrawNumber(Node):
    def __init__(self):
        super().__init__('draw_number')

        self.publisher = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.path_publisher = self.create_publisher(
            Path, '/recorrido_04', 10
        )

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.recibir_odom,
            qos_profile_sensor_data
        )

        self.pose = None
        self.origen = None
        self.ultima_odom = None
        self.ultimo_log = 0.0

        self.indice = 0
        self.girando = True
        self.pausado = False
        self.reinicio_pendiente = False

        self.velocidad_lineal = 0.15
        self.velocidad_angular = 0.20

        self.recorrido = Path()
        self.ultimo_punto_traza = None

        # Coordenadas relativas al inicio del dibujo.
        self.puntos = [
            (-0.6, 0.0),     # 0: lado superior
            (-0.6, -1.2),    # 0: lado izquierdo
            (0.0, -1.2),     # 0: lado inferior
            (0.0, 0.0),      # 0: lado derecho
            (0.4, 0.0),      # Union entre los numeros
            (0.4, -0.6),     # 4: tramo izquierdo
            (1.0, -0.6),     # 4: tramo central
            (1.0, 0.0),      # 4: subir por el lado derecho
            (1.0, -1.2),     # 4: lado derecho completo
        ]

        self.servicio_detener = self.create_service(
            Empty, '/detener_dibujo', self.detener_dibujo
        )
        self.servicio_reanudar = self.create_service(
            Empty, '/reanudar_dibujo', self.reanudar_dibujo
        )
        self.servicio_reiniciar = self.create_service(
            Empty, '/reiniciar_dibujo', self.reiniciar_dibujo
        )

        self.timer = self.create_timer(0.05, self.controlar)
        self.path_timer = self.create_timer(
            0.2, self.publicar_recorrido
        )

        self.get_logger().info('Esperando odometria en /odom...')
        self.get_logger().info(
            'Servicios disponibles: /detener_dibujo, '
            '/reanudar_dibujo y /reiniciar_dibujo.'
        )

    def recibir_odom(self, mensaje):
        posicion = mensaje.pose.pose.position
        q = mensaje.pose.pose.orientation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        self.pose = (posicion.x, posicion.y, yaw)
        self.ultima_odom = time.monotonic()

        # El reinicio utiliza una lectura nueva de odometria.
        if self.reinicio_pendiente:
            # Esperar a que el robot se haya detenido.
            movimiento = mensaje.twist.twist
            if (
                abs(movimiento.linear.x) > 0.01
                or abs(movimiento.angular.z) > 0.02
            ):
                self.detener()
                return

            self.origen = self.pose
            self.indice = 0
            self.girando = True
            self.ultimo_log = 0.0
            self.recorrido = Path()
            self.ultimo_punto_traza = None
            self.reinicio_pendiente = False

            self.get_logger().info(
                'Dibujo reiniciado desde la posicion '
                'y orientacion actuales.'
            )

        if self.origen is None:
            self.origen = self.pose
            self.get_logger().info('Comenzando el dibujo del 04.')

        self.recorrido.header.frame_id = mensaje.header.frame_id
        self.recorrido.header.stamp = mensaje.header.stamp

        # Guardar un punto cada centimetro de desplazamiento.
        guardar = self.ultimo_punto_traza is None

        if not guardar:
            anterior_x, anterior_y = self.ultimo_punto_traza
            desplazamiento = math.hypot(
                posicion.x - anterior_x,
                posicion.y - anterior_y
            )
            guardar = desplazamiento >= 0.01

        if guardar:
            punto = PoseStamped()
            punto.header.frame_id = mensaje.header.frame_id
            punto.header.stamp = mensaje.header.stamp
            punto.pose.position.x = posicion.x
            punto.pose.position.y = posicion.y
            punto.pose.position.z = 0.02
            punto.pose.orientation = q

            self.recorrido.poses.append(punto)
            self.ultimo_punto_traza = (posicion.x, posicion.y)

    def publicar_recorrido(self):
        if self.recorrido.header.frame_id:
            self.path_publisher.publish(self.recorrido)

    def detener(self):
        """Publica velocidad cero."""
        self.publisher.publish(Twist())

    def detener_dibujo(self, request, response):
        self.pausado = True
        self.detener()
        self.get_logger().info(
            'Dibujo pausado. Usa /reanudar_dibujo para continuar.'
        )
        return response

    def reanudar_dibujo(self, request, response):
        if (
            self.indice >= len(self.puntos)
            and not self.reinicio_pendiente
        ):
            self.get_logger().info(
                'El dibujo ya termino. Usa /reiniciar_dibujo '
                'para comenzar otro.'
            )
            return response

        self.pausado = False
        self.girando = True
        self.get_logger().info(
            'Dibujo reanudado. Continuara cuando haya odometria.'
        )
        return response

    def reiniciar_dibujo(self, request, response):
        self.detener()
        self.reinicio_pendiente = True
        self.pausado = False

        self.get_logger().info(
            'Reinicio solicitado: esperando que el robot '
            'se detenga y llegue odometria nueva.'
        )
        return response

    def controlar(self):
        if self.pausado or self.reinicio_pendiente:
            self.detener()
            return

        if self.pose is None:
            self.detener()
            return

        ahora = time.monotonic()

        if ahora - self.ultima_odom > 0.5:
            self.detener()
            return

        if self.indice >= len(self.puntos):
            self.detener()
            return

        x, y, yaw = self.pose
        x0, y0, yaw0 = self.origen
        px, py = self.puntos[self.indice]

        objetivo_x = (
            x0 + px * math.cos(yaw0) - py * math.sin(yaw0)
        )
        objetivo_y = (
            y0 + px * math.sin(yaw0) + py * math.cos(yaw0)
        )

        dx = objetivo_x - x
        dy = objetivo_y - y
        distancia = math.hypot(dx, dy)

        if distancia < 0.03:
            self.detener()
            self.indice += 1
            self.girando = True

            self.get_logger().info(
                f'Punto {self.indice}/{len(self.puntos)} alcanzado.'
            )

            if self.indice == len(self.puntos):
                self.get_logger().info(
                    'Dibujo terminado. Recorrido disponible en RViz.'
                )
            return

        angulo_objetivo = math.atan2(dy, dx)
        error_angulo = normalizar(angulo_objetivo - yaw)

        if abs(error_angulo) > 0.35:
            self.girando = True
        elif abs(error_angulo) < 0.06:
            self.girando = False

        velocidad = Twist()
        velocidad.angular.z = max(
            -self.velocidad_angular,
            min(self.velocidad_angular, 0.6 * error_angulo)
        )

        if not self.girando:
            velocidad.linear.x = min(
                self.velocidad_lineal, 0.6 * distancia
            )

        self.publisher.publish(velocidad)

        if ahora - self.ultimo_log >= 1.0:
            self.ultimo_log = ahora
            estado = 'GIRO' if self.girando else 'AVANCE'

            self.get_logger().info(
                f'{estado} | punto={self.indice + 1} '
                f'| distancia={distancia:.2f} m '
                f'| orientacion={math.degrees(yaw):.1f} grados '
                f'| error={math.degrees(error_angulo):.1f} grados '
                f'| orden_giro={velocidad.angular.z:.2f}'
            )


def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )

    nodo = DrawNumber()

    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            nodo.detener()
            time.sleep(0.1)

        nodo.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()