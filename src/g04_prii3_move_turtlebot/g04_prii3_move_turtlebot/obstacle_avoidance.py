import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan

from g04_prii3_move_turtlebot.draw_number import DrawNumber, normalizar


def limitar(valor, minimo, maximo):
    return max(minimo, min(maximo, valor))


class ObstacleAvoidance(DrawNumber):
    def __init__(self):
        super().__init__()

        self.lecturas = []
        self.ultima_scan = None

        self.rodeando = False
        self.distancia_al_empezar = None
        self.posicion_anterior = None
        self.metros_rodeo = 0.0
        self.bloqueado = False

        self.estado = None
        self.camino_libre_desde = None

        # Distancias medidas desde el laser, en metros.
        self.distancia_derecha = 0.35
        self.margen_camino = 0.24
        self.distancia_deteccion = 0.65

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.recibir_scan,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            'Rodeo activado: pasar por la izquierda '
            'manteniendo el obstaculo a la derecha.'
        )

    def informar(self, estado, mensaje):
        if estado != self.estado:
            self.estado = estado
            self.get_logger().info(mensaje)

    def recibir_scan(self, mensaje):
        lecturas = []

        for indice, distancia in enumerate(mensaje.ranges):
            angulo = normalizar(
                mensaje.angle_min
                + indice * mensaje.angle_increment
            )

            if distancia == math.inf:
                lecturas.append((angulo, distancia))
            elif (
                math.isfinite(distancia)
                and mensaje.range_min <= distancia <= mensaje.range_max
            ):
                lecturas.append((angulo, distancia))

        self.lecturas = lecturas
        self.ultima_scan = time.monotonic()

    def sector(self, centro, semiancho):
        valores = [
            distancia
            for angulo, distancia in self.lecturas
            if abs(normalizar(angulo - centro)) <= semiancho
        ]

        # Sin lecturas validas: tratar el sector como bloqueado.
        return min(valores) if valores else 0.0

    def camino_libre(self, direccion, longitud):
        # Comprobar que hay datos en la direccion consultada.
        tiene_datos = any(
            abs(normalizar(angulo - direccion)) < math.radians(5)
            for angulo, distancia in self.lecturas
        )

        if not tiene_datos:
            return False

        # Comprobar un pasillo con anchura, no solo un rayo.
        for angulo, distancia in self.lecturas:
            if not math.isfinite(distancia):
                continue

            relativo = normalizar(angulo - direccion)
            delante = distancia * math.cos(relativo)
            lateral = distancia * math.sin(relativo)

            if (
                0.0 < delante < longitud
                and abs(lateral) < self.margen_camino
            ):
                return False

        return True

    def objetivo_actual(self):
        x0, y0, yaw0 = self.origen
        px, py = self.puntos[self.indice]

        objetivo_x = (
            x0 + px * math.cos(yaw0) - py * math.sin(yaw0)
        )
        objetivo_y = (
            y0 + px * math.sin(yaw0) + py * math.cos(yaw0)
        )

        return objetivo_x, objetivo_y

    def reiniciar_dibujo(self, request, response):
        self.rodeando = False
        self.distancia_al_empezar = None
        self.posicion_anterior = None
        self.metros_rodeo = 0.0
        self.bloqueado = False
        self.camino_libre_desde = None
        self.estado = None

        return super().reiniciar_dibujo(request, response)

    def controlar(self):
        if self.pausado or self.reinicio_pendiente:
            self.detener()
            self.posicion_anterior = None
            self.camino_libre_desde = None
            return

        if self.bloqueado:
            self.detener()
            return

        ahora = time.monotonic()

        if (
            self.pose is None
            or self.ultima_odom is None
            or ahora - self.ultima_odom > 0.5
            or self.ultima_scan is None
            or ahora - self.ultima_scan > 0.5
        ):
            self.detener()
            self.camino_libre_desde = None
            self.informar(
                'esperando',
                'Esperando odometria y laser recientes.'
            )
            return

        if self.indice >= len(self.puntos):
            self.detener()
            return

        x, y, yaw = self.pose
        objetivo_x, objetivo_y = self.objetivo_actual()
        distancia = math.hypot(objetivo_x - x, objetivo_y - y)
        direccion = normalizar(
            math.atan2(objetivo_y - y, objetivo_x - x) - yaw
        )

        # Dejar que el controlador original registre el punto.
        if distancia < 0.03:
            self.rodeando = False
            self.camino_libre_desde = None
            self.posicion_anterior = None
            super().controlar()
            return

        frontal = self.sector(0.0, math.radians(35))
        derecha = self.sector(
            -math.pi / 2.0, math.radians(20)
        )
        diagonal_derecha = self.sector(
            -math.pi / 4.0, math.radians(15)
        )

        if not self.rodeando:
            # Empezar el rodeo cuando el robot ya mira
            # aproximadamente hacia el punto pendiente.
            longitud = min(
                self.distancia_deteccion,
                distancia + 0.15
            )

            if (
                abs(direccion) < 0.35
                and not self.camino_libre(0.0, longitud)
            ):
                self.rodeando = True
                self.distancia_al_empezar = distancia
                self.posicion_anterior = (x, y)
                self.metros_rodeo = 0.0
                self.camino_libre_desde = None

                self.informar(
                    'rodeo',
                    f'OBSTACULO: iniciando rodeo por la izquierda. '
                    f'Punto pendiente: {self.indice + 1}.'
                )
            else:
                self.informar(
                    'dibujo',
                    f'Siguiendo el dibujo: punto {self.indice + 1}.'
                )
                super().controlar()
                return

        if self.posicion_anterior is not None:
            anterior_x, anterior_y = self.posicion_anterior
            self.metros_rodeo += math.hypot(
                x - anterior_x, y - anterior_y
            )

        self.posicion_anterior = (x, y)

        # No continuar indefinidamente si el objetivo esta
        # encerrado o el obstaculo no se puede superar.
        if self.metros_rodeo > 12.0:
            self.detener()
            self.bloqueado = True
            self.informar(
                'bloqueado',
                'Rodeo demasiado largo. Revisa si la caja tapa '
                'el punto objetivo y reinicia la prueba.'
            )
            return

        # Salir del rodeo cuando se ha progresado y el camino
        # al objetivo permanece despejado durante 0.4 segundos.
        salida_posible = (
            self.metros_rodeo > 0.25
            and distancia < self.distancia_al_empezar - 0.05
            and self.camino_libre(direccion, distancia + 0.15)
        )

        if salida_posible:
            if self.camino_libre_desde is None:
                self.camino_libre_desde = ahora
            elif ahora - self.camino_libre_desde >= 0.4:
                self.rodeando = False
                self.girando = True
                self.posicion_anterior = None
                self.camino_libre_desde = None

                self.informar(
                    'retorno',
                    'Obstaculo superado: volviendo al punto pendiente.'
                )
                super().controlar()
                return
        else:
            self.camino_libre_desde = None

        velocidad = Twist()

        if frontal < 0.28:
            # Muy cerca: girar sobre el sitio para alejar el frente.
            velocidad.angular.z = 0.50

        elif frontal < 0.55:
            # Curva hacia la izquierda al aproximarse al obstaculo.
            velocidad.linear.x = 0.05
            velocidad.angular.z = 0.50

        elif diagonal_derecha < 0.28:
            velocidad.linear.x = 0.06
            velocidad.angular.z = 0.35

        elif derecha < 0.75:
            # Seguir el lateral dejando unos 35 cm.
            # Si esta demasiado cerca, girar a la izquierda.
            error = self.distancia_derecha - derecha
            velocidad.linear.x = 0.10
            velocidad.angular.z = limitar(
                1.5 * error, -0.40, 0.40
            )

        else:
            # Al acabar una cara de la caja, curvar a la derecha
            # para seguir su contorno.
            velocidad.linear.x = 0.08
            velocidad.angular.z = -0.40

        self.publisher.publish(velocidad)


def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )

    nodo = ObstacleAvoidance()

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