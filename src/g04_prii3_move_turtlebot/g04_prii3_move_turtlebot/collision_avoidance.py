import math
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan

from g04_prii3_move_turtlebot.draw_number import DrawNumber


class CollisionAvoidance(DrawNumber):
    def __init__(self):
        # Reutilizar el dibujo, la odometria, la traza y los servicios.
        super().__init__()

        self.distancia_parada = 0.40
        self.distancia_reanudacion = 0.50
        self.semiangulo_frontal = math.radians(30.0)

        self.ultima_scan = None
        self.scan_valido = False
        self.obstaculo = False
        self.distancia_frontal = math.inf
        self.estado_laser = None

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.recibir_scan,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            'Deteccion frontal activa: parar a 0.40 m '
            'y continuar cuando quede libre hasta 0.50 m.'
        )

    def recibir_scan(self, mensaje):
        distancias = []

        for indice, distancia in enumerate(mensaje.ranges):
            angulo = (
                mensaje.angle_min
                + indice * mensaje.angle_increment
            )

            # Convertir 0..2*pi en -pi..pi para incluir
            # ambos lados del frente del robot.
            angulo = math.atan2(
                math.sin(angulo), math.cos(angulo)
            )

            if abs(angulo) > self.semiangulo_frontal:
                continue

            if math.isnan(distancia):
                continue

            # Infinito positivo: ningun obstaculo detectado.
            if distancia == math.inf:
                distancias.append(math.inf)
            elif (
                math.isfinite(distancia)
                and mensaje.range_min <= distancia <= mensaje.range_max
            ):
                distancias.append(distancia)

        self.ultima_scan = time.monotonic()
        self.scan_valido = bool(distancias)

        if not self.scan_valido:
            return

        self.distancia_frontal = min(distancias)

        # Dos umbrales para evitar alternar continuamente
        # entre parar y avanzar cerca de una misma distancia.
        if self.obstaculo:
            if self.distancia_frontal > self.distancia_reanudacion:
                self.obstaculo = False
        elif self.distancia_frontal <= self.distancia_parada:
            self.obstaculo = True

    def informar_estado(self, estado, texto):
        # Mostrar el mensaje solo cuando cambia el estado.
        if estado != self.estado_laser:
            self.estado_laser = estado
            self.get_logger().info(texto)

    def controlar(self):
        # Mantener la pausa manual y el proceso de reinicio.
        if self.pausado or self.reinicio_pendiente:
            self.detener()
            return

        if self.indice >= len(self.puntos):
            self.detener()
            return

        if self.ultima_scan is None:
            self.detener()
            self.informar_estado(
                'esperando',
                'Robot detenido: esperando datos de /scan.'
            )
            return

        if time.monotonic() - self.ultima_scan > 0.5:
            self.detener()
            self.informar_estado(
                'sin_datos',
                'Robot detenido: han dejado de llegar datos del laser.'
            )
            return

        if not self.scan_valido:
            self.detener()
            self.informar_estado(
                'invalido',
                'Robot detenido: no hay medidas frontales validas.'
            )
            return

        if self.obstaculo:
            self.detener()
            self.informar_estado(
                'bloqueado',
                f'OBSTACULO a {self.distancia_frontal:.2f} m. '
                'Esperando que se retire.'
            )
            return

        self.informar_estado(
            'libre',
            'Frente libre: dibujo habilitado.'
        )

        # Ejecutar el controlador de dibujo ya comprobado.
        super().controlar()


def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )

    nodo = CollisionAvoidance()

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