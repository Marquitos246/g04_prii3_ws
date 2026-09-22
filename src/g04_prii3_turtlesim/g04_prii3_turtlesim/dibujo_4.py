import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

from std_srvs.srv import SetBool
from std_srvs.srv import Trigger
from std_srvs.srv import Empty


class Dibujo4(Node):

    def __init__(self):
        super().__init__('dibujo_4')

        # =====================================================
        # PUBLISHER
        # Manda velocidades a turtlesim
        # =====================================================

        self.publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        # =====================================================
        # SUBSCRIBER
        # Recibe posición y orientación de la tortuga
        # =====================================================

        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.actualizar_pose,
            10
        )

        # =====================================================
        # SERVICIO PAUSAR / REANUDAR
        # =====================================================

        self.create_service(
            SetBool,
            '/dibujo_4/pausar',
            self.callback_pausa
        )

        # =====================================================
        # SERVICIO REINICIAR
        # =====================================================

        self.create_service(
            Trigger,
            '/dibujo_4/reiniciar',
            self.callback_reinicio
        )

        # =====================================================
        # SERVICIOS DE TURTLESIM
        # =====================================================

        # Resetear turtlesim
        self.cliente_reset = self.create_client(
            Empty,
            '/reset'
        )

        # Levantar y bajar lápiz
        self.cliente_lapiz = self.create_client(
            SetPen,
            '/turtle1/set_pen'
        )

        # =====================================================
        # VARIABLES
        # =====================================================

        self.pose = None
        self.pausado = False
        self.reiniciando = False

        # Acción actual
        self.paso = 0

        # Variables para medir movimientos rectos
        self.inicio_x = None
        self.inicio_y = None

        # Variables para medir la vuelta del 0
        self.ultimo_angulo = None
        self.angulo_acumulado = 0.0

        # Para esperar a que termine el cambio de lápiz
        self.futuro_lapiz = None

        # =====================================================
        # SECUENCIA PARA DIBUJAR 04
        # =====================================================
        #
        # circle:
        #   dibuja una vuelta completa
        #
        # pen:
        #   1 = levantar
        #   0 = bajar
        #
        # turn:
        #   orientación absoluta en radianes
        #
        # move:
        #   distancia que debe avanzar
        # =====================================================

        self.acciones = [

            # ===============================================
            # DIBUJAR EL 0
            # ===============================================

            ('circle',),

            # ===============================================
            # MOVERSE HASTA EL 4
            # ===============================================

            # Levantamos lápiz
            ('pen', 1),

            # Miramos a la derecha
            ('turn', 0.0),

            # Nos movemos hacia la derecha
            ('move', 2.8),

            # Giramos hacia arriba
            ('turn', math.pi / 2),

            # Subimos
            ('move', 1.5),

            # Miramos hacia abajo
            ('turn', -math.pi / 2),

            # Bajamos lápiz
            ('pen', 0),

            # ===============================================
            # DIBUJAR EL 4
            # ===============================================

            # Línea vertical izquierda
            ('move', 1.5),

            # Miramos hacia la derecha
            ('turn', 0.0),

            # Barra horizontal
            ('move', 1.5),

            # Levantamos lápiz
            ('pen', 1),

            # Miramos hacia arriba
            ('turn', math.pi / 2),

            # Subimos sin dibujar
            ('move', 1.5),

            # Miramos hacia abajo
            ('turn', -math.pi / 2),

            # Bajamos lápiz
            ('pen', 0),

            # Palo largo del 4
            ('move', 3.0),

            # Parar
            ('stop',)
        ]

        # Ejecutamos el control 20 veces por segundo
        self.timer = self.create_timer(
            0.05,
            self.controlar
        )

    # =========================================================
    # ACTUALIZAR POSE
    # =========================================================

    def actualizar_pose(self, msg):

        self.pose = msg

    # =========================================================
    # NORMALIZAR ÁNGULO
    # Convierte cualquier ángulo al rango -pi / +pi
    # =========================================================

    def normalizar_angulo(self, angulo):

        return math.atan2(
            math.sin(angulo),
            math.cos(angulo)
        )

    # =========================================================
    # LIMITAR VELOCIDAD
    # =========================================================

    def limitar(self, valor, minimo, maximo):

        return max(
            minimo,
            min(valor, maximo)
        )

    # =========================================================
    # PARAR
    # =========================================================

    def parar(self):

        # Twist vacío = todas las velocidades a cero
        msg = Twist()

        self.publisher.publish(msg)

    # =========================================================
    # PAUSAR / REANUDAR
    # =========================================================

    def callback_pausa(self, request, response):

        self.pausado = request.data

        self.parar()

        if self.pausado:

            response.success = True
            response.message = 'Dibujo detenido'

            self.get_logger().info(
                'Dibujo detenido'
            )

        else:

            response.success = True
            response.message = 'Dibujo reanudado'

            self.get_logger().info(
                'Dibujo reanudado'
            )

        return response

    # =========================================================
    # REINICIAR
    # =========================================================

    def callback_reinicio(self, request, response):

        self.parar()

        if not self.cliente_reset.service_is_ready():

            response.success = False
            response.message = 'Servicio reset no disponible'

            return response

        self.reiniciando = True
        self.pausado = False

        peticion = Empty.Request()

        futuro = self.cliente_reset.call_async(
            peticion
        )

        futuro.add_done_callback(
            self.reset_terminado
        )

        response.success = True
        response.message = 'Dibujo reiniciado'

        return response

    def reset_terminado(self, futuro):

        # Volvemos al principio
        self.paso = 0

        self.inicio_x = None
        self.inicio_y = None

        self.ultimo_angulo = None
        self.angulo_acumulado = 0.0

        self.futuro_lapiz = None

        self.reiniciando = False

    # =========================================================
    # CAMBIAR LÁPIZ
    # =========================================================

    def cambiar_lapiz(self, estado):

        peticion = SetPen.Request()

        # Color blanco
        peticion.r = 255
        peticion.g = 255
        peticion.b = 255

        # Grosor
        peticion.width = 3

        # 1 = levantado
        # 0 = dibujando
        peticion.off = estado

        self.futuro_lapiz = (
            self.cliente_lapiz.call_async(
                peticion
            )
        )

    # =========================================================
    # CONTROL PRINCIPAL
    # =========================================================

    def controlar(self):

        # Esperamos hasta conocer Pose
        if self.pose is None:
            return

        # Si estamos reiniciando
        if self.reiniciando:

            self.parar()
            return

        # Si está pausado
        if self.pausado:

            self.parar()
            return

        # Si hemos terminado
        if self.paso >= len(self.acciones):

            self.parar()
            return

        # =====================================================
        # ESPERAR A QUE TERMINE EL SERVICIO DEL LÁPIZ
        # =====================================================

        if self.futuro_lapiz is not None:

            self.parar()

            if self.futuro_lapiz.done():

                self.futuro_lapiz = None

                self.paso += 1

            return

        accion = self.acciones[self.paso]

        tipo = accion[0]

        # =====================================================
        # DIBUJAR EL 0
        # =====================================================

        if tipo == 'circle':

            msg = Twist()

            # Avanza y gira al mismo tiempo
            msg.linear.x = 1.0
            msg.angular.z = 1.0

            # Primera lectura del ángulo
            if self.ultimo_angulo is None:

                self.ultimo_angulo = self.pose.theta

            else:

                # Cambio de orientación desde la última lectura
                cambio = self.normalizar_angulo(
                    self.pose.theta
                    - self.ultimo_angulo
                )

                # Sumamos cuánto hemos girado realmente
                self.angulo_acumulado += abs(cambio)

                self.ultimo_angulo = self.pose.theta

            # Cuando ha girado prácticamente 360 grados
            if self.angulo_acumulado >= 2 * math.pi - 0.05:

                self.parar()

                self.ultimo_angulo = None
                self.angulo_acumulado = 0.0

                self.paso += 1

                return

            self.publisher.publish(msg)

            return

        # =====================================================
        # LEVANTAR / BAJAR LÁPIZ
        # =====================================================

        if tipo == 'pen':

            self.parar()

            if self.cliente_lapiz.service_is_ready():

                self.cambiar_lapiz(
                    accion[1]
                )

            return

        # =====================================================
        # GIRAR A UN ÁNGULO CONCRETO
        # =====================================================

        if tipo == 'turn':

            objetivo = accion[1]

            error = self.normalizar_angulo(
                objetivo - self.pose.theta
            )

            # Ya estamos mirando correctamente
            if abs(error) < 0.015:

                self.parar()

                self.paso += 1

                return

            msg = Twist()

            # Solo giramos, no avanzamos
            msg.linear.x = 0.0

            msg.angular.z = self.limitar(
                2.5 * error,
                -1.2,
                1.2
            )

            self.publisher.publish(msg)

            return

        # =====================================================
        # AVANZAR UNA DISTANCIA
        # =====================================================

        if tipo == 'move':

            distancia_objetivo = accion[1]

            # Guardamos dónde empieza el movimiento
            if self.inicio_x is None:

                self.inicio_x = self.pose.x
                self.inicio_y = self.pose.y

            # Calculamos cuánto hemos recorrido
            dx = self.pose.x - self.inicio_x
            dy = self.pose.y - self.inicio_y

            distancia = math.sqrt(
                dx**2 + dy**2
            )

            # Hemos llegado
            if distancia >= distancia_objetivo:

                self.parar()

                self.inicio_x = None
                self.inicio_y = None

                self.paso += 1

                return

            msg = Twist()

            # Avanzamos recto
            msg.linear.x = 1.0
            msg.angular.z = 0.0

            self.publisher.publish(msg)

            return

        # =====================================================
        # FIN
        # =====================================================

        if tipo == 'stop':

            self.parar()


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    rclpy.init(args=args)

    nodo = Dibujo4()

    rclpy.spin(nodo)

    nodo.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
