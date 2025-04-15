#!/usr/bin/env python

import rclpy
import numpy as np
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

# Importar mensajes de beacon_eif_localization_msgs
from beacon_eif_localization_msgs.msg import DroneSetpoint

# Importar mensajes de PX4 (px4_msgs)
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import GotoSetpoint
from px4_msgs.msg import VehicleCommand

class DroneControlNode(Node):

    def __init__(self):
        super().__init__('drone_control_node')

        # Log
        self.get_logger().info("Iniciando nodo de control de drone...")

        # Declarar perfil de QoS
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Crear suscriptor de estado
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.status_callback,
            qos_profile)
        
        # Crear suscriptor de posición local
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.position_callback,
            qos_profile)
        
        # Crear suscriptor de referencia de posición
        self.setpoint_sub = self.create_subscription(
            DroneSetpoint,
            'drone_setpoint',
            self.setpoint_callback,
            10)

        # Crear publicador de modo de control offboard
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)

        # Crear publicador de comando de vehículo
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        # Crear publicador de trayectoria
        self.setpoint_pub = self.create_publisher(GotoSetpoint, '/fmu/in/goto_setpoint', qos_profile)

        # Declarar variables de estado del vehículo
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_DISARMED
        self.flight_check = False
        self.offboard_mode = False

        # Declarar variables para la máquina de estados
        self.current_state = "IDLE"
        self.last_state = self.current_state
        self.state_duration = 0.0
        self.last_update_time = self.get_clock().now()

        # Declarar variables de posición y setpoint
        self.position = None
        self.setpoint = None
        
        # Obtener parámetros del archivo de configuración
        state_update_rate = self.declare_parameter('state_update_rate', 5.0).value
        position_update_rate = self.declare_parameter('position_update_rate', 10.0).value
        self.arm_timeout = self.declare_parameter('arm_timeout', 3.0).value
        self.takeoff_timeout = self.declare_parameter('takeoff_timeout', 5.0).value
        self.takeoff_altitude = self.declare_parameter('takeoff_altitude', 3.0).value
        self.horizontal_vel = self.declare_parameter('horizontal_vel', 1.0).value
        self.vertical_vel = self.declare_parameter('vertical_vel', 0.5).value
        self.heading_vel = self.declare_parameter('heading_vel', 0.5).value

        # Crear timer para la máquina de estados y para la actualización de la posición
        self.state_timer = self.create_timer(1.0 / state_update_rate, self.update_state)
        self.position_timer = self.create_timer(1.0 / position_update_rate, self.update_position)
    
        # Log
        self.get_logger().info("Nodo de control de drone iniciado")
 
    def status_callback(self, msg):
        # Obtener estado de navegación, armado y flight check
        if (msg.nav_state != self.nav_state):
            self.get_logger().info(f"Nuevo estado de navegación: {msg.nav_state}")
            self.nav_state = msg.nav_state
        if (msg.arming_state != self.arm_state):
            self.get_logger().info(f"Nuevo estado de armado: {msg.arming_state}")
            self.arm_state = msg.arming_state
        if (self.flight_check != msg.pre_flight_checks_pass):
            self.get_logger().info(f"Nuevo flight check: {msg.pre_flight_checks_pass}")
            self.flight_check = msg.pre_flight_checks_pass

    def position_callback(self, msg):
        # Convertir coordenadas de NED a ENU
        self.position = [msg.x, -msg.y, -msg.z]

    def setpoint_callback(self, msg):
        # Setpoint en ENU
        self.setpoint = [msg.x, msg.y, msg.z, msg.yaw]

    def update_state(self):
        # Verificar condiciones de seguridad (siempre primero)
        if not self.flight_check:
            self.current_state = "IDLE"
            self.get_logger().info("Flight check falló, volviendo a IDLE")
            return
        elif not self.position:
            self.current_state = "IDLE"
            self.get_logger().info("No se ha recibido posición, volviendo a IDLE")
            return

        # Implementación de la máquina de estados
        match self.current_state:
            case "IDLE":
                if self.arm_state == VehicleStatus.ARMING_STATE_DISARMED:
                    # Si el dron está desarmado, pasar a ARMING
                    self.current_state = "ARMING"
                elif self.arm_state == VehicleStatus.ARMING_STATE_ARMED:
                    # Si el dron está armado, pasar a TAKEOFF
                    self.current_state = "TAKEOFF"
            
            case "ARMING":
                if self.arm_state == VehicleStatus.ARMING_STATE_ARMED:
                    # Si el dron está armado, pasar a TAKEOFF
                    self.current_state = "TAKEOFF"
                else:
                    # Intentar armar el vehículo
                    self.handle_arming()
                    
                # Timeout
                if self.state_duration > self.arm_timeout:
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Timeout en {self.last_state}, volviendo a {self.current_state}")
            
            case "TAKEOFF":
                if self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                    # Si el dron está en estado de despegue, pasar a TAKINGOFF
                    self.current_state = "TAKINGOFF"
                else:
                    # Intentar despegar
                    self.handle_takeoff()
                    
                # Timeout
                if self.state_duration > self.takeoff_timeout:
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Timeout en {self.last_state}, volviendo a {self.current_state}")

            case "TAKINGOFF":
                if self.position[2] >= self.takeoff_altitude * 0.666:
                    # Si el dron ha sobrepasado el 50% de la altitud objetivo, pasar a OFFBOARD
                    self.current_state = "OFFBOARD"
                    # Esperar 1 segundo
                    time.sleep(1)
                elif self.arm_state == VehicleStatus.ARMING_STATE_DISARMED:
                    # Si el dron está desarmado, volver a IDLE
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Desarmado durante {self.last_state}, volviendo a {self.current_state}")
                else:
                    self.get_logger().info(f"Despegando... (altitud: {self.position[2]}m, objetivo: {self.takeoff_altitude}m)")
                    
                # Timeout
                if self.state_duration > self.takeoff_timeout:
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Timeout en {self.last_state}, volviendo a {self.current_state}")
            
            case "OFFBOARD":
                if self.arm_state == VehicleStatus.ARMING_STATE_DISARMED:
                    # Si el dron está desarmado, volver a IDLE
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Desarmado durante {self.last_state}, volviendo a {self.current_state}")
                else:
                    # Activar modo offboard
                    self.handle_offboard()

        # Actualizar duración del estado
        self.state_duration += (self.get_clock().now() - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = self.get_clock().now()

        # Actualizar último estado
        if self.last_state != self.current_state:
            self.get_logger().info(f"Transición de estado: {self.last_state} -> {self.current_state} (duracion: {self.state_duration}s)")
            self.last_state = self.current_state
            self.state_duration = 0.0

    def update_position(self):
        # Verificar si estamos en modo offboard
        if not self.offboard_mode:
            return
        
        # Publicar comando de posición
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.offboard_mode_pub.publish(offboard_msg)
        
        if self.setpoint:
            # Volar hacia el setpoint (convertir coordenadas de ENU a NED)
            goto_msg = GotoSetpoint()
            # Posición
            goto_msg.position[0] = self.setpoint[0]   # x actual
            goto_msg.position[1] = -self.setpoint[1]  # y actual
            goto_msg.position[2] = -self.setpoint[2]  # z actual
            goto_msg.heading = -self.setpoint[3]      # Sin rotación
            # Restricciones de velocidad
            goto_msg.flag_set_max_horizontal_speed = True
            goto_msg.max_horizontal_speed = self.horizontal_vel
            goto_msg.flag_set_max_vertical_speed = True
            goto_msg.max_vertical_speed = self.vertical_vel
            goto_msg.flag_set_max_heading_rate = True
            goto_msg.max_heading_rate = self.heading_vel
            self.setpoint_pub.publish(goto_msg)
            self.get_logger().info(f"Setpoint / Posicion: xr={self.setpoint[0]} / yr={self.setpoint[1]} / zr={self.setpoint[2]} / x={self.position[0]} / y={self.position[1]} / z={self.position[2]}")
        else:
            # Mantener posición actual (convertir coordenadas de NED a ENU)
            goto_msg = GotoSetpoint()
            goto_msg.position[0] = self.position[0]   # x actual
            goto_msg.position[1] = -self.position[1]  # y actual
            goto_msg.position[2] = -self.position[2]  # z actual
            goto_msg.heading = 0.0                    # Sin rotación
            # Restricciones de velocidad
            goto_msg.flag_set_max_horizontal_speed = True
            goto_msg.max_horizontal_speed = self.horizontal_vel
            goto_msg.flag_set_max_vertical_speed = True
            goto_msg.max_vertical_speed = self.vertical_vel
            goto_msg.flag_set_max_heading_rate = True
            goto_msg.max_heading_rate = self.heading_vel
            self.setpoint_pub.publish(goto_msg)
            self.get_logger().info(f"Manteniendo posicion: x={self.position[0]} / y={self.position[1]} / z={self.position[2]}")

    def handle_arming(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Comando de armado enviado")

    def handle_takeoff(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=1.0, param7=self.takeoff_altitude)
        self.get_logger().info(f"Comando de despegue enviado (altitud: {self.takeoff_altitude}m)")

    def handle_offboard(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Modo OFFBOARD activado")
        self.offboard_mode = True

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7         # valor de altitud en el comando de despegue
        msg.command = command       # ID del comando
        msg.target_system = 1       # sistema que debe ejecutar el comando
        msg.target_component = 1    # componente que debe ejecutar el comando, 0 para todos los componentes
        msg.source_system = 1       # sistema que envía el comando
        msg.source_component = 1    # componente que envía el comando
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000) # tiempo en microsegundos
        self.vehicle_command_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    drone_control_node = DroneControlNode()
    rclpy.spin(drone_control_node)
    drone_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()