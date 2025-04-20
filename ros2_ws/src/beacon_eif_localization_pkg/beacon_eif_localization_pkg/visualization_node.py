#!/usr/bin/env python

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

# Importar mensajes de beacon_eif_localization_msgs
from beacon_eif_localization_msgs.msg import DroneSetpoint

# Importar mensajes de PX4 (px4_msgs)
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleAttitude

class VisualizationNode(Node):

    def __init__(self):
        super().__init__('visualization_node')

        # Log
        self.get_logger().info("Iniciando nodo de visualizacion...")

        # Declarar perfil de QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Crear suscriptores
        self.vehicle_setpoint_sub = self.create_subscription(DroneSetpoint, '/setpoint/vehicle_position', self.vehicle_setpoint_callback, 10)

        # Crear publicadores
        self.vehicle_setpoint_pub = self.create_publisher(Marker, '/visualization/vehicle_setpoint', 10)

        # Obtener parámetros del archivo de configuración
        update_rate = self.declare_parameter('update_rate', 20.0).value

        # Declarar variables
        self.vehicle_setpoint = np.array([0.0, 0.0, 0.0])

        # Crear timer para la actualización de la visualización
        self.dt = 1.0 / update_rate
        self.visualization_timer = self.create_timer(self.dt, self.update)
    
        # Log
        self.get_logger().info("Nodo de visualizacion iniciado")

    def vehicle_setpoint_callback(self, msg):
        # Setpoint en ENU
        self.vehicle_setpoint = [msg.x, msg.y, msg.z]

    def update(self):
        # Publicar marcador de punto para setpoint
        vehicle_setpoint_msg = self.create_point_marker(1, self.vehicle_setpoint)
        self.vehicle_setpoint_pub.publish(vehicle_setpoint_msg)

    def create_point_marker(self, id, position):
        msg = Marker()
        msg.action = Marker.ADD
        msg.header.frame_id = 'map'
        msg.ns = 'point'
        msg.id = id
        msg.type = Marker.SPHERE
        msg.scale.x = 1.25
        msg.scale.y = 1.25
        msg.scale.z = 1.25
        # Yellow color
        msg.color.r = 1.0
        msg.color.g = 1.0
        msg.color.b = 0.0
        msg.color.a = 1.0
        msg.pose.position.x = position[0];
        msg.pose.position.y = position[1];
        msg.pose.position.z = position[2];
        msg.pose.orientation.x = 0.0;
        msg.pose.orientation.y = 0.0;
        msg.pose.orientation.z = 0.0;
        msg.pose.orientation.w = 1.0;
        return msg

def main(args=None):
    rclpy.init(args=args)
    visualization_node = VisualizationNode()
    rclpy.spin(visualization_node)
    visualization_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()