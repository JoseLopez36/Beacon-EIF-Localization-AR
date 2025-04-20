#!/usr/bin/env python

import rclpy
import numpy as np
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
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
        self.vehicle_position_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_position_callback, qos_profile)
        self.vehicle_attitude_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile)
        self.vehicle_setpoint_sub = self.create_subscription(DroneSetpoint, '/setpoint/vehicle_position', self.vehicle_setpoint_callback, 10)

        # Crear publicadores
        self.vehicle_pose_pub = self.create_publisher(PoseStamped, "/visualization/vehicle_pose", 10)
        self.vehicle_setpoint_pub = self.create_publisher(Marker, '/visualization/vehicle_setpoint', 10)

        # Obtener parámetros del archivo de configuración
        visualization_update_rate = self.declare_parameter('visualization_update_rate', 20.0).value

        # Declarar variables
        self.vehicle_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.vehicle_setpoint = np.array([0.0, 0.0, 0.0])

        # Crear timer para la actualización de la visualización
        self.dt = 1.0 / visualization_update_rate
        self.visualization_timer = self.create_timer(self.dt, self.update_visualization)
    
        # Log
        self.get_logger().info("Nodo de visualización iniciado")

    def vehicle_position_callback(self, msg):
        # Convertir coordenadas de NED a ENU
        self.vehicle_position = [msg.x, -msg.y, -msg.z]

    def vehicle_attitude_callback(self, msg):
        # Convertir coordenadas de NED a ENU
        self.vehicle_attitude = [msg.q[0], msg.q[1], -msg.q[2], -msg.q[3]]

    def vehicle_setpoint_callback(self, msg):
        # Setpoint en ENU
        self.vehicle_setpoint = [msg.x, msg.y, msg.z]

    def update_visualization(self):
        vehicle_pose_msg = self.create_pose_msg('map', self.vehicle_position, self.vehicle_attitude)
        self.vehicle_pose_pub.publish(vehicle_pose_msg)

        # Publish point marker for setpoint
        vehicle_setpoint_msg = self.create_point_marker(1, self.vehicle_setpoint)
        self.vehicle_setpoint_pub.publish(vehicle_setpoint_msg)

    def create_pose_msg(self, frame_id, position, attitude):
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.pose.orientation.w = float(attitude[0])
        msg.pose.orientation.x = float(attitude[1])
        msg.pose.orientation.y = float(attitude[2])
        msg.pose.orientation.z = float(attitude[3])
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        return msg
    
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