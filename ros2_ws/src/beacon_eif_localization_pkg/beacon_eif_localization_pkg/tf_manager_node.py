#!/usr/bin/env python

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

# Importar mensajes de PX4 (px4_msgs)
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleAttitude

class TFManagerNode(Node):

    def __init__(self):
        super().__init__('tf_manager_node')

        # Log
        self.get_logger().info("Iniciando nodo de gestion de transformaciones...")

        # Declarar perfil de QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Crear suscriptores
        self.position_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.position_callback, qos_profile)
        self.attitude_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos_profile)

        # Crear broadcast de transformaciones
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Obtener parámetros del archivo de configuración
        tf_update_rate = self.declare_parameter('tf_update_rate', 20.0).value

        # Declarar variables
        self.vehicle_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.local_frame_initialized = False

        # Publicar marco global
        self.publish_global_frame()

        # Crear timer para la actualización de la transformaciones
        self.tf_timer = self.create_timer(1.0 / tf_update_rate, self.update)
    
        # Log
        self.get_logger().info("Nodo de gestion de transformaciones iniciado")

    def position_callback(self, msg):
        # Convertir coordenadas de NED a ENU
        self.vehicle_position = [msg.x, -msg.y, -msg.z]

    def attitude_callback(self, msg):
        # Convertir coordenadas de NED a ENU
        self.vehicle_attitude = [msg.q[0], msg.q[1], -msg.q[2], -msg.q[3]]

    def update(self):
        # Publicar transformación del frame 'global' (origen) al frame 'local' (marco local del drone)
        if not self.local_frame_initialized:
            self.publish_local_frame()
            self.local_frame_initialized = True

        # Publicar transformación del frame 'local' (marco local del drone) al frame 'body' (cuerpo del drone)
        if self.local_frame_initialized:
            self.publish_body_frame()

    def publish_global_frame(self):
        # Publicar transformación del frame 'global' (origen)
        # Crear mensaje de transformación
        t = TransformStamped()
        
        # Establecer headers
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'global'
        
        # Establecer posición relativa al origen
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # Establecer orientación
        t.transform.rotation.w = 1.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        
        # Publicar transformación
        self.static_broadcaster.sendTransform(t)
            
    def publish_local_frame(self):
        # Publicar transformación del frame 'global' (origen) al frame 'local' (marco local del drone)
        # Crear mensaje de transformación
        t = TransformStamped()
        
        # Establecer headers
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'global'
        t.child_frame_id = 'local'
        
        # Establecer posición relativa al origen
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # Establecer orientación
        t.transform.rotation.w = 1.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        
        # Publicar transformación
        self.static_broadcaster.sendTransform(t)      

    def publish_body_frame(self):
        # Crear mensaje de transformación
        t = TransformStamped()
        
        # Establecer headers
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'local'
        t.child_frame_id = 'body'
        
        # Establecer posición relativa al origen (local)
        t.transform.translation.x = float(self.vehicle_position[0])
        t.transform.translation.y = float(self.vehicle_position[1])
        t.transform.translation.z = float(self.vehicle_position[2])
        
        # Establecer orientación
        t.transform.rotation.w = float(self.vehicle_attitude[0])
        t.transform.rotation.x = float(self.vehicle_attitude[1])
        t.transform.rotation.y = float(self.vehicle_attitude[2])
        t.transform.rotation.z = float(self.vehicle_attitude[3])
        
        # Publicar transformación
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    tf_manager_node = TFManagerNode()
    rclpy.spin(tf_manager_node)
    tf_manager_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()