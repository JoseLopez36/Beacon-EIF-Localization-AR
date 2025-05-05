#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf2_ros import StaticTransformBroadcaster

import numpy as np

from geometry_msgs.msg import TransformStamped

# Importar mensajes de beacon_eif_localization_msgs
from beacon_eif_localization_msgs.msg import BeaconMeasurement, BeaconMeasurementArray

# Importar mensajes de PX4 (px4_msgs)
from px4_msgs.msg import VehicleLocalPosition

class Beacon:
    def __init__(self, id, position, noise_std, frame_id, node=None):
        self.id = id
        self.position = position
        self.noise_std = noise_std
        self.frame_id = frame_id
        self.node = node

    def create_static_transform(self, tf_static_broadcaster):
        # Crear mensaje de transformación estática
        static_transform = TransformStamped()
        static_transform.header.stamp = self.node.get_clock().now().to_msg()
        static_transform.header.frame_id = self.frame_id
        static_transform.child_frame_id = self.id
        static_transform.transform.translation.x = self.position[0]
        static_transform.transform.translation.y = self.position[1]
        static_transform.transform.translation.z = self.position[2]
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0
        
        # Publicar mensaje de transformación estática
        tf_static_broadcaster.sendTransform(static_transform)

    def compute_distance(self, vehicle_position):
        # Distancia entre baliza y vehículo
        distance = np.sqrt(
            (self.position[0] - vehicle_position[0])**2 +
            (self.position[1] - vehicle_position[1])**2 +
            (self.position[2] - vehicle_position[2])**2
            )
        # Añadir AWGN con desviación estándar de ruido configurada
        distance = distance + np.random.normal(0.0, self.noise_std, 1)
        return distance

class BeaconManagerNode(Node):

    def __init__(self):
        super().__init__('beacon_manager_node')

        # Log
        self.get_logger().info("Iniciando nodo de gestor de balizas...")

        # Declarar perfil de QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Crear suscriptor de posición del drone (ground truth)
        self.vehicle_position_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_position_callback, qos_profile)

        # Crear publicador de medida de baliza
        self.beacon_measurements_pub = self.create_publisher(BeaconMeasurementArray, '/beacons/measurement_array', 10)
        
        # Obtener parámetros del archivo de configuración
        update_rate = self.declare_parameter('update_rate', 100.0).value
        self.beacon_ids = self.declare_parameter('beacons.ids', ['']).value

        # Comprobar si se han configurado balizas
        if len(self.beacon_ids) == 0:
            self.get_logger().error("No se han configurado balizas")
            rclpy.shutdown()
            return

        # Log de la configuración de balizas detectada
        self.get_logger().info(f"Configuración de balizas detectada: {len(self.beacon_ids)} balizas")

        # Inicializar variables
        self.vehicle_position = None

        # Inicializar componentes de tf2
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
    
        # Crear balizas y configurarlas
        self.beacons = []
        self.create_beacons()

        # Crear timer para la actualización de las balizas
        self.beacon_update_timer = self.create_timer(1.0 / update_rate, self.update)
    
        # Log
        self.get_logger().info("Nodo de gestor de balizas iniciado")

    def create_beacons(self):
        self.get_logger().info("Comenzando creacion de balizas...")

        try:
            for id in self.beacon_ids:
                # Obtener parámetros de la baliza
                position = self.declare_parameter(f'beacons.{id}.position', [0.0, 0.0, 0.0]).value
                noise_std = self.declare_parameter(f'beacons.{id}.noise_std', 0.0).value
                frame_id = self.declare_parameter(f'beacons.{id}.frame_id', 'map').value

                # Añadir baliza al mapa de balizas
                self.beacons.append(Beacon(id, position, noise_std, frame_id, node=self))

                # Crear transformación estática
                self.beacons[-1].create_static_transform(self.tf_static_broadcaster)

                # Log
                self.get_logger().info(f"Baliza {id} creada en {position}")
            
            self.get_logger().info(f"Creación de balizas completada. Total: {len(self.beacons)} balizas")

        except Exception as e:
            self.get_logger().error(f"Error al crear balizas: {e}")

    def vehicle_position_callback(self, msg):
        # Convertir coordenadas de NED a ENU
        self.vehicle_position = [msg.x, -msg.y, -msg.z]

    def update(self):
        # Verificar si se ha recibido la posición del vehículo
        if self.vehicle_position is None:
            self.get_logger().info("Esperando posición del vehículo...")
            return
        
        # Crear mensaje de medida de balizas
        msg = BeaconMeasurementArray()

        # Iterar sobre todas las balizas
        for beacon in self.beacons:
            # Calcular distancia a cada baliza
            distance = beacon.compute_distance(self.vehicle_position)
            # Crear y añadir medida de baliza al mensaje
            measurement = BeaconMeasurement()
            measurement.id = beacon.id
            measurement.distance = float(distance)
            measurement.valid = True # De momento se asume que todas las medidas son válidas
            measurement.timestamp = self.get_clock().now().to_msg()
            msg.measurements.append(measurement)

        # Publicar mensaje de medida de balizas
        self.beacon_measurements_pub.publish(msg)
            
def main(args=None):
    rclpy.init(args=args)
    beacon_manager_node = BeaconManagerNode()
    rclpy.spin(beacon_manager_node)
    beacon_manager_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()