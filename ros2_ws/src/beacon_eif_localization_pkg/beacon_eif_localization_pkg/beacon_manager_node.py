#!/usr/bin/env python

import rclpy
from rclpy.node import Node

import numpy as np

# Importar mensajes de ROS2
from nav_msgs.msg import Odometry

# Importar mensajes de beacon_eif_localization_msgs
from beacon_eif_localization_msgs.msg import BeaconMeasurement, BeaconMeasurementArray

class Beacon:
    def __init__(self, id, position, noise_std):
        self.id = id
        self.position = position
        self.noise_std = noise_std

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

        # Crear suscriptor de posición del drone (ground truth)
        self.odometry_sub = self.create_subscription(Odometry, '/ground_truth/vehicle_odom', self.odometry_callback, 10)

        # Crear publicador de medida de baliza
        self.beacon_measurements_pub = self.create_publisher(BeaconMeasurementArray, '/beacons/measurement_array', 10)
        
        # Obtener parámetros del archivo de configuración
        update_rate = self.declare_parameter('update_rate', 100.0).value
        self.beacon_ids = self.declare_parameter('beacons.ids', ['']).value

        # Inicializar variables
        self.vehicle_position = None
    
        # Crear balizas y configurarlas
        self.beacons = []
        for beacon_id in self.beacon_ids:
            position = self.declare_parameter(f'beacons.{beacon_id}.position', [0.0, 0.0, 0.0]).value
            noise_std = self.declare_parameter(f'beacons.{beacon_id}.noise_std', 0.0).value
            self.beacons.append(Beacon(beacon_id, position, noise_std))

        # Crear timer para la actualización de las balizas
        self.beacon_update_timer = self.create_timer(1.0 / update_rate, self.update)
    
        # Log
        self.get_logger().info("Nodo de gestor de balizas iniciado")

    def odometry_callback(self, msg):
        # Convertir coordenadas de NED a ENU
        self.vehicle_position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]

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