#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import numpy as np
from threading import Lock
from geometry_msgs.msg import PoseStamped 

# Importar el modelos
from .EIF_models import g_function, G_jacobian, h_function_n, H_jacobian_n, R_noise_model, Q_noise_model_n

# Importar mensajes de beacon_eif_localization_msgs
from gz_uwb_beacon_msgs.msg import Measurement

class EIFFilterNode(Node):
    def __init__(self):
        super().__init__('EIF_filter_node', automatically_declare_parameters_from_overrides=True, allow_undeclared_parameters=True)
        self.get_logger().info("Iniciando nodo de filtro EIF...")
        params = self._parameters  # Diccionario interno de parámetros
        self.get_logger().debug(f"Parámetros disponibles: {list(params.keys())}")

        # Obtener parámetros del archivo de configuración
        # Parametros del filtro
        self.filter_update_rate = self.get_parameter('filter_update_rate').value
        # Parametros necesarios para los modelos
        self.horizontal_vel = self.get_parameter('horizontal_vel').value 
        self.vertical_vel = self.get_parameter('vertical_vel').value
        self.valid_measurement_threshold = self.get_parameter('valid_measurement_threshold').value                     # Unidades: ns ! Default 10 ms 
        self.beacons_ids = self.get_parameter('beacons.ids').value 
        self.num_beacons = len(self.beacons_ids)
        self.beacons = {}
        for beacon_id in self.beacons_ids:
            position = self.get_parameter(f'{beacon_id}.position').value
            noise_std = self.get_parameter(f'{beacon_id}.noise_std').value
            self.beacons[beacon_id] = {"position" : position, "noise_std" : noise_std}                                                 

        # Modelo de predicción y medición:
        self.g, self.h_n, self.G, self.H_n = g_function, h_function_n, G_jacobian, H_jacobian_n

        # Matrices de ruido:
        self.Q_n = Q_noise_model_n                                                                                     # Ruido de medición
        self.R = R_noise_model(self.horizontal_vel, self.vertical_vel, 1.0 / self.filter_update_rate)                  # Ruido de proceso

        # Variables para la creencia de la localizacion en forma canónica
        self.omega = np.zeros([3,3],dtype=np.float64)                           # Matriz de información
        self.xi    = np.array([[0],[0],[0]],dtype=np.float64)                   # Vector de información 
        self.mu    = np.array([[0],[0],[0]],dtype=np.float64)                   # vector media del estado estimado

        # Variables de resultado de predicción
        self.omega_pred = np.zeros([3,3],dtype=np.float64)                           
        self.xi_pred    = np.array([[0],[0],[0]],dtype=np.float64)                   
        self.mu_pred    = np.array([[0],[0],[0]],dtype=np.float64)                     

        # Variables de suma para actualización
        self.omega_sum = np.zeros([3,3],dtype=np.float64)                           
        self.xi_sum    = np.array([[0],[0],[0]],dtype=np.float64)                      
    
        # Variables para la gestion de mediciones
        self.lock = Lock() # Para proteger el acceso a las mediciones
        self.last_measurements = [[beacon_id, None, None] for beacon_id in self.beacons_ids] # Lista de medidas de balizas [id, distancia, timestamp]

        # Subscriptores y publicadores
        self.predict_pub = self.create_publisher(PoseStamped,"/predicted_position", self.num_beacons)   

        #if self.beacon_id != "":
        for beacon_id in self.beacons_ids:
            self.create_subscription(Measurement, f'/measurement/{beacon_id}', self.beacon_measurements_callback, 10)
            
        # Temporizador para la frecuencia de actualización
        self.timer = self.create_timer(self.filter_update_rate, self.estimate_localization)

        self.get_logger().info("Nodo de filtro EIF iniciado")

    def beacon_measurements_callback(self, beacon_msg):
        beacon_id = beacon_msg.id
        beacon_distance = beacon_msg.distance
        beacon_timestamp = Time.from_msg(beacon_msg.timestamp).nanoseconds

        # Guardar la última medida de la baliza
        with self.lock:
            self.last_measurements[beacon_id][1] = beacon_distance
            self.last_measurements[beacon_id][2] = beacon_timestamp

        self.get_logger().debug(f"Medida de baliza {beacon_id}: distancia = {beacon_distance}, timestamp = {beacon_timestamp}")        

    def estimate_localization(self):
        self.get_logger().info("Estimando localización...")
        
        self.predict()

        now = self.get_clock().now().nanoseconds        # Mismo tipo de timestamp que el mensaje de la baliza
        with self.lock:
            z = self.last_measurements[ now - self.last_measurements[2] < self.valid_measurement_threshold ] # Filtrar medidas válidas ()
        z = z[:, :2] # id y distancia
        if len(z) == 0:
            self.get_logger().warn("No hay medidas válidas disponibles, no es posible actualizar predicción")
            return self.mu, self.omega, self.xi
        xi, omega = self.update(z)

        self.publish_estimation(xi, omega) # Publicar estimación de localización
        
        return self.mu, self.omega, self.xi

    def predict(self):
        # Parte de predicción del filtro EIF
        # Calculo de la media de la estimación en t-1:
        self.mu = np.linalg.inv(self.omega) @ self.xi

        # Calculo prediciones de omega, xi y mu:
        G = self.G(self.mu) # Jacobiano de la función de predicción
        self.omega_pred = np.linalg.inv( G @ np.linalg.inv(self.omega) @ np.transpose(G) + self.R )     #Matriz de información predicha    
        self.mu_pred = self.g(self.mu)
        self.xi_pred = self.omega_pred @ self.mu_pred                                           #Vector de información predicho
        
        return self.xi_pred, self.omega_pred, self.mu_pred

    def update(self, z):
        # Sumatorios de la actualización de matriz y vector de información segun el número de medidas disponibles
        for i in range(len(z)): 
            beacon_id = z[i][0]        # id != i
            H = self.H_n(self.mu, self.beacons[beacon_id]['position']) 
            Q = self.Q_n(self.beacons[beacon_id]['noise_std'])                  # Ruido de medición estandar de cada baliza
            y = z[i][1] - self.h_n(self.mu, self.beacons[beacon_id]['position']) + H @ self.mu_pred #Innovación
            
            self.omega_sum = self.omega_sum + np.transpose(H) @ np.linalg.inv(Q) @ H
            self.xi_sum = self.xi_sum + np.transpose(H) @ np.linalg.inv(Q) @ y

        # Actualizar la creencia de la localización
        self.omega = self.omega_pred + self.omega_sum
        self.xi = self.xi_pred + self.xi_sum

        # Resetear sumas
        self.xi_sum = np.fill(0)
        self.omega_sum = np.fill(0)                         

        return self.xi, self.omega

    def publish_estimation(self, xi, omega):
        # Publicar la estimación de localización
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "global"

        mu = np.linalg.inv(omega) @ xi

        pose_msg.pose.position.x = mu[0][0]
        pose_msg.pose.position.y = mu[1][0]
        pose_msg.pose.position.z = mu[2][0]

        self.predict_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    eif_filter_node = EIFFilterNode()
    rclpy.spin(eif_filter_node)
    eif_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()