#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
import numpy as np
from threading import Lock

# Importar el modelos
from EIF_models import g_function, G_jacobian, h_function_n, H_jacobian_n,  R_noise_model, Q_noise_model_n

# Importar mensajes de beacon_eif_localization_msgs
from beacon_eif_localization_msgs.msg import BeaconMeasurement

class EIFFilterNode(Node):
    def __init__(self):
        super().__init__('EIF_filter_node', automatically_declare_parameters_from_overrides=True)
        self.get_logger().info("Iniciando nodo de filtro EIF...")

        # Obtener parámetros del archivo de configuración
        # Parametros del filtro
        self.filter_update_rate = self.declare_parameter('filter_update_rate', 100.0).value
        # Parametros necesarios para los modelos
        self.horizontal_vel = self.declare_parameter('horizontal_vel' , 10.0).value 
        self.vertical_vel = self.declare_parameter('vertical_vel ', 4.0).value
        self.valid_measurement_threshold = self.declare_parameter('valid_measurement_threshold', 0.5).value         #TODO: Comprobar compatibilidad con el timestamp de las medidas
        self.beacons = []
        for beacon_id in self.beacon_ids:
            position = self.declare_parameter(f'beacons.{beacon_id}.position', [0.0, 0.0, 0.0]).value
            noise_std = self.declare_parameter(f'beacons.{beacon_id}.noise_std', 0.0).value
            self.beacons.append((beacon_id, position, noise_std))               # No clase Beacon, solo se guarda la información
        self.num_beacons = len(self.beacons.shape[0])

        # Modelo de predicción y medición:
        self.g, self.h_n, self.G, self.H_n = g_function, h_function_n, G_jacobian, H_jacobian_n

        # Matrices de ruido:
        #self.Q = Q_noise_model(self.horizontal_vel, self.vertical_vel, 1.0 / self.filter_update_rate)       # Ruido de predicción
        self.Q_n = Q_noise_model_n
        self.R = R_noise_model(noise_std = self.beacons[2])                                  # Ruido de proceso

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
        self.last_measurements = np.zeros([self.num_beacons, 3]) # Ultimas medidas de cada baliza
        self.last_measurements[:][0] = self.beacons[:,0] # ID de la baliza

        # Subscriptores y publicadores
        self.predict_pub = self.create_publisher()   #TODO

        for i in range(self.num_beacons):  
            self.create_subscription(BeaconMeasurement, f'beacon_measurements/{self.beacons[i][0]}', self.beacon_measurements_callback, 10)


        # Temporizador para la frecuencia de actualización
        self.timer = self.create_timer(self.filter_update_rate, self.estimate_localization)

        self.get_logger().info("Nodo de filtro EIF iniciado")

    def beacon_measurements_callback(self, beacon_msg):
        beacon_id = beacon_msg.id
        beacon_distance = beacon_msg.distance
        beacon_timestamp = beacon_msg.timestamp

        # Guardar la última medida de la baliza
        with self.lock:
            self.last_measurements[beacon_id][1] = beacon_distance
            self.last_measurements[beacon_id][2] = beacon_timestamp

        self.get_logger().debug(f"Medida de baliza {beacon_id}: distancia = {beacon_distance}, timestamp = {beacon_timestamp}")        

    def estimate_localization(self):
        self.get_logger().info("Estimando localización...")
        
        self.predict()

        now = Clock().now()         # Mismo tipo de timestamp que el mensaje de la baliza
        with self.lock:
            z = self.last_measurements[ now - self.last_measurements[2] < self.valid_measurement_threshold ] # Filtrar medidas válidas
        z = z[:, :2] # id y distancia
        
        self.update(z)
        
        return self.mu, self.omega, self.xi

    def predict(self):
        # Parte de predicción del filtro EIF
        # Calculo de la media de la estimación en t-1:
        self.mu = np.invert(self.omega) @ self.xi

        # Calculo prediciones de omega, xi y mu:
        G = self.G(self.mu) # Jacobiano de la función de predicción
        self.omega_pred = np.invert( G @ np.invert(self.omega) @ np.transpose(G) + self.R )     #Matriz de información predicha    
        self.mu_pred = self.g(self.mu)
        self.xi_pred = self.omega_pred @ self.mu_pred                                           #Vector de información predicho
        
        return self.xi_pred, self.omega_pred, self.mu_pred

    def update(self, z):
        # Sumatorios de la actualización de matriz y vector de información segun el número de medidas disponibles
        for i in range(z.shape[0]): 
            beacon_id = z[i][0]        # id != i
            H = self.H_n(self.mu, self.beacons[beacon_id][1]) 
            Q = self.Q_n(self.beacons[beacon_id][2])                  # Ruido de medición estandar de cada baliza
            y = z[i][1] - self.h_n(self.mu, self.beacons[beacon_id][1]) + H @ self.mu_pred #Innovación
            
            self.omega_sum = self.omega_sum + np.transpose(H) @ np.invert(Q) @ H
            self.xi_sum = self.xi_sum + np.transpose(H) @ np.invert(Q) @ y

        # Actualizar la creencia de la localización
        self.omega = self.omega_pred + self.omega_sum
        self.xi = self.xi_pred + self.xi_sum

        # Resetear sumas
        self.xi_sum = np.fill(0)
        self.omega_sum = np.fill(0)                         

        return self.xi, self.omega
