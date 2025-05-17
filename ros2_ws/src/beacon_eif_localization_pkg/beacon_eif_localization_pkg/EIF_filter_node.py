#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

# Importar el modelos
from EIF_models import g_function, G_jacobian, h_function_n, H_jacobian_n,  R_noise_model, Q_noise_model

# Importar mensajes de beacon_eif_localization_msgs
from beacon_eif_localization_msgs.msg import BeaconMeasurement, BeaconMeasurementArray

class EIFFilterNode(Node):
    def __init__(self):
        super().__init__('EIF_filter_node')
        self.get_logger().info("Iniciando nodo de filtro EIF...")

        # Subscriptores y publicadores
        self.predict_pub = self.create_publisher( 10)   #TODO

        self.beacon_measurements_sub = self.create_subscriber(BeaconMeasurementArray, '/beacons/measurement_array',self.beacon_measurements_callback, 10)

        # Obtener parámetros del archivo de configuración
        update_rate = self.declare_parameter('update_rate', 100.0).value
        horizontal_vel = self.declare_parameter('horizontal_vel' , 10.0).value     # TODO: Estos parametros son de configuracion de PX4
        vertical_vel = self.declare_parameter('vertical_vel ', 4.0).value
        self.beacons = []
        for beacon_id in self.beacon_ids:
            position = self.declare_parameter(f'beacons.{beacon_id}.position', [0.0, 0.0, 0.0]).value
            noise_std = self.declare_parameter(f'beacons.{beacon_id}.noise_std', 0.0).value
            self.beacons.append((beacon_id, position, noise_std))               # No clase Beacon, solo se guarda la información

        # Modelo de predicción y medición:
        self.g, self.h_n, self.G, self.H_n = g_function, h_function_n, G_jacobian, H_jacobian_n

        # Matrices de ruido:
        self.Q = Q_noise_model(vel_xy_max, vel_z_max, 1.0 / update_rate)                     # Ruido de proceso
        self.R = R_noise_model(noise_std = self.beacons[2])                                  # Ruido de medición

        # Variables para la creencia de la localizacion en forma canónica
        self.omega = np.zeros([3,3])                         # Matriz de información
        self.xi    = np.array[[0],[0],[0]]                   # Vector de información 
        self.mu    = np.array[[0],[0],[0]]                   # vector media del estado estimado

        # Variables de resultado de predicción
        self.omega_pred = np.zeros([3,3])                         # Matriz de información
        self.xi_pred    = np.array[[0],[0],[0]]                   # Vector de información 
        self.mu_pred    = np.array[[0],[0],[0]]                   # vector media del estado estimado     
    
        self.get_logger().info("Nodo de filtro EIF iniciado")

    def beacon_measurements_callback(self, msg):
        #TODO: Depende del tipo de mensaje, si todas las medidas juntas o cada una por separado
        pass

    def estimate_localization(self):
        self.get_logger().info("Estimando localización...")
        
        self.predict()

        self.update()
        
        return self.mu, self.omega, self.xi

    def predict(self):
        # Parte de predicción del filtro EIF
        # Calculo de la media de la estimación en t-1:
        self.mu = np.invert(self.omega) @ self.xi

        # Calculo prediciones de omega, xi y mu:
        G = self.G(self.mu) # Jacobiano de la función de predicción
        self.omega_pred = np.invert( G @ np.invert(self.omega) @ np.transpose(G) + self.R )     #Matriz de información      
        self.mu_pred = self.g(self.mu)
        self.xi_pred = self.omega_pred @ self.mu_pred                                           #Vector de información 
        
        return self.xi_pred, self.omega_pred, self.mu_pred

    def update(self, z):
        # Parte de actualización del filtro EIF
        self.omega = self.omega_pred # TODO:Sumatorio con las balizas.
        
        #Para una baliza:
        H = self.H_n(self.mu, self.beacons[0][1]) 
        y = z[0] - self.h_n(self.mu, self.beacons[0][1]) + H @ self.mu_pred #Innovación

        self.xi = self.xi_pred + H @ np.invert(self.Q) @ y

        pass
