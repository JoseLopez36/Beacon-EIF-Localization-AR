#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

# Importar el modelos
from EIF_drone_models import EIFDroneModel
from EIF_noise_models import R_noise_model, Q_noise_model

class EIFFilterNode(Node):
    def __init__(self):
        super().__init__('EIF_filter_node')
        self.get_logger().info("Iniciando nodo de filtro EIF...")

        # Variables para la creencia de la localizacion en forma canónica
        self.omega                  # Matriz de información
        self.xi                     # Vector de información 

        self.mu                     # vector media del estado estimado

        # Modelo:
        self.modelo = EIFDroneModel()
        

        # Matrices de ruido:
        self.Q = Q_noise_model()                     # Ruido de proceso
        self.R = R_noise_model()                     # Ruido de medición

        # Variables de resultado de predicción
        self.mu_pred                
        self.omega_pred             
        self.xi_pred                
        

        self.get_logger().info("Nodo de filtro EIF iniciado")

    def estimate_localization(self):
        self.get_logger().debug("Estimando localización...")
        
        self.predict()

        self.update()
        
        pass

    def predict(self, u):
        # Parte de predicción del filtro EIF
        # Calculo de la media de la estimación en t-1:
        self.mu = np.invert(self.omega) @ self.xi
        # Calculo prediciones de omega, xi y mu:
        self.omega_pred = np.invert( self.G @ np.invert(self.omega) @ np.transpose(self.G) + self.R )
        self.mu_pred = 0 #g(u, self.mu)
        self.xi_pred = self.omega_pred #@ self.mu_pred
        

        return

    def update(self, z):
        # Parte de actualización del filtro EIF
        self.omega = self.omega_pred # Sumatorio con las balizas

        self.xi = self.xi_pred 

        pass
