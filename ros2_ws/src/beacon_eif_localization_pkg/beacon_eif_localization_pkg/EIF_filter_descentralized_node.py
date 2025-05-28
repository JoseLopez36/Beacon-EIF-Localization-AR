#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.executors import SingleThreadedExecutor
from rclpy.guard_condition import GuardCondition
from message_filters import ApproximateTimeSynchronizer
import numpy as np
from threading import Lock, Event
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped 

# Importar el modelos
from .EIF_models import g_function, G_jacobian, h_function_n, H_jacobian_n, R_noise_model, Q_noise_model_n

# Importar mensajes de beacon_eif_localization_msgs
from gz_uwb_beacon_msgs.msg import EIFInput, EIFOutput
from beacon_eif_localization_msgs.msg import ProcessStats

class EIFFilterDescentralizedNode(Node):
    def __init__(self):
        super().__init__('EIF_filter_descentralized_node', automatically_declare_parameters_from_overrides=True, allow_undeclared_parameters=True)
        self.get_logger().info("Iniciando nodo de filtro EIF...")
        params = self._parameters  # Diccionario interno de parámetros
        self.get_logger().debug(f"Parámetros disponibles: {list(params.keys())}")

        # Obtener parámetros del archivo de configuración
        # Parametros del filtro
        self.filter_update_rate = self.get_parameter('filter_update_rate').value
        # Parametros necesarios para los modelos
        self.horizontal_vel = self.get_parameter('horizontal_vel').value 
        self.vertical_vel = self.get_parameter('vertical_vel').value
        self.valid_time_threshold = self.get_parameter('valid_measurement_threshold').value 
        self.beacons_ids = self.get_parameter('beacons.ids').value 
        self.num_beacons = len(self.beacons_ids)
        self.beacons = {}
        for beacon_id in self.beacons_ids:
            position = self.get_parameter(f'beacons.{beacon_id}.position').value
            noise_std = self.get_parameter(f'beacons.{beacon_id}.noise_std').value
            self.beacons[beacon_id] = {"position" : position, "noise_std" : noise_std}                             

        # Modelo de predicción y medición:
        self.g, self.h_n, self.G, self.H_n = g_function, h_function_n, G_jacobian, H_jacobian_n

        # Matrices de ruido:
        self.Q_n = Q_noise_model_n                                                                                     # Ruido de medición
        self.R = R_noise_model(self.horizontal_vel, self.vertical_vel, 1.0 / self.filter_update_rate)                  # Ruido de proceso

        # Variables para la creencia de la localizacion en forma canónica
        self.omega = np.eye(3, dtype=np.float64)                           # Matriz de información
        self.xi    = np.array([[0],[0],[0]],dtype=np.float64)                   # Vector de información 
        self.mu    = np.array([[0],[0],[0]],dtype=np.float64)                      # vector media del estado estimado
        self.covariance = np.eye(3, dtype=np.float64)                  

        # Variables de resultado de predicción
        self.omega_pred = np.eye(3, dtype=np.float64)                           
        self.xi_pred    = np.array([[0],[0],[0]],dtype=np.float64)                   
        self.mu_pred    = np.array([[0],[0],[0]],dtype=np.float64)                     

        # Variables de suma para actualización
        self.omega_sum = np.zeros([3,3],dtype=np.float64)                           
        self.xi_sum    = np.array([[0],[0],[0]],dtype=np.float64)                      
    
        # Variables para la gestion de mediciones
        self.lock = Lock() # Para proteger el acceso a las mediciones
        self.calculations = []
        self.calculations_received = 0
        self.calculations_event = Event()

        # Subscriptores y publicadores
        self.predict_pub = self.create_publisher(PoseWithCovarianceStamped,f"/{self.get_name()}/predicted_position", self.num_beacons)     
        self.stat_pub = self.create_publisher(ProcessStats,f"/{self.get_name()}/process_stats",10)

        #if self.beacon_id != "":
        for beacon_id in self.beacons_ids:
            self.create_subscription(EIFOutput,f"/uwb_beacon/{beacon_id}/eif_output", self.partial_innovation_callback, 10)
        self.eif_output_pub =  self.create_publisher(EIFInput,f"eif_input",10)
            
        # Temporizador para la frecuencia de actualización
        self.timer = self.create_timer(1.0 / self.filter_update_rate, self.estimate_localization)

        self.get_logger().info("Nodo de filtro EIF descentralizado iniciado")

    def partial_innovation_callback(self, beacon_output_msg):
        self.get_logger().info('Resultado recibido')
        if self.calculations_event.is_set():
            self.get_logger().info('Resultado guardado')
            beacon_id = beacon_output_msg.id
            xi_n = beacon_output_msg.xi
            omega_n = beacon_output_msg.omega
            beacon_timestamp = Time.from_msg(beacon_output_msg.timestamp).nanoseconds

            with self.lock:
                self.calculations.append([xi_n, omega_n])

                self.calculations_received += 1
                if self.calculations_received == self.num_beacons:
                    self.calculations_event.clear()

            
    def publish_eif_input(self, mu, mu_pred) -> int:
        self.get_logger().info("Enviando mensaje de input")
        input_msg = EIFInput() 
        now = self.get_clock().now()
        input_msg.header.stamp = now.to_msg()
        input_msg.mu = mu.flatten().tolist()
        input_msg.mu_predicted = mu_pred.flatten().tolist()

        self.calculations_event.set()
        self.eif_output_pub.publish(input_msg)

        return now.nanoseconds

    def estimate_localization(self):
        self.get_logger().info("Estimando localización...")
        start_filter = self.get_clock().now()
        start = self.get_clock().now()
        self.predict()
        predic_time = (self.get_clock().now() - start).nanoseconds / 1e9

        # Mandar infomación a las valizas para que puedan realizar los calculos
        self.get_logger().info('Enviando petición a todas las balizad')
        start =  self.get_clock().now()
        last_broadcast = self.publish_eif_input(self.mu, self.mu_pred)
      
        while rclpy.ok() and self.calculations_event.is_set():
            now = self.get_clock().now().nanoseconds
            if now - last_broadcast > self.valid_time_threshold:
                self.get_logger().info("Timeout alcanzado")
                break
            rclpy.spin_once(self, timeout_sec=0.02)
        
        self.get_logger().info('Procesando resultados obtenidos')
        self.calculations_event.clear()
        with self.lock:
            innovation = self.calculations

        with self.lock :
            self.calculations = []
            self.calculations_received = 0

        if len(innovation) == 0:
            self.get_logger().warning("No hay medidas válidas disponibles, no es posible actualizar predicción")
        else:   
            self.get_logger().info(innovation)
            xi, omega = self.update(innovation)
        update_time = (self.get_clock().now() - start).nanoseconds / 1e9

        try:
            self.covariance = np.linalg.inv(self.omega)
        except np.linalg.LinAlgError:
            self.get_logger().warning("Singular matrix, filter cannot calculate covariance")
        
        
        with self.lock:
            self.publish_estimation(self.xi, self.covariance) # Publicar estimación de localización
        filter_time = (self.get_clock().now() - start_filter).nanoseconds / 1e9

        with self.lock:
            self.publish_stat(predic_time,update_time,filter_time,len(z),self.omega, self.xi, self.mu,self.ground_truth)

        return self.mu, self.omega, self.xi

    def predict(self):
        try:
            # Parte de predicción del filtro EIF
            # Calculo de la media de la estimación en t-1:
            self.mu = np.linalg.inv(self.omega) @ self.xi

            # Calculo prediciones de omega, xi y mu:
            G = self.G(self.mu) # Jacobiano de la función de predicción
            self.omega_pred = np.linalg.inv( G @ np.linalg.inv(self.omega) @ np.transpose(G) + self.R )     #Matriz de información predicha    
            self.mu_pred = self.g(self.mu)
            self.xi_pred = self.omega_pred @ self.mu_pred                                           #Vector de información predicho
        except np.linalg.LinAlgError:
            self.get_logger().warning("Singular matrix, filter cannot predict")
        return self.xi_pred, self.omega_pred, self.mu_pred

    def update(self, innovation):
        # Sumatorios de la actualización de matriz y vector de información con los calculos recibidos
        with self.lock:
            xi_calulations = np.array(innovation[:][0])
            omega_calculations = np.array(innovation[:][1])
            print(xi_calulations.shape)
            print(omega_calculations.shape)
            self.xi_sum = np.sum(xi_calulations,axis=0).reshape((3,1))
            self.ome_sum_sum = np.sum(omega_calculations,axis=0).reshape((3,3))

        # Actualizar la creencia de la localización
        self.omega = self.omega_pred + self.omega_sum
        self.xi = self.xi_pred + self.xi_sum

        # Resetear sumas
        self.xi_sum.fill(0)
        self.omega_sum.fill(0)                         

        return self.xi, self.omega

    def publish_estimation(self, xi, covariance):
        # Publicar la estimación de localización
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "global"

        print(covariance)
        mu = covariance @ xi

        pose_msg.pose.pose.position.x = mu[0][0]
        pose_msg.pose.pose.position.y = mu[1][0]
        pose_msg.pose.pose.position.z = mu[2][0]

        full_covariance = np.zeros((6,6))
        full_covariance[:3,:3] = covariance

        pose_msg.pose.covariance = full_covariance.flatten().tolist()

        self.predict_pub.publish(pose_msg)

    def ground_truth_callback(self,odom_msg):
        with self.lock:
            self.ground_truth = odom_msg.pose.pose.position


    def publish_stat(self,predict_time, update_time, filter_time, number_beacons, omega, xi, mu, gt):
        s = ProcessStats()
        s.header.stamp = self.get_clock().now().to_msg()

        #Prediccion:
        s.predicted_position.x = mu[0][0]
        s.predicted_position.y = mu[1][0]
        s.predicted_position.z = mu[2][0]

        #Ground_truth
        s.ground_truth.x = gt.x
        s.ground_truth.y = gt.y
        s.ground_truth.z = gt.z

        #Tiempos de ejecucion
        s.predict_time = predict_time
        s.update_time = update_time
        s.filter_time = filter_time

        # Numero de medidas/calculos recibidos
        s.measurements_received = number_beacons

        # Informacion:
        s.omega = omega.flatten().tolist()
        s.xi = xi.flatten().tolist()

        self.stat_pub.publish(s)


def main(args=None):
    rclpy.init(args=args)
    eif_filter_descentralized_node = EIFFilterDescentralizedNode()
    rclpy.spin(eif_filter_descentralized_node)
    eif_filter_descentralized_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()