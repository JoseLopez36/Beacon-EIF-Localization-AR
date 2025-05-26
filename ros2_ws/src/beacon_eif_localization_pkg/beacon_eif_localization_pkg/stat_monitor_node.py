#!/usr/bin/env python

import rclpy
from rclpy.node import Node

# Mensajes
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from beacon_eif_localization_msgs.msg import ProcessStats

class StatsMonitorNode(Node):
    def __init__(self):
        super().__init__('stats_monitor_node')
        self.get_logger().info("Iniciando nodo de manejo de estadisticas...")

        # Subscriptores
        self.odometry_sub = self.create_subscription(Odometry, '/ground_truth/vehicle_odom', self.odometry_callback, 10)
        self.predict_sub = self.create_subscription(PoseStamped,"/predicted_position",self.prediction_callback, 10)
        self.stats_sub = self.create_subscription(ProcessStats,"/process_stats",self.stats_callback, 10)

        # Publicadores
        self.stats_pub = self.create_publisher()
        

    # Subcribers callbacks:
    def odometry_callback(self):
        pass
        
    def prediction_callback(self):
        pass

    def stats_callback(self):
        pass

    def publish(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    stats_monitor_node = StatsMonitorNode()
    rclpy.spin(stats_monitor_node)
    stats_monitor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()