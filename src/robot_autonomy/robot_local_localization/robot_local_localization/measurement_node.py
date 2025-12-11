#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class MeasurementNode(Node):
    def __init__(self):
        super().__init__('measurement_node')
        
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/zed/zed_node/imu/data_raw',
            self.imu_callback,
            10
        )
        
        
        self.vo_sub = self.create_subscription(
            Odometry,
            '/rtabmap/odom',  
            self.vo_callback,
            10
        )
        
        
        self.measurement_pub = self.create_publisher(
            Odometry,
            '/measurement_model/odom',
            10
        )
        
        self.latest_imu = None
        self.latest_vo = None
        
        self.get_logger().info('Measurement Node started')
    
    def imu_callback(self, msg):
        self.latest_imu = msg
        self.publish_combined()
    
    def vo_callback(self, msg):
        self.latest_vo = msg
        self.publish_combined()
    
    def publish_combined(self):
        if self.latest_imu is None or self.latest_vo is None:
            return
        
        
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        
        odom.pose.pose.position = self.latest_vo.pose.pose.position
        
        
        odom.pose.pose.orientation = self.latest_imu.orientation
        
        
        odom.twist.twist = self.latest_vo.twist.twist
        
        self.measurement_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = MeasurementNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
