#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')
        
        
        self.state = np.zeros(3)
        
        
        self.P = np.eye(3) * 0.1
        
        
        self.Q = np.diag([0.1, 0.1, 0.05])
        
        
        self.R = np.diag([0.2, 0.2, 0.1])
        
        
        self.motion_sub = self.create_subscription(
            Odometry,
            '/motion_model/odom',
            self.motion_callback,
            10
        )
        
        
        self.measurement_sub = self.create_subscription(
            Odometry,
            '/measurement_model/odom',
            self.measurement_callback,
            10
        )
        
        
        self.ekf_pub = self.create_publisher(Odometry, '/ekf/odom', 10)
        
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('EKF Node started')
    
    def motion_callback(self, msg):
        
        x_pred = msg.pose.pose.position.x
        y_pred = msg.pose.pose.position.y
        
        
        q = msg.pose.pose.orientation
        theta_pred = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        
        
        self.state = np.array([x_pred, y_pred, theta_pred])
        
        
        self.P = self.P + self.Q
    
    def measurement_callback(self, msg):
        
        x_meas = msg.pose.pose.position.x
        y_meas = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        theta_meas = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        
        z = np.array([x_meas, y_meas, theta_meas])
        
        
        y = z - self.state
        y[2] = math.atan2(math.sin(y[2]), math.cos(y[2]))  
        
        
        S = self.P + self.R
        K = self.P @ np.linalg.inv(S)
        
        
        self.state = self.state + K @ y
        
        
        self.P = (np.eye(3) - K) @ self.P
        
        
        self.publish_ekf()
    
    def publish_ekf(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.state[0]
        odom.pose.pose.position.y = self.state[1]
        
        
        odom.pose.pose.orientation.z = math.sin(self.state[2] / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.state[2] / 2.0)
        
        self.ekf_pub.publish(odom)
        
        
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = 'base_link_ekf'
        t.transform.translation.x = self.state[0]
        t.transform.translation.y = self.state[1]
        t.transform.rotation = odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
