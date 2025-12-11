#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import math

class PredictionNode(Node):
    def __init__(self):
        super().__init__('prediction_node')
        
        
        self.declare_parameter('wheel_separation', 0.46)
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        
        
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        
        self.last_time = self.get_clock().now()
        
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )
        
        
        self.odom_pub = self.create_publisher(Odometry, '/motion_model/odom', 10)
        
        self.get_logger().info('Prediction Node started')
        self.get_logger().info(f'Wheel separation: {self.wheel_separation}m')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius}m')
    
    def cmd_vel_callback(self, msg):
        
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        
        v = msg.linear.x
        w = msg.angular.z
        
        
        
        
        
        
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt
        
        
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        
        self.odom_pub.publish(odom)
        
        self.get_logger().debug(f'Motion Model: x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = PredictionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
