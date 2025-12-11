#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import math

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.real_path_pub = self.create_publisher(Path, '/real_path', 10)
        self.motion_path_pub = self.create_publisher(Path, '/motion_path', 10)
        self.measurement_path_pub = self.create_publisher(Path, '/measurement_path', 10)
        self.ekf_path_pub = self.create_publisher(Path, '/ekf_path', 10)
        
        
        self.real_path = Path()
        self.motion_path = Path()
        self.measurement_path = Path()
        self.ekf_path = Path()
        
        
        self.create_subscription(Odometry, '/wheel_encoder/odom', self.real_odom_callback, 10)
        self.create_subscription(Odometry, '/motion_model/odom', self.motion_callback, 10)
        self.create_subscription(Odometry, '/measurement_model/odom', self.measurement_callback, 10)
        self.create_subscription(Odometry, '/ekf/odom', self.ekf_callback, 10)
        
        
        self.timer = self.create_timer(0.1, self.execute_rectangle)
        self.step = 0
        self.step_duration = 300  
        
        self.get_logger().info('Test Node started - following rectangular path')
    
    def real_odom_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.real_path.poses.append(pose)
        self.real_path.header = msg.header
        self.real_path_pub.publish(self.real_path)
    
    def motion_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.motion_path.poses.append(pose)
        self.motion_path.header = msg.header
        self.motion_path_pub.publish(self.motion_path)
    
    def measurement_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.measurement_path.poses.append(pose)
        self.measurement_path.header = msg.header
        self.measurement_path_pub.publish(self.measurement_path)
    
    def ekf_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.ekf_path.poses.append(pose)
        self.ekf_path.header = msg.header
        self.ekf_path_pub.publish(self.ekf_path)
    
    def execute_rectangle(self):
        cmd = Twist()
        
        phase = self.step // self.step_duration
        
        if phase == 0:
            
            cmd.linear.x = 0.3
            self.get_logger().info('Moving forward', once=True)
        elif phase == 1:
            
            cmd.angular.z = 0.5
            self.get_logger().info('Turning left', once=True)
        elif phase == 2:
            
            cmd.linear.x = 0.3
        elif phase == 3:
            
            cmd.angular.z = 0.5
        elif phase == 4:
            
            cmd.linear.x = 0.3
        elif phase == 5:
            
            cmd.angular.z = 0.5
        elif phase == 6:
            
            cmd.linear.x = 0.3
        elif phase == 7:
            
            cmd.angular.z = 0.5
        else:
            
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('Rectangle complete!')
        
        self.cmd_pub.publish(cmd)
        self.step += 1

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
