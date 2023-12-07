#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import rospkg

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class toOdom(Node):
    def __init__(self):
        super().__init__('toOdom')
        qos = QoSProfile(depth=10)
        self.create_subscription(TFMessage, '/tf', self.tf_CB, qos_profile=qos)
        self.create_subscription(Imu, '/imu/raw_data', self.imu_CB, qos_profile=qos) 
        
        self.odom_pub = self.create_publisher(Odometry, "/odom", qos_profile=qos)    
        
        self.is_tf = False
        self.is_imu = False
        
        self.odom_msg = Odometry()
        self.odom_msg.child_frame_id = "base_link"
        
        self.timer = self.create_timer(0.1, self.publish_odom)
                
    def publish_odom(self):
        if self.is_tf and self.is_imu:
            self.odom_pub.publish(self.odom_pub)
        
    def tf_CB(self, msg):
        transform = TransformStamped()
        
        for tf in msg.transforms:
            if tf.child_frame_id == "base_link":
                transform = tf
            
        self.odom_msg.pose.pose.position.x = transform.transform.translation.x
        self.odom_msg.pose.pose.position.y = transform.transform.translation.y
        self.odom_msg.pose.pose.position.z = transform.transform.translation.z
        
        self.odom_msg.pose.pose.orientation.x = transform.transform.rotation.x
        self.odom_msg.pose.pose.orientation.y = transform.transform.rotation.y
        self.odom_msg.pose.pose.orientation.z = transform.transform.rotation.z
        self.odom_msg.pose.pose.orientation.w = transform.transform.rotation.w
        
        self.is_tf = True
        
    def imu_CB(self, msg):
        
        self.is_imu = True
        
        
def main(args=None):
    rclpy.init(args=args)
    
    odometry = toOdom() 
        
    rclpy.spin(odometry)
        
        
if __name__ == '__main__':
    main()