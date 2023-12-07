#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path


class FootprintMaker(Node):
    def __init__(self):
        super().__init__('image_sub')
        qos = QoSProfile(depth=10)
        self.create_subscription(TFMessage, '/tf', self.tf_CB, qos_profile=qos)
        self.path_pub = self.create_publisher(Path, 'path', qos)
        self.path_msg = Path() 
        
        self.save_freq = 1
        self.prev_time = 0
        
        
        
    def tf_CB(self, msg):
        transform = TransformStamped()
        
        for tf in msg.transforms:
            if tf.child_frame_id == "base_link":
                transform = tf
            
        deltatime = transform.header.stamp.sec - self.prev_time
            
        if self.save_freq > deltatime:
            self.append_pose(transform)
            
        self.prev_time = transform.header.stamp.sec
        
    def append_pose(self, transform):
        pose = PoseStamped()
        
        pose.header = transform.header
        
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        
        pose.pose.orientation.x = transform.transform.rotation.x
        pose.pose.orientation.y = transform.transform.rotation.y
        pose.pose.orientation.z = transform.transform.rotation.z
        pose.pose.orientation.w = transform.transform.rotation.w
        
        self.path_msg.header.frame_id = "map"
        self.path_msg.poses.append(pose)
        
        self.path_pub.publish(self.path_msg)
        
        
def main(args=None):
    rclpy.init(args=args)
    
    footprint_maker = FootprintMaker() 
        
    rclpy.spin(footprint_maker)
        
        
if __name__ == '__main__':
    main()