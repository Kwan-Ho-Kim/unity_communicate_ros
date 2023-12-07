#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PathPublisher(Node):
    def __init__(self):
        super().__init__('global_path_pub')
        qos = QoSProfile(depth=10)
        self.path_publisher = self.create_publisher(Path, '/global_path',  qos_profile=qos)        
        self.create_timer(0.1, self.publish_path)
        self.global_path = Path()
        self.file_path='/root/ros2_ws/src/unity_communicate/resource/test_path.txt'
        
        self.read_path()
        
    def publish_path(self):
        self.path_publisher.publish(self.global_path)
        
    def read_path(self):    # txt file : x, y, z, rx, ry, rz, rw
        self.global_path.header.frame_id = "map"
        
        with open(self.file_path, "r") as f:
            lines = f.readlines()
            
        for line in lines:
            line = line.replace('\n',',')
            pose = line.split(',')
            
            pose_stamp = PoseStamped()
            pose_stamp.header.frame_id = '/map'
            pose_stamp.pose.position.x = float(pose[0])
            pose_stamp.pose.position.y = float(pose[1])
            pose_stamp.pose.position.z = float(pose[2])
            pose_stamp.pose.orientation.x = float(pose[3])
            pose_stamp.pose.orientation.y = float(pose[4])
            pose_stamp.pose.orientation.z = float(pose[5])
            pose_stamp.pose.orientation.w = float(pose[6])
            
            self.global_path.poses.append(pose_stamp)
                
def main(args=None):
    rclpy.init(args=args)
    
    path_publisher = PathPublisher() 
        
    rclpy.spin(path_publisher)
        
if __name__ == '__main__':
    main()