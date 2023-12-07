#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import rospkg

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path




class PathRecoder(Node):
    def __init__(self):
        super().__init__('path_recoder')
        qos = QoSProfile(depth=10)
        self.create_subscription(TFMessage, '/tf', self.tf_CB, qos_profile=qos)        
        
        # rospack=rospkg.RosPack()
        # pkg_path=rospack.get_path('unity_communicate')
        self.file_path='/root/ros2_ws/src/unity_communicate/resource/test_path.txt'
        
        f = open(self.file_path, "w")
        f.write("")
        f.close()
        
        self.save_freq = 1
        self.prev_time = 0
        
        
        
    def tf_CB(self, msg):
        transform = TransformStamped()
        
        for tf in msg.transforms:
            if tf.child_frame_id == "base_link":
                transform = tf
            
        deltatime = transform.header.stamp.sec - self.prev_time
            
        if self.save_freq > deltatime:
            self.write_pose(transform)
            
        self.prev_time = transform.header.stamp.sec
        
    def write_pose(self, transform):    # txt file : x, y, z, rx, ry, rz, rw
        
        pose = (f"{transform.transform.translation.x}, "
                f"{transform.transform.translation.y}, "
                f"{transform.transform.translation.z}, "
                f"{transform.transform.rotation.x}, "
                f"{transform.transform.rotation.y}, "
                f"{transform.transform.rotation.z}, "
                f"{transform.transform.rotation.w} \n")
        
        with open(self.file_path, 'a') as f:
            f.write(pose)
        
def main(args=None):
    rclpy.init(args=args)
    
    path_recoder = PathRecoder() 
        
    rclpy.spin(path_recoder)
        
        
if __name__ == '__main__':
    main()