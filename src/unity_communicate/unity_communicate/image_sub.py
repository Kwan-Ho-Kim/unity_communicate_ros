#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridgeError
import numpy as np
import cv2


class image_sub(Node):
    def __init__(self):
        super().__init__('image_sub')
        qos = QoSProfile(depth=100)
        self.create_subscription(CompressedImage, '/compressed_image', self.image_CB, qos_profile=qos)
        # self.create_subscription(CompressedImage, '/image/compressed', self.image_CB, qos_profile=qos)
        
        
    def image_CB(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            im0 = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
        except CvBridgeError as e:
            print(e)
            

        cv2.imshow("window",im0)
        cv2.waitKey(1)
        
        
def main(args=None):
    rclpy.init(args=args)
    
    img_sub = image_sub() 
        
    rclpy.spin(img_sub)
        
        
if __name__ == '__main__':
    main()