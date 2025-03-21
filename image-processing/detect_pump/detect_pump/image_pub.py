#!/usr/bin/env python
from requests_toolbelt import ImproperBodyPartContentException
import rclpy
import cv2 # OpenCV library
import sys
import time
from rclpy.node import Node
from rclpy.parameter import Parameter
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from sensor_msgs.msg import Image

class ImagePubNode(Node):
    def __init__(self):
        super().__init__('image_pub', allow_undeclared_parameters = False,
                            automatically_declare_parameters_from_overrides = True)
        
        self.filename = self.get_parameter_or('filename', Parameter('', Parameter.Type.STRING, '')).value
        self.br = CvBridge() # Used to convert between ROS and OpenCV images
        
        self.start_node()
        
    def start_node(self):
        self.get_logger().info('image_pub node started')
        self.get_logger().info(f'Loading image from {self.filename}')
        if not self.filename or self.filename == '':
            self.get_logger().error('No file parameter found')
            return
        img = cv2.imread(str(self.filename))
        if img is None:
            self.get_logger().error(f'Could not read image from {self.filename}')
            return
        
        bridge = CvBridge()
        
        pub = self.create_publisher(Image, 'image', 10)
        angle = 0
        while rclpy.ok():
            rotImg = self.rotateImg(img, angle)
            imgMsg = bridge.cv2_to_imgmsg(rotImg, "bgr8")
            pub.publish(imgMsg)
            angle = (angle + 10) % 360
            time.sleep(1.0)  # Sleep for 1 second (1 Hz publication rate)
            
        #cv2.waitKey(2000)
        
    def rotateImg(self, img, angle):
        rows, cols, ch = img.shape
        M = cv2.getRotationMatrix2D((cols/2,rows/2),angle,1)
        return cv2.warpAffine(img,M,(cols,rows))
        
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ImagePubNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()