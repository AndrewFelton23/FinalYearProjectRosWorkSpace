#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

from vision_system_pkg.utils import *


class VisionSensorNode(Node):
    def __init__(self):
        super().__init__('vision_sensor_node')
        self.bridge_ = CvBridge()
        #create subscriber
        self.subscriber_ = self.create_subscription(Image,
            '/image',self.vision_callback,10)
        #create the publisher
        self.publisher_ = self.create_publisher(Image, '/image/vision', 10) 


    def vision_callback(self, img):
        '''video_data Subscriber callback function for vision system'''
        self.get_logger().info("Received new image from ROS 2")
        # convert the image back to a readable cv file
        frame = self.bridge_.imgmsg_to_cv2(img, "passthrough")
        #run vision system on data
        imgContour = frame.copy()
        imgOrientation = frame.copy()
        imgBlur = cv2.GaussianBlur(frame,(7,7),1)
        imgGray = cv2.cvtColor(imgBlur,cv2.COLOR_BGR2GRAY)
        threshold1 = 45
        threshold2 = 50
        minArea = 5000
        imgCanny = cv2.Canny(imgGray,threshold1,threshold2)
        kernel = np.ones((5,5))
        imgDil = cv2.dilate(imgCanny,kernel,iterations=1)

        kernel = np.ones((5,5))
        imgDil = cv2.dilate(imgCanny,kernel,iterations=1)

        angle_deg = getContours(imgDil,imgContour,minArea,imgOrientation)


        imgStack = stackImages(0.8,([frame,imgGray,imgCanny],[frame,imgContour, imgOrientation]))
        resized_imgStack = cv2.resize(imgStack, (240, 120))
        
        self.publisher_.publish(self.bridge_.cv2_to_imgmsg(imgOrientation,"bgr8"))


        
def main(args=None):
    rclpy.init(args=args)
    vision_sensor_node = VisionSensorNode()
    rclpy.spin(vision_sensor_node)
    vision_sensor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
