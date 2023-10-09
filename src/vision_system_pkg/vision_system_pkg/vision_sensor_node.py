#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tutorial_interfaces.msg import Vision
import cv2
from cv_bridge import CvBridge
import pickle

from vision_system_pkg.utils import *

# Load camera calibration matrices
calibration_data = pickle.load(open("/home/rock/ros2_ws/src/vision_system_pkg/vision_system_pkg/calibration/calibration.pkl", "rb"))
cameraMatrix, dist = calibration_data

frameWidth = 640
frameHeight = 480

# Physical measurements
camera_width_cm = 13
camera_height_cm = 9.5
max_object_width_cm = 8  # For example, setting a maximum object width of 8 cm
ratio_w = camera_width_cm/frameWidth
ratio_h = camera_height_cm/frameHeight


class VisionSensorNode(Node):
    def __init__(self):
        super().__init__('vision_sensor_node')
        self.bridge_ = CvBridge()
        #create subscriber
        self.subscriber_ = self.create_subscription(Image,
            '/image',self.vision_callback,10)
        #create the publisher
        self.publisher_ = self.create_publisher(Vision, '/vision', 10) 


    def vision_callback(self, img):
        '''video_data Subscriber callback function for vision system'''
        self.get_logger().info("Received new image from ROS 2")
        # convert the image back to a readable cv file
        frame = self.bridge_.imgmsg_to_cv2(img, "passthrough")
        # Undistort the input image
        img = cv2.undistort(frame, cameraMatrix, dist, None, cameraMatrix)
        imgContour = img.copy()
        imgOrientation = img.copy()
        imgBlur = cv2.GaussianBlur(img,(7,7),1)
        imgGray = cv2.cvtColor(imgBlur,cv2.COLOR_BGR2GRAY)
        threshold1 = 85
        threshold2 = 80
        minArea = cv2.getTrackbarPos("Area","Parameters")
        maxArea = 12000
        imgCanny = cv2.Canny(imgGray,threshold1,threshold2)
        kernel = np.ones((5,5))
        imgDil = cv2.dilate(imgCanny,kernel,iterations=1)

        center, angle, x, y = getContours(imgDil,imgContour,minArea,maxArea,imgOrientation)
        if (y != None) :
            if y < 240:
                y_distance = 240 -y
                y_distance_cm = (y_distance)*ratio_h
                x_distance = 320 -x
                x_distance_cm = (x_distance)*ratio_w
                print("stop conveyor")
                print("distance to y : " + str(y_distance_cm))
                print("distance to x : " + str(x_distance_cm))
                msg = Vision()
                msg.angle = angle
                msg.x = x_distance_cm
                msg.y = y_distance_cm
                self.publisher_.publish(msg)
                self.get_logger().info("Published info")


        
def main(args=None):
    rclpy.init(args=args)
    vision_sensor_node = VisionSensorNode()
    rclpy.spin(vision_sensor_node)
    vision_sensor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
