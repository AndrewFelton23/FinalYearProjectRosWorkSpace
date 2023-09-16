#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg._compressed_image import CompressedImage
import cv2
from cv_bridge import CvBridge
from utils import *

class StackNode(Node):
    def __init__(self):
        super().__init__('streamer_node')
        #declare the communication between ros and CV2
        self.bridge_ = CvBridge()
        self.subscriber_ = self.create_subscription(CompressedImage,
            'video_data',self.stack_image_callback,10)
        self.publisher_ = self.create_publisher(CompressedImage, 'stack_data', 10)        
        self.get_logger().info("streamer node has started")


    def stack_image_callback(self, img):
        '''video_data Subscriber callback function'''
        self.get_logger().info("Received new image from ROS 2")
        # convert the image back to a readable cv file
        frame = self.bridge_.compressed_imgmsg_to_cv2(img, "passthrough")
        # change the format of the frame
        ret, buffer = cv2.imencode('.jpg', frame)
        imgContour = img.copy()
        imgOrientation = img.copy()
        imgBlur = cv2.GaussianBlur(img,(7,7),1)
        imgGray = cv2.cvtColor(imgBlur,cv2.COLOR_BGR2GRAY)
        threshold1 = cv2.getTrackbarPos("Threshold1","Parameters")
        threshold2 = cv2.getTrackbarPos("Threshold2","Parameters")
        minArea = cv2.getTrackbarPos("Area","Parameters")

        imgCanny = cv2.Canny(imgGray,threshold1,threshold2)
        kernel = np.ones((5,5))
        imgDil = cv2.dilate(imgCanny,kernel,iterations=1)

        getContours(imgDil,imgContour,minArea,imgOrientation)

        imgStack = stackImages(0.8,([img,imgGray,imgCanny],[img,imgContour, imgOrientation]))
        global image_data
        with image_mutex:
            image_data = buffer.tobytes()
        self.latest_image = image_data
        self.publisher_.publish(self.bridge_.cv2_to_compressed_imgmsg(imgStack,"jpg"))
        self.get_logger().info("published image")

    def publish_vidData(self):
        # Load an image using cv2
        self.capture_ = cv2.VideoCapture("/dev/video5")  # Use index 2 for your webcam
        if not self.capture_.isOpened():
            self.get_logger().info("Publisher unable to open image")
            exit()
        while (True):
            #capture the current frame
            success,frame = self.capture_.read()
            #check if the current frame was read correctly
            if not success:
                self.get_logger().info("Publisher unable to recieve frame (possibly end of stream)")
                break
            #publish the frames
            self.publisher_.publish(self.bridge_.cv2_to_compressed_imgmsg(frame,"jpg"))
            self.get_logger().info("published image")
        self.capture_.release()

def main(args=None):
    rclpy.init(args=args)
    stack_node = StackNode()
    stack_node.publish_vidData()
    rclpy.spin(stack_node)
    stack_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
