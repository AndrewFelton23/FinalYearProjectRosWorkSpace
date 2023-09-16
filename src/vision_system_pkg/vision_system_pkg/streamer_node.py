#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg._compressed_image import CompressedImage
import cv2
from cv_bridge import CvBridge
import time

class StreamerNode(Node):
    def __init__(self):
        super().__init__('streamer_node')
        #declare the communication between ros and CV2
        self.bridge_ = CvBridge()
        self.publisher_ = self.create_publisher(CompressedImage, 'video_data', 10)     
        self.get_logger().info("streamer node has started")

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
            time.sleep(0.2)
        self.capture_.release()

def main(args=None):
    rclpy.init(args=args)
    streamer_node = StreamerNode()
    streamer_node.publish_vidData()
    rclpy.spin(streamer_node)
    streamer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
