#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__("image_publisher")
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture("/dev/video5")
        self.pub = self.create_publisher(Image, "/image", 10)

    def run(self):
        while True:
            try:
                r, frame = self.cap.read()
                if not r:
                    return
                self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
                time.sleep(1)
            except CvBridgeError as e:
                print(e)
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)

    image_publisher_node = ImagePublisherNode()
    print("Publishing...")
    image_publisher_node.run()

    image_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()