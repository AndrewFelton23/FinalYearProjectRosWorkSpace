#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from flask import Flask, Response, render_template, request, jsonify
import concurrent.futures
import threading
import time


app = Flask(__name__)

image_data = None
image_mutex = threading.Lock()


class FlaskNode(Node):
    def __init__(self):
        super().__init__('flask_node')
        self.bridge_ = CvBridge()
        #create subscriber
        self.subscriber_ = self.create_subscription(Image,
            '/image/vision',self.image_callback,10)

    def image_callback(self, img):
        '''video_data Subscriber callback function'''
        self.get_logger().info("Received new image from ROS 2")
        # convert the image back to a readable cv file
        frame = self.bridge_.imgmsg_to_cv2(img, "passthrough")
        # change the format of the frame
        ret, buffer = cv2.imencode('.jpg', frame)
        
        global image_data
        with image_mutex:
            image_data = buffer.tobytes()
        self.latest_image = image_data

    def start_image_timer(self):
        self.image_timer = threading.Timer(1.0, self.update_image_data)  # Update image every 5 seconds
        self.image_timer.daemon = True
        self.image_timer.start()

    def update_image_data(self):
        while True:
            with image_mutex:
                self.latest_image = image_data
            time.sleep(50.0)  # Wait for the next update

        

@app.route('/')
def index():
    return render_template('/index.html')

@app.route('/get_image', methods=['GET'])
def get_image():
    with image_mutex:
        if image_data is not None:
            return Response(image_data, mimetype='image/jpeg')
        else:
            return jsonify({'error': 'Failed to capture image'})


def run_ros_node(args=None):
    rclpy.init(args=args)
    flask_node = FlaskNode()
    rclpy.spin(flask_node)
    flask_node.destroy_node()
    rclpy.shutdown()

def run_flask_app():
    app.run(host='0.0.0.0', port=5000)

def main(args=None):
    with concurrent.futures.ThreadPoolExecutor() as executor:
        ros_future = executor.submit(run_ros_node)
        flask_future = executor.submit(run_flask_app)
        # Wait for both threads to complete
    ros_future.result()
    flask_future.result()

if __name__ == '__main__':
    main()
