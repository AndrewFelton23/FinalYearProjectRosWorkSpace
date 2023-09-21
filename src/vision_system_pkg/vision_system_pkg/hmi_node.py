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

class HMINode(Node):
    def __init__(self):
        super().__init__('hmi_node')
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
            time.sleep(5.0)  # Wait for the next update

@app.route('/')
def index():
    return render_template('/index.html')

@app.route('/get_image')
def get_image():
    with image_mutex:
        if image_data is not None:
            return Response(image_data, mimetype='image/jpeg')
        else:
            return jsonify({'error': 'Failed to capture image'})

# Define a route to receive commands from the web interface
@app.route('/update_led_color', methods=['POST'])
def update_led_color():
    # Get the color information from the request's JSON payload
    data = request.get_json()
    started = data.get('start')
    print("Start sequence: "+ str(started))

    # Perform any required actions with the LED color
    # For example, you can store it in a database or trigger a physical LED to change color

    # You can also send a response back to the frontend if needed
    # For this example, we'll simply send a success response
    return jsonify({'message': 'LED color updated successfully'})

@app.route('/manual_mode', methods=['POST'])
def manual_mode():
    # Get the color information from the request's JSON payload
    Manualdata = request.get_json()
    mode = Manualdata.get('manualMode')
    print("Manual mode: " + str(mode))

    # Perform any required actions with the LED color
    # For example, you can store it in a database or trigger a physical LED to change color

    # You can also send a response back to the frontend if needed
    # For this example, we'll simply send a success response
    return jsonify({'message': 'LED color updated successfully'})

@app.route('/get_coordinates')
def get_coordinates():
    return jsonify({'joint1': 'Joint 1 position',
                    'joint2': 'Joint 2 position',
                    'joint3': 'Joint 3 position',
                    'joint4': 'Joint 4 position',
                    'gripper': 'Gripper Open or Closed'
                    })


def run_ros_node(args=None):
    rclpy.init(args=args)
    hmi_node = HMINode()
    rclpy.spin(hmi_node)
    hmi_node.destroy_node()
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