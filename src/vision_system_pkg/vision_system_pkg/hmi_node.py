#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg._string import String
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
            "/image",self.image_callback,10)
        #create conveyor start publisher
        self.start_pub = self.create_publisher(String,
            'hmi_button_command', 10)
        #create manual mode publisher
        self.manual_pub = self.create_publisher(String,
            'robot_command', 10)
        self.timer_period = 1  # Check every 1 second
        self.timer = self.create_timer(self.timer_period, self.publish_commands_sequence)
        self.start_sequence = None  
        self.manual_mode = None
        self.auto_mode = None
        self.manual_command = None


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

    def publish_commands_sequence(self):
        if self.start_sequence is not None:  
            msg = String()
            if self.start_sequence == True:
                msg.data = 'F'
            else:
                msg.data = 'S'
            self.start_pub.publish(msg)
            self.get_logger().info("Published on start_pub: " + msg.data)
            self.start_sequence = None
        if self.manual_mode is not None:  
            if self.manual_mode == True:  
                command = String()
                if self.manual_command is not None:  
                    command.data = self.manual_command
                    self.manual_pub.publish(command)
                    self.get_logger().info("Published on manual_pub: " + command.data)
                    self.manual_command = None
            else:
                self.manual_command = None
                self.manual_mode = None
        # if self.auto_mode is not None:  
        #     msg = String()
        #     if self.auto_mode == True:
        #         msg.data = 'True' 
        #     else:
        #         msg.data = 'False'
        #     self.auto_pub.publish(msg)
        #     self.get_logger().info("Published on auto_pub: " + msg.data)
        #     self.auto_mode = None

    def start_image_timer(self):
        self.image_timer = threading.Timer(1.0, self.update_image_data)  # Update image every 1 seconds
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
@app.route('/start_conveyor', methods=['POST'])
def start_conveyor():
    # Get the start information from the request's JSON payload
    data = request.get_json()
    started = data.get('start')
    print("Start sequence: " + str(started))
    # Set the start_sequence variable in the ROS node
    hmi_node_instance.start_sequence = started  # Assuming 'hmi_node_instance' is your ROS node instance
    return jsonify({'message': 'LED color updated successfully'})

@app.route('/manual_mode', methods=['POST'])
def manual_mode():
    # Get the color information from the request's JSON payload
    Manualdata = request.get_json()
    mode = Manualdata.get('manualMode')
    print("Manual mode: " + str(mode))
    hmi_node_instance.manual_mode = mode
    return jsonify({'message': 'LED color updated successfully'})

@app.route('/auto_mode', methods=['POST'])
def auto_mode():
    # Get the color information from the request's JSON payload
    Autodata = request.get_json()
    mode = Autodata.get('autoMode')
    print("Auto mode: " + str(mode))
    hmi_node_instance.auto_mode = mode
    return jsonify({'message': 'LED color updated successfully'})

@app.route('/send_robot_command', methods=['POST'])
def send_command():
    command = request.form.get('command')
    print("Received command:", command)
    # Handle the command as needed
    hmi_node_instance.manual_command = command
    return 'Command received'

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
    global hmi_node_instance
    hmi_node_instance = HMINode()
    rclpy.spin(hmi_node_instance)
    hmi_node_instance.destroy_node()
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