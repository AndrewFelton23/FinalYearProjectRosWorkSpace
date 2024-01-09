#!/usr/bin/python3

from tutorial_interfaces.srv import RobotCommands
from tutorial_interfaces.msg import Joints
from std_msgs.msg._string import String
from serial import Serial

import rclpy
from rclpy.node import Node

ser = Serial('/dev/ttyACM0', 9600)  # Change 'COM3' to your Arduino's serial port


class RobotArduinoNode(Node):
    def __init__(self):
        super().__init__('robot_arduino_node')
        self.srv = self.create_service(RobotCommands, 'robot_commands', self.robot_commands_callback)    
        self.get_logger().info('node has started') # CHANGE
        self.motor_positions = [0, 0, 0, 0]  # Initialize motor positions
        self.timer_period = 1.0  # Timer period in seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.pos_publisher = self.create_publisher(Joints,"/position_update",10)


    def robot_commands_callback(self, request, response):
        self.get_logger().info('Incoming request\ncommand: ' +  str(request.command)) # CHANGE
        com = str(request.command)
        ser.write(com.encode())
        response.completed = True 
        return response

    def timer_callback(self):
        # Read motor positions from the Arduino periodically
        self.motor_positions = self.read_motor_positions_from_arduino()
        


    def read_motor_positions_from_arduino(self):
        joints = Joints()
        try:
            ser.flushInput()  # Ensure the serial input buffer is clear
            ser.write(b'get_positions\n')  # Send a command to request motor positions
            response = ser.readline().decode().strip()  # Read and decode the response
            motor_positions = [int(pos) for pos in response.split(',')]  # Assuming positions are comma-separated integers
            joints.joint1 = motor_positions[0]
            joints.joint2 = motor_positions[1]
            joints.joint3 = motor_positions[2]
            joints.joint4 = motor_positions[3]
            return motor_positions
        except Exception as e:
            self.get_logger().error(f"Error reading motor positions: {str(e)}")
            joints.joint1 = 0
            joints.joint2 = 0
            joints.joint3 = 0
            joints.joint4 = 0
            return joints # Return default values in case of an error


def main(args=None):
    rclpy.init(args=args)

    robot_arduino_node = RobotArduinoNode()

    rclpy.spin(robot_arduino_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()