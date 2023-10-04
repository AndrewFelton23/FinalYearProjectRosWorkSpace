#!/usr/bin/python3

from tutorial_interfaces.srv import AddThreeInts     # CHANGE
from tutorial_interfaces.srv import ConveyorCommands     # CHANGE
from std_msgs.msg._string import String
from serial import Serial
import time

import rclpy
from rclpy.node import Node

# Define the serial port and baud rate
ser = Serial('/dev/ttyUSB0', 9600)  # Change 'COM3' to your Arduino's serial port


class ConveyorArduinoNode(Node):
    def __init__(self):
        super().__init__('conveyor_arduino_node')
        self.srv = self.create_service(ConveyorCommands, 'conveyor_commands', self.conveyor_commands_callback)        # CHANGE
        self.get_logger().info('node has started') # CHANGE

    def conveyor_commands_callback(self, request, response):
        self.get_logger().info('Incoming request\ncommand: ' +  str(request.command)) # CHANGE
        com = str(request.command)
        ser.write(com.encode())
        response.completed = True 
        return response

def main(args=None):
    rclpy.init(args=args)

    conveyor_arduino_node = ConveyorArduinoNode()

    rclpy.spin(conveyor_arduino_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()