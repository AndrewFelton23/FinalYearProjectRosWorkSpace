#!/usr/bin/python3

from tutorial_interfaces.srv import RobotCommands
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

    def robot_commands_callback(self, request, response):
        self.get_logger().info('Incoming request\ncommand: ' +  str(request.command)) # CHANGE
        com = str(request.command)
        ser.write(com.encode())
        response.completed = True 
        return response


def main(args=None):
    rclpy.init(args=args)

    robot_arduino_node = RobotArduinoNode()

    rclpy.spin(robot_arduino_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()