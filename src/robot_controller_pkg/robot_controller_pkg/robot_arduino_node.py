#!/usr/bin/python3

from tutorial_interfaces.srv import RobotCommands     # CHANGE
from std_msgs.msg._string import String

import rclpy
from rclpy.node import Node


class RobotArduinoNode(Node):
    def __init__(self):
        super().__init__('robot_arduino_node')
        self.srv = self.create_service(RobotCommands, 'robot_commands', self.robot_commands_callback)        # CHANGE
        self.get_logger().info('node has started') # CHANGE

    def robot_commands_callback(self, request, response):
        self.get_logger().info('Incoming request\ncommand: ' +  str(request.command)) # CHANGE
        response.completed = True 
        return response

def main(args=None):
    rclpy.init(args=args)

    robot_arduino_node = RobotArduinoNode()

    rclpy.spin(robot_arduino_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()