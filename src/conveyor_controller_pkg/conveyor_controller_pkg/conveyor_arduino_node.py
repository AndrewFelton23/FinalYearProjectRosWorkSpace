#!/usr/bin/python3

from tutorial_interfaces.srv import AddThreeInts     # CHANGE
from tutorial_interfaces.srv import ConveyorCommands     # CHANGE
from std_msgs.msg._string import String

import rclpy
from rclpy.node import Node


class ConveyorArduinoNode(Node):
    def __init__(self):
        super().__init__('conveyor_arduino_node')
        self.srv = self.create_service(AddThreeInts, 'add_three_ints', self.add_three_ints_callback)        # CHANGE
        self.srv = self.create_service(ConveyorCommands, 'conveyor_commands', self.conveyor_commands_callback)        # CHANGE

    def add_three_ints_callback(self, request, response):
        response.sum = request.a + request.b + request.c                                                  # CHANGE
        self.get_logger().info('Incoming request\na: %d b: %d c: %d' % (request.a, request.b, request.c)) # CHANGE

        return response

    def conveyor_commands_callback(self, request, response):
        response.completed = True                                         # CHANGE
        self.get_logger().info('Incoming request\ncommand: ' +  str(request.command)) # CHANGE

        return response

def main(args=None):
    rclpy.init(args=args)

    conveyor_arduino_node = ConveyorArduinoNode()

    rclpy.spin(conveyor_arduino_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()