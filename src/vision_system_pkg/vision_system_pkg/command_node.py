#!/usr/bin/python3

from tutorial_interfaces.srv import AddThreeInts       # CHANGE
from tutorial_interfaces.srv import ConveyorCommands       # CHANGE
from std_msgs.msg._string import String
import sys
import rclpy
from rclpy.node import Node


class CommandNode(Node):

    def __init__(self):
        super().__init__('command_node')
        self.cli = self.create_client(ConveyorCommands, 'conveyor_commands')       
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ConveyorCommands.Request()              
        self.subscriber_ = self.create_subscription(String,
            'hmi_button_command',self.button_command_callback,10)         

    def send_command(self,com):
        '''Command client function to send a command'''
        self.req.command = com
        self.future = self.cli.call_async(self.req)

    def button_command_callback(self,command):
        '''hmi_button_command Subscriber callback function'''
        self.get_logger().info("instruction recieved: " + str(command))
        self.send_command(command)


def main(args=None):
    rclpy.init(args=args)

    command_node = CommandNode()
    command_node.send_command()
    while rclpy.ok():
        rclpy.spin_once(command_node)
        if command_node.future.done():
            try:
                response = command_node.future.result()
            except Exception as e:
                command_node.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                command_node.get_logger().info(
                    'Result of conveyor_commands: ' +                               # CHANGE
                    str(response.completed)) # CHANGE
            break

    command_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()