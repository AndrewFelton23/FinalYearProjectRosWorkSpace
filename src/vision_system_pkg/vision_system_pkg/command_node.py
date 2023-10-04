#!/usr/bin/python3

from tutorial_interfaces.srv import ConveyorCommands       # CHANGE
from tutorial_interfaces.srv import RobotCommands       # CHANGE
from std_msgs.msg._string import String
import sys
import rclpy
from rclpy.node import Node
from functools import partial


class CommandNode(Node):

    def __init__(self):
        super().__init__('command_node')
        self.cli = self.create_client(ConveyorCommands, 'conveyor_commands')       
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.cli_robot = self.create_client(ConveyorCommands, 'conveyor_commands')       
        while not self.cli_robot.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ConveyorCommands.Request()
        self.reqRobot = RobotCommands.Request()                     
        self.subscriber_ = self.create_subscription(String,
            'hmi_button_command',self.start_command_callback,10)         
        self.subscriber_ = self.create_subscription(String,
            'robot_command',self.robot_command_callback,10)         

    def send_command(self,com):
        '''Command client function to send a command'''
        self.get_logger().info('Command ' + str(com.data) + ' recieved')
        self.req.command = com.data
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(partial(self.callback_completed))

    def send_robot_command(self,com):
        '''Command client function to send a command'''
        self.get_logger().info('Command ' + str(com.data) + ' recieved')
        self.req.command = com.data
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(partial(self.callback_completed))

    def callback_completed(self,future):
        '''Client to check if the service works'''
        try:
            response = self.future.result()
        except:
            self.get_logger().error("service call failed %r" % (e,))


    def start_command_callback(self,command):
        '''start_button_command Subscriber callback function'''
        self.get_logger().info("instruction recieved: " + str(command))
        self.send_command(command)

    def robot_command_callback(self,command):
        '''start_button_command Subscriber callback function'''
        self.get_logger().info("instruction recieved: " + str(command.data))

        # send service message
        # self.send_command(command)

def main(args=None):
    rclpy.init(args=args)
    command_node = CommandNode()
    rclpy.spin(command_node)
    command_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()