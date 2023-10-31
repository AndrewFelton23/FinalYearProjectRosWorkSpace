#!/usr/bin/python3
from std_msgs.msg._string import String
from tutorial_interfaces.msg import Vision

import rclpy
from rclpy.node import Node

xrc = 250
yrc = 190



class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')
        self.get_logger().info("Node has started")
        self.subscriber_ = self.create_subscription(Vision,
            '/vision',self.kinematics_callback,10)
        self.get_logger().info('node has started') 

    def kinematics_callback(self,msg):
        '''a callback to handle the incoming coordinates'''
        # Find the x and y distance from the robot to the part
        angle = msg.angle 
        xcp = msg.x 
        ycp = msg.y
        # Perform kinematic equations to find joint angles
        self.get_logger().info("Received the following coordinates from vision sensor node: angle {}, xcp {}, ycp {}".format(angle,xcp,ycp))

        

        


def main(args=None):
    rclpy.init(args=args)

    kinematics_node = KinematicsNode()

    rclpy.spin(kinematics_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()