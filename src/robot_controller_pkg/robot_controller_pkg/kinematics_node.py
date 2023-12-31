#!/usr/bin/python3
import numpy as np
from math import acos, atan2, sin, cos, pi, sqrt
from std_msgs.msg._string import String
from tutorial_interfaces.msg import Vision

import rclpy
from rclpy.node import Node

class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')
        self.get_logger().info("Node has started")
        self.subscriber_ = self.create_subscription(Vision,
            '/vision',self.kinematics_callback,10)
        self.publisher_robot_command = self.create_publisher(String,
            'robot_command',10)         
        self.get_logger().info('node has started') 

    def kinematics_callback(self,msg):
        '''a callback to handle the incoming coordinates'''
        # Find the x and y distance from the robot to the part
        flag1 = True
        flag2 = True
        flag3 = True
        xrc = 235
        yrc = 190
        L1 = 295
        L2 = 200
        InverseKinematics = True
        ForwardKinematics = True
        d1 = 20
        d2 = 60
        ratio1 = d1/d2
        ratio2 = d1/d2
        stepsPerDeg = 400/360
        angle = msg.angle 
        xcp = msg.x 
        ycp = msg.y
        self.get_logger().info("Received the following coordinates from vision sensor node: angle {}, xcp {}, ycp {}".format(angle,xcp,ycp))
        xrp = yrc - ycp
        yrp = xrc - xcp
        self.get_logger().info("The part is detected to be x :{} and y: {} away from the robot".format(xrp,yrp))
        # Perform kinematic equations to find joint angles
        if (sqrt((xrp**2)+(yrp**2)) > (L1+L2)):
            flag1 = False
            print("Desired position is out of range.")
        if xrp < 0:
            flag2 = False
            print("X position is negative")
        if yrp < 10:
            flag3 = False
            print("Y position is too close to the robot")

        if flag1 & flag2 & flag3:
            theta2_radians = acos(((xrp**2)+(yrp**2)-(L1**2)-(L2**2))/(2*L1*L2))
            theta1_radians = atan2(xrp,yrp) - atan2((L2*sin(theta2_radians)), (L1 + L2*cos(theta2_radians)))

            theta2_deg = (theta2_radians / (2 * pi)) * 360
            theta1_deg = 90 + (theta1_radians / (2 * pi)) * 360
            theta1_steps = theta1_deg*stepsPerDeg/(ratio1*ratio2)
            theta2_steps = theta2_deg*stepsPerDeg/(ratio1*ratio2)

            output_steps_1 = round(theta1_steps)
            output_steps_2 = round(theta2_steps)

            # The error will be as follows
            theta1_act_deg = output_steps_1/stepsPerDeg*(ratio1*ratio2)
            theta2_act_deg = output_steps_2/stepsPerDeg*(ratio1*ratio2)

            theta1_error = theta1_deg - theta1_act_deg
            theta2_error = theta2_deg - theta2_act_deg
            # calculate the current position using forward kinematics
            theta1_act_radians = (theta1_act_deg / 360) * 2 * pi
            theta2_act_radians = (theta2_act_deg / 360) * 2 * pi
            x_act = L1 * sin(theta1_act_radians) + L2 * sin(theta1_act_radians + theta2_act_radians)
            y_act = L1 * cos(theta1_act_radians) + L2 * cos(theta1_act_radians + theta2_act_radians)
        # Forward Kinematics
        if ForwardKinematics:
            theta1_radians = (theta1_deg / 360) * 2 * pi
            theta2_radians = (theta2_deg / 360) * 2 * pi
            xrp = L1 * sin(theta1_radians) + L2 * sin(theta1_radians + theta2_radians)
            yrp = L1 * cos(theta1_radians) + L2 * cos(theta1_radians + theta2_radians)
        # move the x motor by theta 1
        xMove = output_steps_1
        zMove = output_steps_2
        aCurrent = (theta1_deg+theta2_deg)
        aMove = round((aCurrent - angle)*stepsPerDeg/(ratio1*ratio2))
        self.get_logger().info("Theta 1 is {} and theta 2 is {}. This means that motor connected to X must move {} and Z {} the current orientation of the gripper is {}".format(theta1_act_deg,theta2_act_deg,xMove,zMove,aMove))
        self.get_logger().info("The x desired position is {} and the y desired position is {}. The x actual position is {} and the y actual position is {}.".format(str(xrp), str(yrp), str(x_act), str(y_act)))
        command = String()
        command.data = "Data,{},{},{},{}".format(xMove,zMove,0,aMove)
        self.publisher_robot_command.publish(command)
        self.get_logger().info("Published on robot_command: " + command.data)

def main(args=None):
    rclpy.init(args=args)

    kinematics_node = KinematicsNode()

    rclpy.spin(kinematics_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()