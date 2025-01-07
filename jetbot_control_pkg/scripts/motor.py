#!/usr/bin/env python3

import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the LaserScan module from sensor_msgs interface
#from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from adafruit_motorkit import MotorKit
# import Quality of Service library, to set the correct profile and reliability to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile


class Motor(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('motor')
        # create the subscriber object
        # in this case, the subscriptor will be subscribed on /scan topic with a queue size of 10 messages.
        # use the LaserScan module for /scan topic
        # send the received info to the listener_callback method.
        self.subscriber = self.create_subscription(
            Twist,                  # message type
            '/cmd_vel',                    # topic
            self.listener_callback,     # listener callback function
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))  
            # is the most used to read LaserScan data and some sensor data.

        self.kit = MotorKit()


    def listener_callback(self, msg):
        # print the log info in the terminal
        self.get_logger().info('I receive: "%s"' % str(msg))
        self.command_motors(msg.linear.x, msg.angular.z)
    
    def command_motors(self, linear, angular):
        # motor1 is the right motor
        # motor2 is the left motor
    	# Convert linear and angular velocities to motor commands
        left_motor_speed = linear - angular
        right_motor_speed = linear + angular
    	# Ensure the values are within [-1.0, 1.0] for MotorKit
        self.kit.motor2.throttle = max(min(left_motor_speed, 1.0), -1.0)
        self.kit.motor1.throttle = max(min(right_motor_speed, 1.0), -1.0)
        

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    motor = Motor()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(motor)
    # Explicity destroy the node
    motor.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
