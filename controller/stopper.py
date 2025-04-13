#!/usr/bin/env python3

'''
# Team ID:          2649
# Theme:            Hologlyph bot
# Author List:      Chandan Singh Chauhan , Ankit Dinkar , Anirudh Kumar
# Filename:         stopper.py
# Functions:        StopperNode.__init__, StopperNode.bot1_status_callback,
#                   StopperNode.bot2_status_callback, StopperNode.bot3_status_callback,
#                   StopperNode.check_and_call_service, StopperNode.call_stop_flag_service,
#                   StopperNode.stop_flag_callback, main
# Global variables: None
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Empty

class StopperNode(Node):

    """
    This node monitors the status of three robots and calls a service to stop an evaluator
    when all robots have reached their final positions.

    Attributes:
        bot1_stat_sub (Subscription): Subscription to the status topic of robot 1.
        bot2_stat_sub (Subscription): Subscription to the status topic of robot 2.
        bot3_stat_sub (Subscription): Subscription to the status topic of robot 3.
        bot1_status (bool): Flag indicating whether robot 1 has reached its final position.
        bot2_status (bool): Flag indicating whether robot 2 has reached its final position.
        bot3_status (bool): Flag indicating whether robot 3 has reached its final position.
        stop_flag_client (Client): Client for calling the Stop_Flag service.
    
    Methods:
        bot1_status_callback: Callback function for robot 1 status topic.
        bot2_status_callback: Callback function for robot 2 status topic.
        bot3_status_callback: Callback function for robot 3 status topic.
        check_and_call_service: Checks if all robots have reached their final positions and calls the service accordingly.
        call_stop_flag_service: Calls the Stop_Flag service.
        stop_flag_callback: Callback function for the Stop_Flag service.
    """

    def __init__(self):

        """
        Initializes the StopperNode.
        """

        super().__init__("stopper")
        self.bot1_stat_sub = self.create_subscription(Bool, "bot1_status", self.bot1_status_callback, 10)
        self.bot2_stat_sub = self.create_subscription(Bool, "bot2_status", self.bot2_status_callback, 10)
        self.bot3_stat_sub = self.create_subscription(Bool, "bot3_status", self.bot3_status_callback, 10)
        self.bot1_status = False
        self.bot2_status = False
        self.bot3_status = False
        self.stop_flag_client = self.create_client(Empty, "/Stop_Flag")
        self.get_logger().info("stopper_node_list")

    def bot1_status_callback(self, msg1: Bool):

        """
        Purpose:
            Callback function for robot 1 status topic.

        Args:
            msg1 (Bool): Message indicating whether robot 1 has reached its final position.
        """

        self.bot1_status = msg1.data
        self.get_logger().info("bot1 reached")
        self.check_and_call_service()

    def bot2_status_callback(self, msg2: Bool):

        """
        Purpose:
            Callback function for robot 2 status topic.

        Args:
            msg2 (Bool): Message indicating whether robot 2 has reached its final position.
        """

        self.bot2_status = msg2.data
        self.get_logger().info("bot2 reached")
        self.check_and_call_service()

    def bot3_status_callback(self, msg3: Bool):

        """
        Purpose:
            Callback function for robot 3 status topic.

        Args:
            msg3 (Bool): Message indicating whether robot 3 has reached its final position.
        """

        self.bot3_status = msg3.data
        self.get_logger().info("bot3 reached")
        self.check_and_call_service()

    def check_and_call_service(self):

        """
        Purpose:
            Checks if all robots have reached their final positions and calls the service accordingly.
        """

        if self.bot1_status and self.bot2_status and self.bot3_status:
            self.get_logger().info("stopping the evaluator")
            self.call_stop_flag_service()


    def call_stop_flag_service(self):
        
        """
        Purpose:
            create a Stop_Flag service.
        """

        self.srv = self.create_service(Empty, '/Stop_Flag', self.stop_flag_callback)
        self.get_logger().info('Stop_Flag server is ready')

    def stop_flag_callback(self, request, response):
        self.get_logger().info('Stop_Flag service called')

        return response


def main(args=None):

    """
    Main function to initialize the node and start spinning.
    """

    rclpy.init(args=args)
    node = StopperNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
