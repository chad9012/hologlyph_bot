#!/usr/bin/env python3

'''
# Team ID:          2649
# Theme:            Hologlyph Bot
# Author List:      Chandan Singh Chauhan
# Filename:         initializer.py
# Functions:        InitializerNode.__init__, InitializerNode.bot1_initial_callback,
#                   InitializerNode.bot2_initial_callback, InitializerNode.bot3_initial_callback,
#                   InitializerNode.publish_initial_reach, main
# Global variables: None
'''


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

class InitializerNode(Node):

    """
    This node subscribes to the initial reach topics of three robots and publishes a Bool message
    to indicate when all robots have reached their initial positions.

    Attributes:
        bot1_initial_value (bool): Flag indicating whether robot 1 has reached its initial position.
        bot2_initial_value (bool): Flag indicating whether robot 2 has reached its initial position.
        bot3_initial_value (bool): Flag indicating whether robot 3 has reached its initial position.
        subscription1 (Subscription): Subscription to the initial reach topic of robot 1.
        subscription2 (Subscription): Subscription to the initial reach topic of robot 2.
        subscription3 (Subscription): Subscription to the initial reach topic of robot 3.
        publisher (Publisher): Publisher for the initial reach topic indicating all robots have reached initial position.
    
    Methods:
        bot1_initial_callback: Callback function for robot 1 initial reach topic.
        bot2_initial_callback: Callback function for robot 2 initial reach topic.
        bot3_initial_callback: Callback function for robot 3 initial reach topic.
        publish_initial_reach: Publishes the Bool message indicating all robots have reached initial position.
    """

    def __init__(self):
        """
        Initializes the InitializerNode.
        """
        super().__init__("initializer")
        self.bot1_initial_value = False
        self.bot2_initial_value = False
        self.bot3_initial_value = False
        self.subscription1 = self.create_subscription(Bool, 'bot1_initial', self.bot1_initial_callback, 10)
        self.subscription2 = self.create_subscription(Bool, 'bot2_initial', self.bot2_initial_callback, 10)
        self.subscription3 = self.create_subscription(Bool, 'bot3_initial', self.bot3_initial_callback, 10)
        self.publisher = self.create_publisher(Bool, 'initial_reach', 10)
        self.get_logger().info("node started")

    def bot1_initial_callback(self, msg:Bool):
        """
        Callback function for robot 1 initial reach topic.

        Args:
            msg (Bool): Message indicating whether robot 1 has reached its initial position.
        """
        self.bot1_initial_value=msg.data
        self.get_logger().info("bot1_reached")
        self.publish_initial_reach()

    def bot2_initial_callback(self, msg:Bool):
        """
        Callback function for robot 2 initial reach topic.

        Args:
            msg (Bool): Message indicating whether robot 2 has reached its initial position.
        """
        self.bot2_initial_value=msg.data
        self.get_logger().info("bot2_reached")
        self.publish_initial_reach()

    def bot3_initial_callback(self, msg:Bool):
        """
        Callback function for robot 3 initial reach topic.

        Args:
            msg (Bool): Message indicating whether robot 3 has reached its initial position.
        """
        self.bot3_initial_value=msg.data
        self.get_logger().info("bot3_reached")
        self.publish_initial_reach()

    def publish_initial_reach(self):
        """
        Publishes the Bool message indicating all robots have reached initial position.
        """
        if self.bot1_initial_value and self.bot2_initial_value and self.bot3_initial_value:
            msg = Bool()
            msg.data = True
            self.get_logger().info("publishing data "+str(msg.data))
            self.publisher.publish(msg)


def main(args=None):
    """
    Main function to initialize the node and start spinning.
    """
    rclpy.init(args=args)
    node = InitializerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
