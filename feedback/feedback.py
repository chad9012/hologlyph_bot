#! /usr/bin/env python3

# File Level Comments
'''
# Team ID:          2649
# Theme:            Hologlyph Bots
# Author List:      Chandan Singh Chauhan
# Filename:         feedback.py
# Functions:        ArUcoDetector.__init__, ArUcoDetector.send_pose_value, 
#                   ArUcoDetector.image_callback, ArUcoDetector.process_aruco_marker,
#                   main
# Global variables: aruco_dict, parameters
'''

# Import necessary libraries and modules
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
import cv2
from cv_bridge import CvBridge
import numpy as np

# Define the ArUco dictionary and parameters for detection
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
parameters = cv2.aruco.DetectorParameters()

class ArUcoDetector(Node):

    def __init__(self):

        '''
        Purpose:
        ---
        Constructor method to initialize the ArUcoDetector node and its attributes.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None
        '''

        super().__init__('ar_uco_detector')

        '''
        Purpose:
        ---
        Constructor method to initialize the ArUcoDetector node and its attributes.
        '''

        self.camera_subscriber = self.create_subscription(Image, '/image_processed', self.image_callback, 10)
        self.bridge = CvBridge()
        self.pose_publisher_1 = self.create_publisher(Pose2D, "/pen1_pose", 10)
        self.pose_publisher_2 = self.create_publisher(Pose2D, "/pen2_pose",10)
        self.pose_publisher_3 = self.create_publisher(Pose2D, "/pen3_pose",10)
        self.hb=14 #height of bot (from group to aruco position height )
        self.hc=240 #height of camera 


    def send_pose_value(self, x, y, theta, marker_id):

        '''
        Purpose:
        ---
        Method to publish the pose information (x, y, theta) of ArUco markers.

        Input Arguments:
        ---
        x : float
            X-coordinate of the marker.
        y : float
            Y-coordinate of the marker.
        theta : float
            Orientation angle of the marker (in radians).
        marker_id : int
            ID of the marker.

        Returns:
        ---
        None
        '''

        msg = Pose2D()
        msg.x = x
        msg.y = y
        msg.theta = theta
        if marker_id==1:
            self.pose_publisher_1.publish(msg)
        elif marker_id==2:
            self.pose_publisher_2.publish(msg)
        elif marker_id==3:
            self.pose_publisher_3.publish(msg)

    def image_callback(self, msg: Image):

        '''
        Purpose:
        ---
        Callback function to process images received from the camera subscriber.

        Input Arguments:
        ---
        msg : Image
            ROS Image message containing the captured image.

        Returns:
        ---
        None
        '''

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        cv2.aruco.drawDetectedMarkers(cv_image, corners,ids)
        if (ids is not None):
            for i in range(len(ids)):
                if ids[i] == 1:
                    self.process_aruco_marker(cv_image, corners[i], ids[i])

                if ids[i] == 2 :
                    self.process_aruco_marker(cv_image, corners[i], ids[i])

                if ids[i]==3:
                    self.process_aruco_marker(cv_image,corners[i],ids[i])



        cv2.imshow('Image', cv_image)
        cv2.waitKey(1)

    def process_aruco_marker(self, cv_image, corners, marker_id):

        '''
        Purpose:
        ---
        Method to process the detected ArUco markers and compute their poses.

        Input Arguments:
        ---
        cv_image : numpy.ndarray
            OpenCV image containing the detected markers.
        corners : list
            List of corner points of the detected marker.
        marker_id : int
            ID of the detected marker.

        Returns:
        ---
        None
        '''

        x_coordinate = (corners[0][0][0] + corners[0][1][0] + corners[0][2][0] + corners[0][3][0]) / 4
        y_coordinate = (corners[0][0][1] + corners[0][1][1] + corners[0][2][1] + corners[0][3][1]) / 4

        delta_x = corners[0][1][0] - corners[0][0][0]
        delta_y = -(corners[0][1][1] - corners[0][0][1])

        theta_radians = float(np.arctan2(delta_y, delta_x))
        theta_radians = float(np.arctan2(delta_y, delta_x))
        theta_radians = (theta_radians + 2 * np.pi) % (2 * np.pi)

        #below code remove the effect occur due to height of bot (center of aruco marker not match with pen pose )
        #pen is just below the center of aruco marker so no lateral displacemt . just displacement due to height 
        x_coordinate=x_coordinate*(1-(self.hb/self.hc))+250*(self.hb/self.hc)
        y_coordinate=y_coordinate*(1-(self.hb/self.hc))+250*(self.hb/self.hc)
        # x_coordinate=x_coordinate-250

        # y_coordinate=250-y_coordinate


        print("Center of the ArUco marker {}: ({}, {}, {})".format(marker_id, x_coordinate, y_coordinate, theta_radians))
        self.send_pose_value(x_coordinate, y_coordinate, theta_radians, marker_id)

def main(args=None):

    '''
    Purpose:
    ---
    Main function to initialize ROS 2 node and ArUcoDetector.
    
    Input Arguments:
    ---
    args : list
        List of input arguments from command line. Defaults to None.
    
    Returns:
    ---
    None
    '''

    rclpy.init(args=args)

    aruco_detector = ArUcoDetector()

    rclpy.spin(aruco_detector)

    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
