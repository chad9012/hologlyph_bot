'''
# Team ID:          2649
# Theme:            Hologlyph Bots
# Author List:      Chandan Singh Chauhan
# Filename:         camera_calibrater.py
# Functions:        CameraNode, image_callback, main
# Global variables: aruco_dict, parameters
'''


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
import cv_bridge
import numpy as np

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
parameters = cv2.aruco.DetectorParameters()


class CameraNode(Node):

    '''
    Purpose:
    ---
    Class to handle camera operations, including image calibration and perspective transformation.

    Functions:
    ---
    __init__: Initialize the CameraNode class.
    image_callback: Callback function for processing image messages.
    '''

    def __init__(self):

        '''
        Purpose:
        ---
        Initializes the CameraNode class and creates publishers and subscribers.
        '''

        super().__init__("CameraOutputNode")
        
        # Create a publisher for the calibrated and transformed image
        self.publisher = self.create_publisher(Image, 'image_processed', 10)
        
        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            10)
        self.cv_bridge = cv_bridge.CvBridge()

        # Load new calibration matrix
        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = 'narrow_stereo'
        self.camera_info.width = 640
        self.camera_info.height = 480
        self.camera_info.distortion_model = 'plumb_bob'
        self.camera_info.d = [-0.335191, 0.090382, -0.007615, -0.000089, 0.000000]
        self.camera_info.k = [442.52336, 0., 311.84312, 0., 450.8585, 196.40298, 0., 0., 1.]
        self.camera_info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        self.camera_info.p = [345.2465, 0., 306.81417, 0., 0., 389.68532, 180.00655, 0., 0., 0., 1., 0.]


        self.transform_matrix1=None
        self.flag=0
        

    def image_callback(self, msg):

        '''
        Purpose:
        ---
        Callback function to process image messages and perform image calibration and perspective transformation.

        Input Arguments:
        ---
        msg : Image
            Image message received from the camera.
        '''

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error('Error converting ROS Image to OpenCV image: %s' % str(e))
            return

        # Perform image calibration using the new camera info
        k = np.reshape(self.camera_info.k, (3, 3))
        d = np.array(self.camera_info.d)
        calibrated_image = cv2.undistort(cv_image, k, d)

        if (self.flag==0):
            gray_calibrated_image=cv2.cvtColor(calibrated_image,cv2.COLOR_BGR2GRAY)
            corners,ids,rejectedImgPoints=cv2.aruco.detectMarkers(gray_calibrated_image,aruco_dict,parameters=parameters)
            if (ids is not None):
                for i in range(len(ids)):
                    if ids[i]==10 :
                        top_left_corner_x=corners[i][0][1][0]
                        top_left_corner_y=corners[i][0][1][1]
                    elif ids[i]==12 :
                        top_right_corner_x=corners[i][0][2][0]
                        top_right_corner_y=corners[i][0][2][1]
                    elif ids[i]==4 :
                        bottom_right_corner_x=corners[i][0][3][0]
                        bottom_right_corner_y=corners[i][0][3][1]  
                    elif ids[i]==8 :
                        bottom_left_corner_x=corners[i][0][0][0]
                        bottom_left_corner_y=corners[i][0][0][1]

            self.transform_matrix1=np.array([
                [top_left_corner_x,top_left_corner_y],
                [top_right_corner_x,top_right_corner_y],
                [bottom_right_corner_x,bottom_right_corner_y],
                [bottom_left_corner_x,bottom_left_corner_y]],dtype=np.float32)
            self.flag=1
        # Perform perspective transform
        transform_matrix = np.array([
            [0, 0],
            [499, 0],
            [499, 499],
            [0, 499]], dtype=np.float32)

        perspective_transform = cv2.getPerspectiveTransform(self.transform_matrix1, transform_matrix)
        transformed_image = cv2.warpPerspective(calibrated_image, perspective_transform, (500, 500))

        # Publish the transformed image
        transformed_image_msg = self.cv_bridge.cv2_to_imgmsg(transformed_image, encoding='bgr8')
        transformed_image_msg.header = msg.header
        self.publisher.publish(transformed_image_msg)

    


def main(args=None):

    '''
    Purpose:
    ---
    Main function to initialize ROS 2 node and CameraNode.

    Input Arguments:
    ---
    args : list
        List of input arguments from command line. Defaults to None.
    '''

    rclpy.init(args=args)

    output = CameraNode()

    rclpy.spin(output)

    output.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
