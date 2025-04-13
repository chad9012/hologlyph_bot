'''
# Team ID:          2649
# Theme:            Hologlyph Bots
# Author List:      Chandan Singh Chauhan
# Filename:         controllerV2_bot1.py
# Functions:        update_goal, inverse_kinematics, convert_to_servo_scaled publish_data, current_pose_subscription_callback
# Global variables: None
'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int64
import math
from std_msgs.msg import Bool
import time

class ControllerBot1Node(Node):

    '''
    Purpose:
    ---
    Class to control Bot1 including its movement and pen status.

    Variables:
    ---
    current_x : float
        Current x-coordinate of Bot1.
    current_y : float
        Current y-coordinate of Bot1.
    current_theta : float
        Current orientation angle of Bot1.
    goal_x : float
        X-coordinate of the current goal position.
    goal_y : float
        Y-coordinate of the current goal position.
    goal_array : list
        Array containing coordinates of all the goals.
    current_goal_index : int
        Index of the current goal position.
    kp : float
        Proportional gain constant.
    '''

    def __init__(self):
        super().__init__("controller_bot1")
        self.current_pose_subscription = self.create_subscription(Pose2D, "/pen1_pose", self.current_pose_subscription_callback, 10)
        self.wheel_velocity_publisher = self.create_publisher(Int64, "bot1WheelVelocity", 10)
        self.pen_pose_publisher = self.create_publisher(Bool,"/pen1_down",10)

        #this is for telling initilizer node that it has reached it's starting position and now bot will do pen down move if it get True from initial_reach
        self.inital_reach_subscription=self.create_subscription(Bool,"initial_reach",self.inital_reach_callback,10)
        self.bot1_initial_publisher=self.create_publisher(Bool,"bot1_initial",10)

        # bot2_status is for telling the information that this bot has completed it's run and then creating the server /Stop_Flag to stop evaluator
        self.completion_publisher=self.create_publisher(Bool,"bot1_status",10)

        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0

        self.goal_theta = 0
        self.left_wheel_velocity = 0
        self.right_wheel_velocity = 0
        self.rear_wheel_velocity = 0
        self.kp = 0.045

        self.goal_array=[[220.0, 0.0],
        [218.27984252273777, 6.627604142602745],
        [213.1522144302783, 12.955773180312598],
        [204.7149491982645, 18.69317448337697],
        [193.12878643351044, 23.564484003932183],
        [178.61395191634446, 27.317904244013803],
        [161.44547703132255, 29.732111404173757],
        [141.94736202166595, 30.62246240303997],
        [120.48571322260597, 29.846309800998842],
        [97.46100718984168, 27.30729357446187],
        [73.29965392513124, 22.958502686762618],
        [48.44504677478476, 16.804425931922683],
        [23.348297691207765, 8.90163997387843],
        [-1.5411368471647044, -0.6417877927635282],
        [-25.784732012847055, -11.668173662209183],
        [-48.963868405113416, -23.973331280069385],
        [-70.68830630189278, -37.310857205599994],
        [-90.60397509599647, -51.39749323686115],
        [-108.39990863571363, -65.91944472787274],
        [-123.81417701207029, -80.53951124832726],
        [-136.63868851052843, -94.90486634701067],
        [-146.7227614399485, -108.65530733769623],
        [-153.97539376847377, -121.4317843027473],
        [-158.3661882864499, -132.88501018211178],
        [-159.92492169692076, -142.6839510597993],
        [-158.7397769012078, -150.5239976470378],
        [-154.95428809353155, -156.13462545490998],
        [-148.76307741012158, -159.2863621070144],
        [-140.4064891288868, -159.79689541926672],
        [-130.16425216340903, -157.5361749266968],
        [-118.34832327578751, -152.43038203378825],
        [-105.29508155373378, -144.4646693937071],
        [-91.35705884762282, -133.68459790289162],
        [-76.89440072396572, -120.19622919651403],
        [-62.26625784298707, -104.1648620727794],
        [-47.822308393630685, -85.81243216078012],
        [-33.894608309944466, -65.41362467000657],
        [-20.789957545711303, -43.290779519635166],
        [-8.782957901143014, -19.80769586614735],
        [1.8900789220740042, 4.637531608250744],
        [11.036234818853918, 29.620489899520365],
        [18.509985733514192, 54.69906472182607],
        [24.21601070098225, 79.42250872983892],
        [28.110749343789454, 103.34076153047029],
        [30.202677271512876, 126.01381586090322],
        [30.55129906747836, 147.02092273504618],
        [29.264888743087585, 165.9694314818075],
        [26.497036992987006, 182.50306834893175],
        [22.442092611904837, 196.30946955270548],
        [17.329611387065462, 207.12680104181348],
        [11.417949057607684, 214.74931744249002],
        [4.987154995438419, 219.03173620411275],
        [-1.6686603539832003, 219.8923293350011],
        [-8.249298906525649, 217.3146637083551],
        [-14.45659590301874, 211.3479510781258],
        [-20.002491600356, 202.1059999883153],
        [-24.616858042466863, 189.76479298070123],
        [-28.05489099861218, 174.55874319512841],
        [-30.10388734016483, 156.77571391536898],
        [-30.58924252373413, 136.75091217233992],
        [-29.379521124983178, 114.85979254893076],
        [-26.390475100732786, 91.51012926841018],
        [-21.587909143804104, 67.13343299379123],
        [-14.989319563882642, 42.17590310331057],
        [-6.664261951518571, 17.089116213729422],
        [3.266567206628869, -7.679342825975687],
        [14.633519488365023, -31.69510047303674],
        [27.22140860356113, -54.54586147821833],
        [40.77407510921159, -75.84972874836124],
        [54.99999999999989, -95.26279441628812]]

        self.flag=False
        self.current_goal_index=0

        self.goal_x,self.goal_y=self.goal_array[self.current_goal_index]

    def inital_reach_callback(self,initial_reach_msg:Bool):
        self.flag=initial_reach_msg.data
        

    def update_goal(self):

        '''
        Purpose:
        ---
        Update the current goal position to the next position in the array.
        '''
        self.goal_x,self.goal_y=self.goal_array[self.current_goal_index]
        
        self.goal_theta=0

    def inverse_kinematics(self, v_x, v_y, v_theta):

        '''
        Purpose:
        ---
        Calculate wheel velocities based on the linear and angular velocities.

        Input Arguments:
        ---
        v_x : float
            Linear velocity in x-direction.
        v_y : float
            Linear velocity in y-direction.
        v_theta : float
            Angular velocity around the z-axis.

        Returns:
        ---
        servo_1 : int
            Velocity for servo 1.
        servo_2 : int
            Velocity for servo 2.
        servo_3 : int
            Velocity for servo 3.
        '''      

        v_1 = -0.33 * v_x + 0.577 * v_y + 0.33 * v_theta
        v_2 = -0.33 * v_x - 0.577 * v_y + 0.33 * v_theta  
        v_3 = 0.67 * v_x + 0.33 * v_theta

        v_1 = max(min(v_1, 1), -1)
        v_2 = max(min(v_2, 1), -1)
        v_3 = max(min(v_3, 1), -1)

        servo_1,servo_2,servo_3=self.convert_to_servo_scaled(v_1,v_2,v_3)
        return servo_1,servo_2,servo_3

    def convert_to_servo_scaled(self, v1, v2, v3):

        '''
        Purpose:
        ---
        Scale velocities to servo values.

        Input Arguments:
        ---
        v1 : float
            Velocity for servo 1.
        v2 : float
            Velocity for servo 2.
        v3 : float
            Velocity for servo 3.

        Returns:
        ---
        servo_1 : int
            Scaled velocity for servo 1.
        servo_2 : int
            Scaled velocity for servo 2.
        servo_3 : int
            Scaled velocity for servo 3.
        '''

        # Scale velocities to servo values
        servo_1 = int(90 * (-v1 + 1))
        servo_2 = int(90 * (-v2 + 1))
        servo_3 = int(90 * (-v3 + 1))

        '''by applying the below code servo will only stop moving for pwm value of 90 
          if the pwm vaue is in range between 91 to 98 servo_1 will not stop due of manufacturing issue of motors so i 
          forcefully move it by apply min pwm value for which servo_1 move (99) same in reverse direction and for all three servo 
        '''
        if 91<=servo_1<99:
            servo_1=99
        elif 89>=servo_1>81:
            servo_1=81

        if 91<=servo_2<99:
            servo_2=99
        elif 89>=servo_2>81:
            servo_2=81

        if 91<=servo_3<99:
            servo_3=99
        elif 89>=servo_1>81:
            servo_1=81



        return servo_1, servo_2, servo_3

    def publish_data(self, wheel_velocity1, wheel_velocity2, wheel_velocity3):

        '''
        Purpose:
        ---
        Encode data and publish wheel velocities and pen status.

        Input Arguments:
        ---
        wheel_velocity1 : float
            Velocity for wheel 1.
        wheel_velocity2 : float
            Velocity for wheel 2.
        wheel_velocity3 : float
            Velocity for wheel 3.
        '''

        # Encode data and publish
        encoded_data = int(round(wheel_velocity1 * 1e15) +
                           round(wheel_velocity2 * 1e9) +
                           round(wheel_velocity3 * 1e3))

        msg = Int64()
        msg.data = encoded_data

        pen_msg=Bool()
        # to go to the first goal position from home position without doing pen down motion 
        if (self.current_goal_index==0):
            pen_msg.data=False
        else:
            #for other goal position do pen down motion 
            pen_msg.data=True
        self.pen_pose_publisher.publish(pen_msg)

        self.wheel_velocity_publisher.publish(msg)

    def current_pose_subscription_callback(self, msg: Pose2D):

        '''
        Purpose:
        ---
        Callback function to update current pose of the bot and control its movement.

        Input Arguments:
        ---
        msg : Pose2D
            Current pose message containing position and orientation data.
        '''

        self.current_x = msg.x
        self.current_y = msg.y

        #change coodinate system from image coodinate system to arena coodinate system with center of arena is origin with y direction reversed of image coodinate 
        self.current_x=self.current_x-250
        self.current_y=250-self.current_y

        self.current_theta = msg.theta
        
        #calucuating error in global frame 
        error_x_global = self.goal_x - self.current_x
        error_y_global = self.goal_y - self.current_y
        error_theta = math.atan2(math.sin(self.goal_theta - self.current_theta), math.cos(self.goal_theta - self.current_theta))


        #calculating error in robot frame
        error_x_body = error_x_global * math.cos(self.current_theta) - error_y_global * math.sin(self.current_theta)
        error_y_body = error_x_global * math.sin(self.current_theta) + error_y_global * math.cos(self.current_theta)


        error_x_body = max(min(error_x_body, 100), -100)
        error_y_body = max(min(error_y_body, 100), -100)
        error_theta = max(min(error_theta, math.pi), -math.pi)


        # Calculate control velocities
        v_x = self.kp * error_x_body
        v_y = self.kp * error_y_body
        v_theta = 110*self.kp * error_theta

        
        #calcuate pwm values based on control velocity
        self.right_wheel_velocity, self.left_wheel_velocity, self.rear_wheel_velocity = self.inverse_kinematics(v_x, v_y, v_theta)

        # Publish velocities
        if (not((-3 < error_x_body < 3) and (-0.05 < error_theta < 0.05) and (-3 < error_y_body < 3))):
            # publish velocity if not reach the goal in certain error range 
            self.publish_data(self.right_wheel_velocity, self.left_wheel_velocity, self.rear_wheel_velocity)

        else:
            #if it iterated all the goal points 
            if self.current_goal_index == len(self.goal_array)-1:
                #send bool msg to stopper to stop the evaluator as this bot has completed it's run 
                bool_msg=Bool()
                bool_msg.data=True
                self.completion_publisher.publish(bool_msg)
                time.sleep(1)
                self.get_logger().warn("completed run")
                rclpy.shutdown()
            else:
                #goal to the next goal it has reached the end of list
                # if it reached in the starting position send bool msg to initilizer to send msg that this bot has reached to it's initial position 
                if self.current_goal_index==0:
                    bot1_initial_msg=Bool()
                    bot1_initial_msg.data=True
                    self.bot1_initial_publisher.publish(bot1_initial_msg)
                #check if all the bot has reached it's initial positon (tell by initilizer node  by /initial_reach topic )
                if self.flag:
                    self.current_goal_index=self.current_goal_index+1
                    self.update_goal()

        

def main(args=None):

    '''
    Purpose:
    ---
    Main function to initialize ROS 2 node and spin the controller node.

    Input Arguments:
    ---
    args : list
        List of input arguments from command line. Defaults to None.
    '''

    rclpy.init(args=args)
    node = ControllerBot1Node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
