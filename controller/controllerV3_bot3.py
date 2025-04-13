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
        self.current_pose_subscription = self.create_subscription(Pose2D, "/pen3_pose", self.current_pose_subscription_callback, 10)
        self.wheel_velocity_publisher = self.create_publisher(Int64, "bot3WheelVelocity", 10)

        # bot2_status is for telling the information that this bot has completed it's run and then creating the server /Stop_Flag to stop evaluator
        self.completion_publisher=self.create_publisher(Bool,"bot3_status",10)

        #this is for telling initilizer code that it has reached it's starting position and now bot will do pen down move if it get True from initial_reach 
        self.inital_reach_publisher=self.create_subscription(Bool,"initial_reach",self.inital_reach_callback,10)
        self.bot3_initial_publisher=self.create_publisher(Bool,"bot3_initial",10)

        self.pen_pose_publisher = self.create_publisher(Bool,"/pen3_down",10)
        
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0

        self.goal_theta = 0
        self.left_wheel_velocity = 0
        self.right_wheel_velocity = 0
        self.rear_wheel_velocity = 0
        self.kp = 0.04

        self.goal_array=[
            [55.00000000000022, 95.26279441628851],
            [40.774075109211914, 75.84972874836173],
            [27.221408603561432, 54.54586147821884],
            [14.633519488365472, 31.695100473037616],
            [3.266567206628955, 7.679342825975884],
            [-6.664261951518498, -17.089116213729223],
            [-14.989319563882582, -42.17590310331036],
            [-21.587909143804016, -67.13343299379085],
            [-26.390475100732697, -91.51012926840963],
            [-29.379521124983135, -114.85979254893027],
            [-30.58924252373413, -136.75091217233947],
            [-30.103887340164867, -156.77571391536839],
            [-28.0548909986122, -174.5587431951283],
            [-24.616858042466898, -189.7647929807011],
            [-20.00249160035604, -202.1059999883152],
            [-14.456595903018833, -211.34795107812567],
            [-8.249298906525796, -217.31466370835503],
            [-1.6686603539833516, -219.89232933500108],
            [4.987154995438269, -219.0317362041128],
            [11.41794905760768, -214.74931744249002],
            [17.32961138706542, -207.12680104181356],
            [22.442092611904798, -196.30946955270562],
            [26.497036992986974, -182.50306834893186],
            [29.26488874308756, -165.96943148180782],
            [30.551299067478347, -147.02092273504664],
            [30.202677271512908, -126.01381586090373],
            [28.11074934378952, -103.34076153047081],
            [24.21601070098239, -79.42250872983965],
            [18.50998573351425, -54.69906472182628],
            [11.036234818853986, -29.62048989952057],
            [1.8900789220740841, -4.637531608250944],
            [-8.78295790114284, 19.807695866146975],
            [-20.78995754571102, 43.29077951963465],
            [-33.894608309944154, 65.41362467000609],
            [-47.822308393630784, 85.81243216078023],
            [-62.26625784298706, 104.16486207277937],
            [-76.8944007239656, 120.19622919651393],
            [-91.35705884762271, 133.68459790289154],
            [-105.29508155373364, 144.46466939370703],
            [-118.34832327578732, 152.43038203378813],
            [-130.16425216340883, 157.5361749266967],
            [-140.40648912888656, 159.79689541926672],
            [-148.76307741012138, 159.28636210701447],
            [-154.9542880935314, 156.13462545491012],
            [-158.73977690120776, 150.5239976470378],
            [-159.92492169692073, 142.68395105979937],
            [-158.36618828644995, 132.88501018211196],
            [-153.97539376847388, 121.4317843027475],
            [-146.72276143994867, 108.65530733769647],
            [-136.63868851052868, 94.90486634701098],
            [-123.81417701207025, 80.53951124832723],
            [-108.39990863571363, 65.91944472787274],
            [-90.60397509599656, 51.39749323686123],
            [-70.68830630189295, 37.3108572056001],
            [-48.96386840511368, 23.973331280069534],
            [-25.784732012847428, 11.668173662209364],
            [-1.5411368471651736, 0.6417877927637244],
            [23.348297691207186, -8.90163997387823],
            [48.445046774784096, -16.8044259319225],
            [73.29965392513051, -22.958502686762465],
            [97.46100718984157, -27.307293574461852],
            [120.48571322260578, -29.846309800998828],
            [141.94736202166573, -30.62246240303997],
            [161.44547703132224, -29.73211140417378],
            [178.61395191634415, -27.317904244013867],
            [193.1287864335105, -23.56448400393215],
            [204.71494919826452, -18.693174483376957],
            [213.1522144302783, -12.955773180312601],
            [218.27984252273777, -6.627604142602774],
            [220.0, -5.388445916248354e-14]
        ]                        


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
            self.publish_data(self.right_wheel_velocity, self.left_wheel_velocity, self.rear_wheel_velocity)

        else:
            if self.current_goal_index == len(self.goal_array)-1:
                bool_msg=Bool()
                bool_msg.data=True
                self.completion_publisher.publish(bool_msg)
                time.sleep(1)
                self.get_logger().warn("completed run")
                rclpy.shutdown()
            else:
                if self.current_goal_index==0:
                    bot3_initial_msg=Bool()
                    bot3_initial_msg.data=True
                    self.bot3_initial_publisher.publish(bot3_initial_msg)

                if (self.flag):
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
