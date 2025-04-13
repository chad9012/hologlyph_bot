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
    self.flag: Bool
        to initialize the pen down motion syncronizely for all three bots 
    '''

    def __init__(self):
        super().__init__("controller_bot1")
        self.current_pose_subscription = self.create_subscription(Pose2D, "/pen2_pose", self.current_pose_subscription_callback, 10)
        self.wheel_velocity_publisher = self.create_publisher(Int64, "bot2WheelVelocity", 10)
        
        # bot2_status is for telling the information that this bot has completed it's run and then creating the server /Stop_Flag to stop evaluator
        self.completion_publisher=self.create_publisher(Bool,"bot2_status",10)
        
        #this is for telling initilizer code that it has reached it's starting position and now bot will do pen down move if it get True from initial_reach 
        self.bot2_initial_publisher=self.create_publisher(Bool,"bot2_initial",10)
        self.inital_reach_publisher=self.create_subscription(Bool,"initial_reach",self.initial_reach_callback,10)


        self.pen_pose_publisher = self.create_publisher(Bool,"/pen2_down",10)
        
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0

        self.goal_theta = 0
        self.left_wheel_velocity = 0
        self.right_wheel_velocity = 0
        self.rear_wheel_velocity = 0
        self.kp = 0.04

        
        self.goal_array=[
            [54.99999999999989, -95.26279441628812],
            [69.57884040345574, -112.4858374528834],
            [84.16873850529545, -127.26998386278333],
            [98.41423613814406, -139.42120943823193],
            [111.95461262288202, -148.80359165441556],
            [124.43245280519365, -155.34124596223333],
            [135.50224603290118, -159.01891181255624],
            [144.83881522228089, -159.88118453944944],
            [152.14537820786296, -158.03042003274425],
            [157.16105119729733, -153.6233692373122],
            [159.66761619222927, -146.86662823863705],
            [159.49539041751999, -138.01101637545963],
            [156.5280557586847, -127.34501885153404],
            [150.70632948799445, -115.1874511525112],
            [142.0303836332955, -101.87951973725619],
            [130.56094861550142, -87.77646657697144],
            [116.41906660507533, -73.23899386550532],
            [99.78449074249481, -58.62466942578313],
            [80.89275722993382, -44.279512900892456],
            [60.03098762389983, -30.52995776128774],
            [37.53250774580624, -17.67537460559496],
            [13.770396811618557, -5.981327408809115],
            [-10.84989496201198, 4.326283396313547],
            [-35.89870045663753, 13.066058415054593],
            [-60.93088366846353, 20.10480910139973],
            [-85.49498989301428, 25.360058412982617],
            [-109.14259669737466, 28.80127947277916],
            [-131.43765922331912, 30.449849209255976],
            [-151.96564296430557, 30.377724240163957],
            [-170.3422414483008, 28.70487634026508],
            [-186.22148514308574, 25.59555399277931],
            [-199.30306118222853, 21.253464092548718],
            [-209.338680888914, 15.915993207257115],
            [-216.137353158079, 9.847610324746126],
            [-219.56944606680233, 3.332612198121237],
            [-219.56944606680236, -3.3326121981211836],
            [-216.13735315807912, -9.847610324745984],
            [-209.33868088891418, -15.915993207256982],
            [-199.30306118222882, -21.253464092548608],
            [-186.22148514308608, -25.59555399277922],
            [-170.3422414483012, -28.70487634026503],
            [-151.96564296430603, -30.377724240163936],
            [-131.4376592233193, -30.44984920925598],
            [-109.14259669737483, -28.80127947277918],
            [-85.49498989301482, -25.360058412982713],
            [-60.93088366846411, -20.104809101399876],
            [-35.8987004566381, -13.066058415054773],
            [-10.849894962012549, -4.326283396313764],
            [13.770396811618006, 5.981327408808864],
            [37.53250774580571, 17.67537460559468],
            [60.03098762389965, 30.52995776128763],
            [80.89275722993366, 44.27951290089235],
            [99.78449074249441, 58.624669425782805],
            [116.41906660507497, 73.238993865505],
            [130.56094861550113, 87.77646657697112],
            [142.03038363329523, 101.87951973725588],
            [150.70632948799428, 115.18745115251089],
            [156.52805575868462, 127.34501885153378],
            [159.49539041751999, 138.01101637545955],
            [159.66761619222927, 146.86662823863696],
            [157.1610511972974, 153.6233692373121],
            [152.1453782078631, 158.0304200327442],
            [144.83881522228108, 159.8811845394494],
            [135.50224603290155, 159.01891181255633],
            [124.43245280519409, 155.34124596223353],
            [111.95461262288251, 148.80359165441584],
            [98.41423613814457, 139.42120943823232],
            [84.16873850529558, 127.26998386278345],
            [69.57884040345606, 112.48583745288374],
            [55.00000000000022, 95.26279441628851]
        ]

        self.flag=False
        self.current_goal_index=0

        self.goal_x,self.goal_y=self.goal_array[self.current_goal_index]

    def initial_reach_callback(self,initial_reach_msg:Bool):
        '''
        Purpose:
        ---
        set self.flag to true when all True is publisihed in initial_reach topic by initializer node '''
        self.flag=initial_reach_msg.data

    def update_goal(self):

        '''
        Purpose:
        ---
        Update the current goal position to the next position in the array.
        '''

        # D_goal_x,D_goal_y,=self.goal_array[self.current_goal_index]
        # self.goal_x=D_goal_x-250
        # self.goal_y=250-D_goal_y
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



        print("==================================================")
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

        # if v1>0:
        #     servo_1=47*v1+97
        # elif v1<0:
        #     servo_1=39*v1+87
        # else:
        #     servo_1=90


        
        # if v2>0:
        #     servo_2=42*v2+97
        # elif v2<0:
        #     servo_2=37*v2+87
        # else:
        #     servo_2=90



        # if v3>0:
        #     servo_3=40*v3+97
        # elif v3<0:
        #     servo_3=39*v3+87
        # else:
        #     servo_3=90

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
        v_theta =110*self.kp * error_theta

        
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
                self.get_logger().warn("completed run")
                time.sleep(1)
                rclpy.shutdown()
            else:
                if self.current_goal_index==0:
                    bot2_initial_msg=Bool()
                    bot2_initial_msg.data=True
                    self.bot2_initial_publisher.publish(bot2_initial_msg)
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
