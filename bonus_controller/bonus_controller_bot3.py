'''
# Team ID:          2649
# Theme:            Hologlyph Bots
# Author List:      Chandan Singh Chauhan
# Filename:         bonus_controller_bot3.py
# Functions:        update_goal,update_goal_array, inverse_kinematics, convert_to_servo_scaled publish_data, current_pose_subscription_callback
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
    current_goal_array_index: int
        tells at which number of contour we are in 
    self.number_of_countours: int
        tells how many countours will be drawn by this bot
    self.point_jump: int 
        give the number of goal points to jump in each of countour  
    '''

    def __init__(self):
        super().__init__("controller_bot3")
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

        self.current_goal_array_index=0
        self.number_of_countours=7
        self.point_jump=5

        self.Countour1=[[164, 430], [163, 429], [163, 428], [163, 427], [163, 426], [163, 425], [164, 424], [164, 423], [164, 422], [164, 421],
        [165, 420], [165, 419], [165, 418], [166, 417], [167, 416], [168, 415], [169, 414], [169, 413], [170, 412], [171, 411],
        [172, 410], [173, 410], [174, 409], [175, 409], [176, 408], [177, 408], [178, 407], [179, 407], [180, 407], [181, 407],
        [182, 407], [183, 407], [184, 406], [185, 406], [186, 407], [187, 407], [188, 407], [189, 407], [190, 407], [191, 408],
        [192, 408], [193, 408], [194, 409], [195, 410], [196, 410], [197, 411], [198, 412], [199, 412], [200, 411], [200, 410],
        [201, 409], [202, 408], [203, 409], [203, 410], [204, 411], [205, 412], [205, 413], [206, 414], [205, 415], [204, 416],
        [204, 417], [203, 418], [204, 419], [204, 420], [204, 421], [205, 422], [205, 423], [205, 424], [205, 425], [205, 426],
        [205, 427], [205, 428], [205, 429]]

        self.Countour3=[[134, 420], [134, 419], [135, 418], [135, 417], [135, 416], [136, 415], [136, 414], [137, 413], [138, 412], [138, 411],
        [139, 410], [140, 409], [141, 408], [142, 408], [143, 407], [144, 406], [146, 405], [147, 405], [148, 404], [149, 404],
        [150, 404], [151, 404], [152, 403], [153, 403], [155, 403], [156, 403], [157, 404], [158, 404], [159, 404], [160, 404],
        [161, 404], [162, 405], [163, 405], [164, 406], [165, 406], [166, 407], [167, 407], [168, 408], [169, 409], [170, 410],
        [171, 411]]

        self.Countour6=[[230, 434], [229, 435], [228, 435], [227, 436], [226, 436], [225, 436], [224, 437], [223, 437], [222, 437], [221, 437],
        [220, 438], [219, 438], [218, 438], [217, 438], [216, 439], [215, 439], [214, 439], [213, 439], [211, 439], [210, 440],
        [209, 440], [208, 440], [207, 440], [206, 440], [205, 440], [204, 440], [203, 440], [202, 440], [200, 440], [199, 441],
        [198, 441], [197, 441], [196, 441], [195, 441], [194, 441], [192, 441], [191, 441], [190, 441], [188, 441], [187, 441],
        [186, 441], [185, 441], [184, 441], [183, 441], [182, 441], [180, 441], [179, 441], [178, 441], [177, 441], [176, 441],
        [175, 441], [174, 441], [173, 441], [172, 441], [171, 440], [169, 440], [168, 440], [167, 440], [166, 440], [165, 440],
        [164, 440], [163, 440], [162, 440], [161, 439], [160, 439], [159, 439], [158, 439], [157, 439], [155, 439], [154, 438],
        [153, 438], [152, 438], [151, 438], [150, 437], [149, 437], [148, 437], [146, 436], [145, 436], [144, 435], [143, 435],
        [142, 434], [140, 434], [139, 433], [138, 432], [137, 432], [136, 431], [135, 430], [133, 429], [132, 428], [131, 427],
        [129, 425], [129, 424], [127, 422], [126, 421], [126, 420], [126, 419], [125, 418], [124, 417], [124, 415], [123, 414],
        [123, 412], [123, 411], [122, 409], [122, 408], [122, 406], [122, 405], [122, 404], [121, 403], [121, 402], [121, 401],
        [121, 400], [121, 399], [121, 398], [121, 397], [121, 396], [121, 395], [122, 394], [122, 393], [122, 392], [122, 391],
        [122, 390], [122, 389], [122, 388], [122, 387], [122, 386], [122, 385], [122, 384], [122, 383], [122, 382], [123, 381],
        [123, 380], [123, 379], [123, 378], [123, 376], [123, 375], [124, 374], [124, 373], [124, 372], [124, 371], [125, 370],
        [125, 369], [125, 368], [125, 367], [126, 365], [126, 364], [127, 363], [128, 361], [128, 360], [128, 359], [129, 358],
        [129, 357], [130, 356], [130, 355], [130, 354], [131, 353], [132, 352], [133, 351], [133, 350], [134, 349], [135, 348],
        [136, 346], [137, 345], [138, 344], [139, 343], [140, 342], [141, 342], [142, 341], [143, 340], [144, 339], [145, 339],
        [146, 338], [147, 337], [148, 337], [149, 336], [150, 336], [151, 335], [152, 335], [153, 335], [154, 334], [155, 334],
        [156, 334], [157, 334], [158, 333], [159, 333], [161, 333], [162, 333], [163, 333], [164, 333], [165, 333], [166, 333],
        [167, 333], [169, 333], [170, 333], [171, 333], [172, 333], [173, 333], [174, 333], [175, 333], [176, 333], [177, 333],
        [178, 333], [179, 334], [180, 334], [181, 334], [183, 335], [184, 335], [185, 335], [186, 335], [187, 335], [188, 336],
        [189, 336], [190, 336], [191, 337], [192, 337], [194, 338], [195, 338], [196, 338], [197, 338], [198, 339], [199, 339],
        [200, 340], [201, 340], [203, 341], [204, 341], [205, 342], [206, 342], [207, 343], [208, 343], [209, 343], [210, 344],
        [211, 344]]

        self.Countour12=[[126, 361], [125, 360], [124, 359], [123, 358], [123, 357], [122, 356], [121, 355], [120, 354], [119, 353], [118, 352],
        [118, 351], [117, 350], [116, 349], [115, 348], [115, 347], [114, 346], [113, 345], [113, 344], [112, 343], [111, 342],
        [110, 341], [110, 340], [109, 339], [109, 338], [108, 337], [107, 336], [107, 335], [106, 334], [105, 333], [105, 332],
        [105, 331], [104, 330], [103, 329], [103, 328], [102, 327], [102, 326], [101, 325], [101, 324], [100, 323], [100, 322],
        [99, 321], [99, 320], [98, 319], [98, 318], [97, 317], [97, 316], [97, 315], [96, 314], [96, 313], [95, 312],
        [95, 311], [95, 310], [94, 309], [94, 308], [93, 307], [93, 306], [93, 305], [92, 304], [92, 303], [92, 302],
        [91, 301], [91, 300], [91, 299], [90, 298], [90, 297], [90, 296], [90, 295], [89, 294], [89, 293], [89, 292],
        [89, 291], [88, 290], [88, 289], [88, 288], [88, 287], [87, 286], [87, 285], [87, 284], [87, 283], [87, 282],
        [86, 281], [86, 280], [86, 279], [86, 278], [86, 277], [85, 276], [85, 275], [85, 274], [85, 273], [85, 272],
        [85, 271], [85, 270], [85, 269], [84, 268], [84, 267], [84, 266], [84, 265], [84, 264], [84, 263], [84, 262],
        [84, 261], [84, 260], [84, 259], [84, 258], [84, 257], [84, 256], [84, 255], [84, 254], [84, 253], [84, 252],
        [84, 251], [84, 250], [84, 249], [84, 248], [85, 247]]

        self.Countour23=[[144, 268], [143, 269], [142, 269], [141, 269], [140, 269], [139, 269], [138, 269], [137, 269], [136, 269], [135, 269],
        [134, 269], [133, 269], [132, 269], [131, 269], [130, 269], [129, 269], [128, 269], [127, 269], [126, 269], [125, 269],
        [124, 269], [123, 268], [122, 268], [121, 268], [120, 268], [119, 267], [118, 267], [117, 267], [116, 267], [115, 266],
        [114, 266], [113, 266], [112, 265], [111, 265], [110, 264], [109, 264], [108, 264], [107, 263], [106, 263], [105, 262],
        [104, 262], [103, 261], [102, 260], [101, 260], [100, 259], [99, 259], [98, 258], [97, 257], [96, 257], [95, 256],
        [94, 255], [93, 255], [92, 254], [91, 253], [90, 252], [89, 251], [88, 250], [87, 249], [86, 248], [85, 247],
        [84, 246], [83, 245], [82, 245], [81, 244], [81, 243], [80, 242], [79, 241], [78, 240], [77, 239], [76, 238],
        [75, 237], [74, 236], [74, 235], [73, 234], [72, 233], [72, 232], [71, 231], [70, 230], [69, 229], [69, 228],
        [68, 227], [68, 226], [67, 225], [66, 224], [66, 223], [65, 222], [65, 221], [64, 220], [64, 219], [63, 218],
        [63, 217], [62, 216], [62, 215], [61, 214], [61, 213], [60, 212], [60, 211], [59, 210], [59, 209], [59, 208],
        [58, 207], [58, 206], [57, 205], [57, 204], [57, 203], [56, 202], [56, 201], [56, 200], [55, 199], [55, 198],
        [55, 197], [54, 196], [54, 195], [54, 194], [53, 193], [53, 192], [53, 191], [53, 190], [53, 189], [52, 188],
        [52, 187], [52, 186], [52, 185], [51, 184], [51, 183], [51, 182], [51, 181], [51, 180], [51, 179], [51, 178],
        [51, 177], [50, 176], [50, 175], [50, 174], [50, 173], [50, 172], [50, 171], [50, 170], [50, 169], [50, 168],
        [50, 167], [50, 166], [50, 165], [50, 164], [50, 163], [50, 162], [51, 161], [51, 160], [51, 159], [51, 158],
        [51, 157], [51, 156], [52, 155], [52, 154], [52, 153], [53, 152], [53, 151], [54, 150], [54, 149], [54, 148],
        [55, 147], [55, 146], [56, 145], [57, 144], [57, 143], [58, 142], [59, 141], [59, 140], [60, 139], [61, 138],
        [62, 137], [63, 136], [64, 135], [65, 134], [66, 133], [67, 132], [68, 132], [69, 131], [70, 130], [71, 129],
        [72, 129], [73, 128], [74, 128], [75, 127], [76, 126], [77, 126], [78, 125], [79, 125], [80, 124], [81, 124],
        [82, 123], [83, 123], [84, 123], [85, 122], [86, 122], [87, 121], [88, 121], [89, 121], [90, 120], [91, 120],
        [92, 119], [93, 119], [94, 119], [95, 118], [96, 118], [97, 118], [98, 118], [99, 117], [100, 117], [101, 117],
        [102, 116], [103, 116], [104, 116], [105, 115], [106, 115], [107, 115], [108, 115], [109, 114], [110, 114], [111, 114],
        [112, 114], [113, 113], [114, 113], [115, 113], [116, 113], [117, 113], [118, 112], [119, 112], [120, 112], [121, 112],
        [122, 111], [123, 111], [124, 111], [125, 111]]

        self.Countour21=[[212, 128], [213, 128], [214, 128], [215, 128], [216, 129], [217, 129], [218, 129], [219, 129], [220, 129], [221, 130], 
            [222, 130], [223, 130], [224, 131], [225, 131], [226, 132], [227, 132], [228, 133], [229, 133], [230, 134], [231, 134], 
            [232, 135], [233, 136], [234, 137], [235, 138], [236, 138], [237, 139], [238, 140], [239, 141], [239, 142], [240, 143], 
            [241, 144], [242, 145], [242, 146], [243, 147], [243, 148], [244, 149], [245, 150], [245, 151], [245, 152], [246, 153], 
            [246, 154], [246, 155], [247, 156], [247, 157], [247, 158], [247, 159], [247, 160], [248, 161], [248, 162], [248, 163], 
            [248, 164], [248, 165], [248, 166], [248, 167], [248, 168], [248, 169], [248, 170], [247, 171], [247, 172], [247, 173], 
            [247, 174], [247, 175], [246, 176], [246, 177], [246, 178], [245, 179], [245, 180], [245, 181], [244, 182], [244, 183], 
            [243, 184], [242, 185], [242, 186], [241, 187], [240, 188], [239, 189], [239, 190], [238, 191], [237, 192], [236, 193], 
            [235, 194], [234, 194], [233, 195], [232, 196], [231, 197], [230, 197], [229, 198], [228, 199], [227, 199], [226, 200], 
            [225, 200], [224, 200], [223, 201], [222, 201], [221, 201], [220, 202], [219, 202], [218, 202], [217, 202], [216, 202], 
            [215, 203], [214, 203], [213, 203], [212, 203], [211, 203], [210, 203], [209, 203], [208, 203], [207, 203], [206, 203], 
            [205, 202], [204, 202], [203, 202], [202, 202], [201, 202], [200, 201], [199, 201], [198, 201], [197, 200], [196, 200], 
            [195, 200], [194, 199], [193, 198], [192, 198], [191, 197], [190, 197], [189, 196], [188, 195], [187, 194], [186, 194], 
            [185, 193], [184, 192], [183, 191], [183, 190], [182, 189], [181, 188], [180, 187], [179, 186], [179, 185], [178, 184], 
            [178, 183], [177, 182], [177, 181], [176, 180], [176, 179], [175, 178], [175, 177], [175, 176], [174, 175], [174, 174], 
            [174, 173], [174, 172], [174, 171], [173, 170], [173, 169], [173, 168], [173, 167], [173, 166], [173, 165], [173, 164], 
            [173, 163], [173, 162], [173, 161], [174, 160], [174, 159], [174, 158], [174, 157], [174, 156], [175, 155], [175, 154], 
            [175, 153], [176, 152], [176, 151], [177, 150], [177, 149], [178, 148], [178, 147], [179, 146], [179, 145], [180, 144], 
            [181, 143], [182, 142], [183, 141], [183, 140], [184, 139], [185, 138], [186, 138], [187, 137], [188, 136], [189, 135], 
            [190, 134], [191, 134], [192, 133], [193, 133], [194, 132], [195, 132], [196, 131], [197, 131], [198, 130], [199, 130], 
            [200, 130], [201, 129], [202, 129], [203, 129], [204, 129], [205, 129], [206, 128], [207, 128], [208, 128]]

        self.Countour19=[[220, 158], [221, 158], [222, 159], [223, 159], [224, 159], [225, 160], [226, 161], [227, 162], [228, 163], [229, 164], 
            [230, 165], [230, 166], [230, 167], [231, 168], [231, 169], [231, 170], [231, 171], [231, 172], [231, 173], [231, 174], 
            [230, 175], [230, 176], [230, 177], [229, 178], [228, 179], [228, 180], [227, 181], [226, 181], [225, 182], [224, 183], 
            [223, 183], [222, 184], [221, 184], [220, 184], [219, 184], [218, 184], [217, 184], [216, 183], [215, 183], [214, 183], 
            [213, 182], [212, 181], [211, 180], [210, 179], [209, 178], [209, 177], [208, 176], [208, 175], [207, 174], [207, 173], 
            [207, 172], [207, 171], [207, 170], [207, 169], [207, 168], [208, 167], [208, 166], [209, 165], [209, 164], [210, 163], 
            [211, 162], [212, 161], [213, 160], [214, 159], [215, 159], [216, 159], [217, 158], [218, 158], [219, 159]]

        self.Countour17=[[223, 227], [223, 226], [223, 225], [224, 224], [225, 223], [226, 222], [227, 221], [228, 220], [229, 219], [230, 219], 
            [231, 218], [232, 217], [233, 217], [234, 216], [235, 216], [236, 215], [237, 215], [238, 214], [239, 214], [240, 214], 
            [241, 214], [242, 214], [243, 213], [244, 213], [245, 213], [246, 213], [247, 213], [248, 213], [249, 213], [250, 213], 
            [251, 213], [252, 213], [253, 213], [254, 213], [255, 214], [256, 214], [257, 214], [258, 214], [259, 214], [260, 215], 
            [261, 215], [262, 216], [263, 216], [264, 216], [265, 217], [266, 217], [267, 218], [268, 218], [269, 219], [270, 219], 
            [271, 220], [272, 221], [273, 222], [274, 222], [275, 223]
            ]
        
        self.goal_array=self.Countour3
        
        self.flag=False
        self.current_goal_index=0
        D_goal_x,D_goal_y=self.goal_array[self.current_goal_index]
        self.goal_x=D_goal_x-250
        self.goal_y=250-D_goal_y


    def inital_reach_callback(self,initial_reach_msg:Bool):
        self.flag=initial_reach_msg.data


    def update_goal(self):

        '''
        Purpose:
        ---
        Update the current goal position to the next position in the array.
        '''

        D_goal_x,D_goal_y,=self.goal_array[self.current_goal_index]
        self.goal_x=D_goal_x-250
        self.goal_y=250-D_goal_y

        
        self.goal_theta=0


    def update_goal_array(self):


        '''
        Purpose:
        ---
        Update the current goal array to the next goal array (each goal array is a countour )
        and for each countour their is specific point_jump means if the countour .
        '''


        self.current_goal_index=0
        if self.current_goal_array_index==1:
            self.goal_array=self.Countour1
            self.point_jump=5
        elif self.current_goal_array_index==2:
            self.goal_array=self.Countour6
            self.point_jump=10
        elif self.current_goal_array_index==3:
            self.goal_array=self.Countour12
            self.point_jump=15
        elif self.current_goal_array_index==4:
            self.goal_array=self.Countour23
            self.point_jump=20
        elif self.current_goal_array_index==5:
            self.goal_array=self.Countour21
            self.point_jump=10            
        elif self.current_goal_array_index==6:
            self.goal_array=self.Countour19
            self.point_jump=5



        D_goal_x,D_goal_y,=self.goal_array[self.current_goal_index]
        self.goal_x=D_goal_x-250
        self.goal_y=250-D_goal_y

        return

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
            #if it reached a goal points
            print("goal_point reach")
            if self.current_goal_index == len(self.goal_array)-1:
                print("countour completed")
                #check if it is the last term of last countour
                #send bool msg to stopper to stop the evaluator as this bot has completed it's run 
             
                if self.current_goal_array_index==self.number_of_countours-1:

                    bool_msg=Bool()
                    bool_msg.data=True
                    self.completion_publisher.publish(bool_msg)
                    time.sleep(1)
                    self.get_logger().warn("completed run")
                    rclpy.shutdown()

                else:
                    #occur when its is last term of a countour (goal_array) but now the last term of last countour
                    self.current_goal_array_index=self.current_goal_array_index+1
                    self.update_goal_array()
            else:
                #goal to the next goal it has reached the end of list
                # if it reached in the starting position send bool msg to initilizer to send msg that this bot has reached to it's initial position 
          
                if self.current_goal_index==0:
                    bot3_initial_msg=Bool()
                    bot3_initial_msg.data=True
                    self.bot3_initial_publisher.publish(bot3_initial_msg)
                #check if all the bot has reached it's initial positon (tell by initilizer node  by /initial_reach topic )

                if (self.flag):
                    self.current_goal_index=self.current_goal_index+self.point_jump
                    if self.current_goal_index>len(self.goal_array)-1:
                        self.current_goal_index=len(self.goal_array)-1
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

