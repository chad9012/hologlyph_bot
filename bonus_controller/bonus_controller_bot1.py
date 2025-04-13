'''
# Team ID:          2649
# Theme:            Hologlyph Bots
# Author List:      Chandan Singh Chauhan
# Filename:         bonus_controller_bot1.py
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
        self.current_goal_array_index=0
        self.number_of_countours=8
        

        self.Countour2=[[272, 419], [272, 418], [273, 417], [274, 416], [274, 415], [275, 414], [276, 413], [277, 412], [277, 411], [278, 410], [279, 409], [280, 408],
        [280, 407], [281, 406], [282, 406], [283, 406], [284, 405], [285, 405], [286, 404], [287, 404], [288, 404], [289, 404], [290, 403], [291, 403],
        [292, 403], [293, 403], [294, 403], [295, 404], [296, 404], [297, 404], [298, 404], [299, 405], [300, 405], [301, 405], [302, 406], [303, 406],
        [304, 407], [305, 408], [306, 408], [307, 409], [308, 410], [309, 411], [308, 412]]

        self.Countour4=[[301, 427], [301, 426], [301, 425], [301, 424], [301, 423], [302, 422], [302, 421], [302, 420], [303, 419], [303, 418], [304, 417], [305, 416],
        [306, 415], [307, 414], [307, 413], [308, 412], [309, 411], [310, 410], [311, 410], [312, 409], [313, 408], [314, 408], [315, 408], [316, 407],
        [317, 407], [318, 407], [319, 407], [320, 407], [321, 406], [322, 406], [323, 406], [324, 407], [325, 407], [326, 407], [327, 407], [328, 407],
        [329, 408], [330, 408], [331, 408], [332, 409], [333, 410], [334, 410], [335, 411], [336, 411], [337, 412], [337, 411], [338, 410], [339, 409],
        [340, 408], [341, 407], [342, 407], [343, 406], [344, 405], [345, 405], [346, 404], [347, 404], [348, 404], [349, 403], [350, 403], [351, 403],
        [352, 403], [353, 402], [354, 402], [355, 403], [356, 403], [357, 403], [358, 403], [359, 403], [360, 403], [361, 404], [362, 404], [363, 405],
        [364, 405], [365, 405], [366, 406], [367, 407], [368, 408], [369, 409], [370, 410], [371, 411], [372, 412], [373, 413], [373, 414], [374, 415]]

        self.Countour5=[[285, 351], [286, 351], [287, 350], [288, 349], [289, 349], [290, 348], [291, 348], [292, 347], [293, 346], [294, 346], [295, 345], [296, 345],
        [297, 344], [298, 344], [299, 343], [300, 343], [301, 343], [302, 342], [303, 342], [304, 341], [305, 341], [306, 340], [307, 340], [308, 340],
        [309, 339], [310, 339], [311, 338], [312, 338], [313, 338], [314, 338], [315, 337], [316, 337], [317, 337], [318, 336], [319, 336], [320, 336],
        [321, 335], [322, 335], [323, 335], [324, 335], [325, 335], [326, 334], [327, 334], [328, 334], [329, 334], [330, 333], [331, 333], [332, 333],
        [333, 333], [334, 333], [335, 333], [336, 333], [337, 333], [338, 333], [339, 333], [340, 333], [341, 333], [342, 333], [343, 333], [344, 333],
        [345, 333], [346, 333], [347, 333], [348, 333], [349, 333], [350, 334], [351, 334], [352, 334], [353, 334], [354, 335], [355, 335], [356, 335],
        [357, 335], [358, 336], [359, 336], [360, 337], [361, 337], [362, 338], [363, 338], [364, 339], [365, 340], [366, 340], [367, 341], [367, 342],
        [368, 342], [369, 343], [370, 344], [371, 345], [372, 346], [372, 347], [373, 348], [374, 349], [375, 350], [375, 351], [376, 352], [376, 353],
        [377, 354], [377, 355], [378, 356], [379, 357], [379, 358], [379, 359], [380, 360], [380, 361], [381, 362], [381, 363], [381, 364], [382, 365],
        [382, 366], [382, 367], [383, 368], [383, 369], [383, 370], [383, 371], [384, 372], [384, 373], [384, 374], [384, 375], [384, 376], [385, 377],
        [385, 378], [385, 379], [385, 380], [385, 381], [385, 382], [385, 383], [385, 384], [386, 385], [386, 386], [386, 387], [386, 388], [386, 389],
        [386, 390], [386, 391], [386, 392], [386, 393], [386, 394], [386, 395], [386, 396], [386, 397], [386, 398], [386, 399], [386, 400], [386, 401],
        [386, 402], [386, 403], [386, 404], [386, 405], [386, 406], [386, 407], [386, 408], [385, 409], [385, 410], [385, 411], [385, 412], [385, 413],
        [384, 414], [384, 415], [384, 416], [383, 417], [383, 418], [382, 419], [382, 420], [381, 421], [381, 422], [380, 423], [379, 424], [378, 425],
        [377, 426], [377, 427], [376, 428], [375, 429], [374, 429], [373, 430], [372, 431], [371, 432], [370, 432], [369, 433], [368, 433], [367, 434],
        [366, 434], [365, 435], [364, 435], [363, 436], [362, 436], [361, 437], [360, 437], [359, 437], [358, 437], [357, 438], [356, 438], [355, 438],
        [354, 438], [353, 438], [352, 439], [351, 439], [350, 439], [349, 439], [348, 439], [347, 439], [346, 440], [345, 440], [344, 440], [343, 440],
        [342, 440], [341, 440], [340, 440], [339, 440], [338, 440], [337, 440], [336, 441], [335, 441], [334, 441], [333, 441], [332, 441], [331, 441],
        [330, 441], [329, 441], [328, 441], [327, 441], [326, 441], [325, 441], [324, 441], [323, 441], [322, 441], [321, 441], [320, 441], [319, 441],
        [318, 441], [317, 441], [316, 441], [315, 441], [314, 441], [313, 441], [312, 441], [311, 441], [310, 441], [309, 441], [308, 441], [307, 440],
        [306, 440], [305, 440], [304, 440], [303, 440], [302, 440], [301, 440], [300, 440], [299, 440], [298, 440], [297, 439], [296, 439], [295, 439],
        [294, 439], [293, 439], [292, 439], [291, 438], [290, 438], [289, 438], [288, 438], [287, 438], [286, 437], [285, 437], [284, 437], [283, 436],
        [282, 436], [281, 436], [280, 435], [279, 435], [278, 434], [277, 434], [276, 433], [275, 433], [274, 432], [273, 431], [272, 431], [271, 430],
        [270, 429], [269, 428], [268, 427], [267, 426], [266, 425]]

        self.Countour11=[[413, 250], [414, 249], [415, 250], [415, 251], [415, 252], [415, 253], [415, 254], [415, 255], [415, 256], [415, 257], [415, 258], [415, 259],
        [415, 260], [415, 261], [415, 262], [415, 263], [415, 264], [415, 265], [415, 266], [414, 267], [414, 268], [414, 269], [414, 270], [414, 271],
        [414, 272], [414, 273], [413, 274], [413, 275], [413, 276], [413, 277], [413, 278], [413, 279], [413, 280], [413, 281], [412, 282], [412, 283],
        [412, 284], [412, 285], [411, 286], [411, 287], [411, 288], [411, 289], [411, 290], [410, 291], [410, 292], [410, 293], [410, 294], [409, 295],
        [409, 296], [409, 297], [408, 298], [408, 299], [408, 300], [407, 301], [407, 302], [407, 303], [406, 304], [406, 305], [406, 306], [405, 307],
        [405, 308], [405, 309], [404, 310], [404, 311], [403, 312], [403, 313], [403, 314], [402, 315], [402, 316], [401, 317], [401, 318], [400, 319],
        [400, 320], [400, 321], [399, 322], [399, 323], [398, 324], [398, 325], [397, 326], [397, 327], [396, 328], [395, 329], [395, 330], [394, 331],
        [394, 332], [393, 333], [393, 334], [392, 335], [392, 336], [391, 337], [390, 338], [390, 339], [389, 340], [388, 341], [388, 342], [387, 343],
        [386, 344], [386, 345], [385, 346], [384, 347], [384, 348], [383, 349], [382, 350], [381, 351], [380, 352], [380, 353]]

        self.Countour22=[[367, 115], [367, 114], [366, 113], [366, 112], [365, 111], [366, 110], [367, 110], [368, 110], [369, 110], [370, 110], [371, 110], [372, 110],
        [373, 110], [374, 111], [375, 111], [376, 111], [377, 111], [378, 111], [379, 111], [380, 112], [381, 112], [382, 112], [383, 112], [384, 113],
        [385, 113], [386, 113], [387, 113], [388, 113], [389, 114], [390, 114], [391, 114], [392, 114], [393, 115], [394, 115], [395, 115], [396, 116],
        [397, 116], [398, 116], [399, 116], [400, 117], [401, 117], [402, 117], [403, 118], [404, 118], [405, 118], [406, 118], [407, 119], [408, 119],
        [409, 119], [410, 120], [411, 120], [412, 121], [413, 121], [414, 121], [415, 122], [416, 122], [417, 123], [418, 123], [419, 123], [420, 124],
        [421, 124], [422, 125], [423, 125], [424, 126], [425, 126], [426, 127], [427, 128], [428, 128], [429, 129], [430, 129], [431, 130], [432, 131],
        [433, 132], [434, 133], [435, 133], [436, 134], [437, 135], [438, 136], [439, 137], [440, 138], [441, 139], [441, 140], [442, 141], [443, 142],
        [444, 143], [444, 144], [445, 145], [445, 146], [446, 147], [447, 148], [447, 149], [447, 150], [448, 151], [448, 152], [449, 153], [449, 154],
        [449, 155], [449, 156], [449, 157], [450, 158], [450, 159], [450, 160], [450, 161], [450, 162], [450, 163], [451, 164], [451, 165], [451, 166],
        [451, 167], [451, 168], [451, 169], [451, 170], [451, 171], [451, 172], [451, 173], [451, 174], [451, 175], [451, 176], [450, 177], [450, 178],
        [450, 179], [450, 180], [450, 181], [450, 182], [450, 183], [449, 184], [449, 185], [449, 186], [449, 187], [449, 188], [448, 189], [448, 190],
        [448, 191], [448, 192], [447, 193], [447, 194], [447, 195], [447, 196], [446, 197], [446, 198], [446, 199], [445, 200], [445, 201], [445, 202],
        [444, 203], [444, 204], [443, 205], [443, 206], [443, 207], [442, 208], [442, 209], [441, 210], [441, 211], [441, 212], [440, 213], [440, 214],
        [439, 215], [439, 216], [438, 217], [438, 218], [437, 219], [437, 220], [436, 221], [436, 222], [435, 223], [434, 224], [434, 225], [433, 226],
        [433, 227], [432, 228], [431, 229], [431, 230], [430, 231], [429, 232], [429, 233], [428, 234], [427, 235], [426, 236], [426, 237], [425, 238],
        [424, 239], [423, 240], [422, 241], [421, 242], [420, 243], [419, 244], [418, 245], [417, 246], [416, 247], [415, 248], [414, 249], [413, 250],
        [412, 251], [411, 251], [410, 252], [409, 253], [408, 254], [407, 255], [406, 256], [405, 256], [404, 257], [403, 258], [402, 259], [401, 259],
        [400, 260], [399, 260], [398, 261], [397, 261], [396, 262], [395, 263], [394, 263], [393, 264], [392, 264], [391, 264], [390, 265], [389, 265],
        [388, 266], [387, 266], [386, 266], [385, 266], [384, 267], [383, 267], [382, 267], [381, 268], [380, 268], [379, 268], [378, 268], [377, 269],
        [376, 269], [375, 269], [374, 269], [373, 269], [372, 269], [371, 269], [370, 269], [369, 269], [368, 270]]

        self.Countour16=[[215, 386], [214, 385], [213, 384], [213, 383], [213, 382], [213, 381], [213, 380], [213, 379], [213, 378], [213, 377], 
            [213, 376], [213, 375], [213, 374], [214, 373], [214, 372], [214, 371], [214, 370], [214, 369], [214, 368], [214, 367], 
            [214, 366], [214, 365], [214, 364], [214, 363], [215, 362], [215, 361], [215, 360], [215, 359], [215, 358], [215, 357], 
            [215, 356], [215, 355], [215, 354], [215, 353], [215, 352], [216, 351], [216, 350], [216, 349], [216, 348], [216, 347], 
            [216, 346], [216, 345], [216, 344], [216, 343], [216, 342], [216, 341], [216, 340], [216, 339], [216, 338], [216, 337], 
            [216, 336], [216, 335], [216, 334], [216, 333], [216, 332], [216, 331], [216, 330], [216, 329], [216, 328], [216, 327], 
            [216, 326], [216, 325], [216, 324], [216, 323], [216, 322], [215, 321], [215, 320], [215, 319], [216, 318], [216, 317], 
            [216, 316], [216, 315], [216, 314], [216, 313], [216, 312], [216, 311], [216, 310], [215, 309], [215, 308], [215, 307], 
            [215, 306], [215, 305], [214, 304], [214, 303], [214, 302], [214, 301], [214, 300], [214, 299], [214, 298], [214, 297], 
            [214, 296], [213, 295], [213, 294], [213, 293], [213, 292], [213, 291], [213, 290], [213, 289], [213, 288], [213, 287], 
            [213, 286], [213, 285], [212, 284], [212, 283], [212, 282], [212, 281], [212, 280], [211, 279], [211, 278], [211, 277], 
            [211, 276], [211, 275], [211, 274], [210, 273], [210, 272], [210, 271], [210, 270], [209, 269], [209, 268], [209, 267], 
            [209, 266], [209, 265], [208, 264], [208, 263], [208, 262], [208, 261], [207, 260], [207, 259], [207, 258], [206, 257], 
            [206, 256], [206, 255], [206, 254], [205, 253], [205, 252], [205, 251], [204, 250], [204, 249], [204, 248], [203, 247], 
            [203, 246], [202, 245], [202, 244], [202, 243], [202, 242], [202, 241], [203, 240], [204, 240], [205, 239]
            ]
        self.Countour14=[[192, 249], [193, 249], [194, 250], [195, 250], [196, 251], [197, 252], [198, 253], [199, 254], [200, 255], [201, 256], 
            [202, 257], [202, 258], [203, 259], [203, 260], [204, 261], [204, 262], [204, 263], [205, 264], [205, 265], [205, 266], 
            [205, 267], [205, 268], [205, 269], [204, 270], [204, 271], [204, 272], [204, 273], [204, 274], [203, 275], [203, 276], 
            [202, 277], [202, 278], [201, 279], [201, 280], [200, 281], [200, 282], [199, 283], [199, 284], [198, 285], [198, 286], 
            [197, 287], [197, 288], [196, 289], [195, 290], [195, 291], [194, 292], [193, 293], [193, 294], [192, 295], [191, 296], 
            [190, 297], [189, 298], [188, 299], [187, 300], [186, 301], [186, 302], [185, 303], [184, 304], [183, 305], [182, 306], 
            [181, 307], [180, 308], [179, 309], [178, 310], [177, 311], [176, 312], [175, 313], [174, 313], [173, 313], [172, 314], 
            [171, 314], [170, 314], [169, 315], [168, 315], [167, 314], [166, 314], [165, 313], [164, 312], [163, 311], [163, 310], 
            [162, 309], [162, 308], [162, 307], [162, 306], [161, 305], [161, 304], [161, 303], [161, 302], [160, 301], [160, 300], 
            [160, 299], [160, 298], [160, 297], [159, 296], [159, 295], [159, 294], [159, 293], [159, 292], [159, 291], [159, 290], 
            [159, 289], [159, 288], [159, 287], [159, 286], [159, 285], [159, 284], [159, 283], [159, 282], [159, 281], [159, 280], 
            [159, 279], [159, 278], [159, 277], [159, 276], [159, 275], [160, 274], [160, 273], [160, 272], [160, 271], [160, 270], 
            [161, 269], [161, 268], [161, 267], [161, 266], [161, 265], [162, 264], [162, 263], [162, 262], [162, 261], [163, 260], 
            [163, 259], [163, 258], [164, 257], [164, 256], [165, 255], [165, 254], [166, 253], [167, 252], [168, 251], [169, 250], 
            [170, 250], [171, 249], [172, 249], [173, 248], [174, 248], [175, 247], [176, 247], [177, 247], [178, 247], [179, 247], 
            [180, 247], [181, 246], [182, 246], [183, 246], [184, 247], [185, 247]
            ]
        self.Countour8=[[216, 315], [216, 316], [215, 317], [214, 317], [213, 316], [212, 316], [211, 316], [210, 316], [209, 315], [208, 315], 
            [207, 315], [206, 314], [205, 314], [204, 313], [203, 313], [202, 313], [201, 313], [200, 312], [199, 312], [198, 311], 
            [197, 311], [196, 310], [195, 310], [194, 309], [193, 309], [192, 308], [191, 308], [190, 308], [189, 307], [188, 306], 
            [187, 306], [186, 305], [185, 305], [184, 304], [185, 303]
            ]



        self.goal_array=self.Countour22
        self.point_jump=20
        self.flag=False
        self.current_goal_index=0
        D_goal_x,D_goal_y=self.goal_array[self.current_goal_index]
        self.goal_x=D_goal_x-250
        self.goal_y=250-D_goal_y
        # self.goal_x,self.goal_y=self.goal_array[self.current_goal_index]

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
        # self.goal_x,self.goal_y=self.goal_array[self.current_goal_index]
        
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
            self.goal_array=self.Countour11
            self.point_jump=10
        elif self.current_goal_array_index==2:
            self.goal_array=self.Countour5
            self.point_jump=15
        elif self.current_goal_array_index==3:
            self.goal_array=self.Countour2
            self.point_jump=5
        elif self.current_goal_array_index==4:
            self.goal_array=self.Countour4
            self.point_jump=5
        elif self.current_goal_array_index==5:
            self.goal_array=self.Countour8
            self.point_jump=5
        elif self.current_goal_array_index==6:
            self.goal_array=self.Countour16
            self.point_jump=10
        elif self.current_goal_array_index==7:
            self.goal_array=self.Countour14
            self.point_jump=5

        #now set goal with new goal_array
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
            #to cheeck if it is in last term of countour
            if self.current_goal_index >= len(self.goal_array)-1:
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
                    bot1_initial_msg=Bool()
                    bot1_initial_msg.data=True
                    self.bot1_initial_publisher.publish(bot1_initial_msg)
                #check if all the bot has reached it's initial positon (tell by initilizer node  by /initial_reach topic )
                if self.flag:
                    self.current_goal_index=self.current_goal_index+self.point_jump
                    #to check if adding self.point_jump to current_goal_index do not exceed the size of self.goal
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
