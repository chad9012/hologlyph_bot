/*
 * Team Id: 2649
 * Author List: [Chandan Singh Chauhan]
 * Filename: microRos_bot3.ino
 * Theme: Hologlyph Bots
 * Functions: setup, loop, servo_init, RCCHECK, RCSOFTCHECK, error_loop, subscription_callback,
 *            pen_down_callback
 * Global Variables: subscriber, pen_down_subscriber, msg, pen_down_msg, executor, support,
 *                   allocator, node, servo1, servo2, servo3, micro_servo, disingaze
 */

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int64.h>
#include <std_msgs/msg/bool.h> // Include the Bool message type
#include <ESP32Servo.h>

rcl_subscription_t subscriber;
rcl_subscription_t pen_down_subscriber; // Subscriber for pen down topic
std_msgs__msg__Int64 msg;
std_msgs__msg__Bool pen_down_msg; // Message for pen down topic
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Assigning servo names
Servo servo1; 
Servo servo2; 
Servo servo3;
Servo micro_servo; 
int disingaze=90;

// Defining pins
#define servo1_pin  27
#define servo2_pin  26
#define servo3_pin  25
#define servo_pin   33
#define LED_PIN     2


/*
 * Function Name: servo_init
 * Input: None
 * Output: None
 * Logic: Initializes servo objects and attaches them to pins.
 * Example Call: servo_init();
 */

void servo_init(){
  servo1.attach(servo1_pin);
  servo2.attach(servo2_pin);
  servo3.attach(servo3_pin);

  micro_servo.attach(servo_pin);
}


/*
 * Function Name: RCCHECK
 * Input: fn - rcl_ret_t
 * Output: None
 * Logic: Checks if the return value of a function is RCL_RET_OK, if not, calls error_loop.
 * Example Call: RCCHECK(rcl_init(...));
 */
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}


/*
 * Function Name: RCSOFTCHECK
 * Input: fn - rcl_ret_t
 * Output: None
 * Logic: Checks if the return value of a function is RCL_RET_OK.
 * Example Call: RCSOFTCHECK(rcl_init(...));
 */

#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


/*
 * Function Name: error_loop
 * Input: None
 * Output: None
 * Logic: Blinks an LED to indicate an error state.
 * Example Call: error_loop();
 */

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

unsigned long lastMessageTimeInt64 = 0;
unsigned long lastMessageTimeBool = 0;

// Subscription callback for wheel velocities


/*
 * Function Name: subscription_callback
 * Input: msgin - const void *
 * Output: None
 * Logic: Callback function for processing received wheel velocities.
 * Example Call: Automatically called by the framework.
 */

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int64 * msg = (const std_msgs__msg__Int64 *)msgin;

  // Extracting the receiver number and velocity values
  int velocity_servo1 = (msg->data / 1000000000000) % 1000000;
  int velocity_servo2 = (msg->data / 1000000) % 1000000;
  int velocity_servo3 = msg->data % 1000000;
  float velocity1=velocity_servo1/1000;
  float velocity2=velocity_servo2/1000;
  float velocity3=velocity_servo3/1000;

  // Setting the velocities of servo1, servo2, and servo3
  servo1.write(velocity1);
  servo2.write(velocity2);
  servo3.write(velocity3);

  lastMessageTimeInt64 = millis();  // Update the time of the last received message
}

/*
 * Function Name: pen_down_callback
 * Input: msgin - const void *
 * Output: None
 * Logic: Callback function for processing pen down topic.
 * Example Call: Automatically called by the framework.
 */

// Subscription callback for pen down topic
void pen_down_callback(const void * msgin) {
  const std_msgs__msg__Bool * pen_down_msg = (const std_msgs__msg__Bool *)msgin;
  
  // Update disingaze based on the received message
  if (pen_down_msg->data) {
    disingaze = 0; // Set disingaze if pen is down
  } else {
    disingaze = 90; // Set disingaze if pen is up
  }
  lastMessageTimeBool = millis();
}

void setup() {
  set_microros_wifi_transports("ROBO-LAB", "roboticslab", "192.168.50.53", 8888);
  
  servo_init();
  pinMode(LED_PIN, OUTPUT);
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
 
  // Create node
  RCCHECK(rclc_node_init_default(&node, "microRos_bot2", "", &support));

  // Create subscriber for wheel velocities
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
    "bot2WheelVelocity"));

  // Create subscriber for pen down topic
  RCCHECK(rclc_subscription_init_default(
    &pen_down_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "pen2_down"));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &pen_down_subscriber, &pen_down_msg, &pen_down_callback, ON_NEW_DATA));
}

void loop()  {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  
  // Check if data has been received within the last 1 second (adjust as needed)
  if (millis() - lastMessageTimeInt64 > 80) {
    // Stop all motors when no data is received
    
    digitalWrite(LED_PIN, LOW);
    servo1.write(90);
    servo2.write(90);
    servo3.write(90);
  };

  if (millis() - lastMessageTimeBool > 1000) {
    // If no data received for 1 second, set servo to 90 degrees
    disingaze = 90;
  }
  
  // Set servo to disingaze value
  micro_servo.write(disingaze);


  
  delay(10);
}
