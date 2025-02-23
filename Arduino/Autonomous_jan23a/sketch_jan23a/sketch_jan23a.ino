#include <ros.h>
#include <std_msgs/String.h>

const int LEFT_MOTOR_PWM = 5;  
const int LEFT_MOTOR_DIR = 3;  
const int RIGHT_MOTOR_PWM = 6;  
const int RIGHT_MOTOR_DIR = 7;  

// Parameters
const int MAX_PWM = 255;        // Maximum PWM value
const float MAX_LINEAR_VEL = 1.0;  // Max linear velocity in m/s
const float MAX_ANGULAR_VEL = 1.0; // Max angular velocity in rad/s
const float WHEEL_BASE = 0.95;      // Distance between wheels in meters

// ROS NodeHandle
ros::NodeHandle nh;

// Current command
String current_command = "";

void commandCallback(const std_msgs::String& msg) 
{
  current_command = msg.data;  // Directly assign data to current_command
}

// ROS Subscriber
ros::Subscriber<std_msgs::String> command_sub("command", commandCallback);


// Function to compute PWM and direction
void computeMotorCommands(float linear, float angular, int& left_pwm, int& right_pwm, int& left_dir, int& right_dir) 
{
  float left_speed = linear - (angular * WHEEL_BASE / 2.0);
  float right_speed = linear + (angular * WHEEL_BASE / 2.0);

  // Normalize speeds to PWM values
  left_pwm = constrain(abs(left_speed) / MAX_LINEAR_VEL * MAX_PWM, 0, MAX_PWM);
  right_pwm = constrain(abs(right_speed) / MAX_LINEAR_VEL * MAX_PWM, 0, MAX_PWM);

  // Determine directions
  left_dir = left_speed >= 0 ? HIGH : LOW;
  right_dir = right_speed >= 0 ? HIGH : LOW;
}


// Function to control the motors
void controlMotors(int left_pwm, int left_dir, int right_pwm, int right_dir) 
{
  digitalWrite(LEFT_MOTOR_DIR, left_dir);
  analogWrite(LEFT_MOTOR_PWM, left_pwm);

  digitalWrite(RIGHT_MOTOR_DIR, right_dir);
  analogWrite(RIGHT_MOTOR_PWM, right_pwm);
}


void turnLeft()
{
  int left_pwm, right_pwm, left_dir, right_dir;
  computeMotorCommands(0, MAX_ANGULAR_VEL, left_pwm, right_pwm, left_dir, right_dir); // Calculate for turning left
  controlMotors(left_pwm, left_dir, right_pwm, right_dir);
  delay(7000);  // Turn for 7 seconds
  controlMotors(0, LOW, 0, LOW);  // Stop motors
  delay(10000);  // Pause for 10 seconds
}


void turnRight() 
{
  int left_pwm, right_pwm, left_dir, right_dir;
  computeMotorCommands(0, -MAX_ANGULAR_VEL, left_pwm, right_pwm, left_dir, right_dir); // Calculate for turning right
  controlMotors(left_pwm, left_dir, right_pwm, right_dir);
  delay(7000);  // Turn for 7 seconds
  controlMotors(0, LOW, 0, LOW);  // Stop motors
  delay(10000);  // Pause for 10 seconds
}


void moveStraight() 
{
  int left_pwm, right_pwm, left_dir, right_dir;
  computeMotorCommands(MAX_LINEAR_VEL, 0, left_pwm, right_pwm, left_dir, right_dir); // Calculate for moving straight
  controlMotors(left_pwm, left_dir, right_pwm, right_dir);
}


void stopMotors() 
{
  controlMotors(0, LOW, 0, LOW);  // Stop all motors
}


void setup() 
{
  // Initialize motor driver pins
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);

  // Initialize ROS node
  nh.initNode();
  nh.subscribe(command_sub);
}


void loop() 
{
  // Always move straight by default
  moveStraight();

  if (current_command == "left") 
  {
    turnLeft();
    current_command = "";  // Reset command after execution
  } 
  else if (current_command == "right")
  {
    turnRight();
    current_command = "";  
  } 
  else if (current_command == "orange_cone_detected") 
  {
    stopMotors();  
    current_command = "";  
  }

  // Spin ROS node
  nh.spinOnce();

  delay(10);
}
