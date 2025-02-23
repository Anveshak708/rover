#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>

// Function prototype
void commandCallback(const std_msgs::String& msg);

// Motor driver pins
const int LEFT_MOTOR_PWM = 5;  
const int LEFT_MOTOR_DIR = 3;  
const int RIGHT_MOTOR_PWM = 6;  
const int RIGHT_MOTOR_DIR = 7;  

// Encoder pins for motor 1
#define HALLSEN_1_A 2
#define HALLSEN_1_B 4

// Encoder pins for motor 2
#define HALLSEN_2_A 3
#define HALLSEN_2_B 5

// Parameters
const int MAX_PWM = 255;         // Maximum PWM value
const float MAX_LINEAR_VEL = 1.0;   // Max linear velocity in m/s
const float MAX_ANGULAR_VEL = 1.0;  // Max angular velocity in rad/s
const float WHEEL_BASE = 0.95;      // Distance between wheels in meters

// Encoder values
volatile long encoderValue_1 = 0;
volatile long encoderValue_2 = 0;
std_msgs::Int16 right_wheel_tick_count;
std_msgs::Int16 left_wheel_tick_count;

// ROS NodeHandle
ros::NodeHandle nh;

// ROS Publishers
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

// ROS Subscriber
ros::Subscriber<std_msgs::String> command_sub("command", commandCallback);

String current_command = "";

// Callback for receiving commands
void commandCallback(const std_msgs::String& msg) 
{
  current_command = msg.data;
}

// Interrupts for encoders
void updateEncoder_1() 
{
  encoderValue_1++;
}

void updateEncoder_2() 
{
  encoderValue_2++;
}

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

// Publish encoder ticks
void publishEncoderTicks()
{
  left_wheel_tick_count.data = encoderValue_1;
  right_wheel_tick_count.data = encoderValue_2;

  leftPub.publish(&left_wheel_tick_count);
  rightPub.publish(&right_wheel_tick_count);
}

// Turn left
void turnLeft()
{
  int left_pwm, right_pwm, left_dir, right_dir;
  computeMotorCommands(0, MAX_ANGULAR_VEL, left_pwm, right_pwm, left_dir, right_dir);

  unsigned long start_time = millis();
  while (millis() - start_time < 7000) // Turn for 7 seconds
  {
    controlMotors(left_pwm, left_dir, right_pwm, right_dir);
    publishEncoderTicks();
    nh.spinOnce();
    delay(10);
  }

  controlMotors(0, LOW, 0, LOW);  // Stop motors
  delay(10000);  // Pause for 10 seconds
}

// Turn right
void turnRight()
{
  int left_pwm, right_pwm, left_dir, right_dir;
  computeMotorCommands(0, -MAX_ANGULAR_VEL, left_pwm, right_pwm, left_dir, right_dir);

  unsigned long start_time = millis();
  while (millis() - start_time < 7000) // Turn for 7 seconds
  {
    controlMotors(left_pwm, left_dir, right_pwm, right_dir);
    publishEncoderTicks();
    nh.spinOnce();
    delay(10);
  }

  controlMotors(0, LOW, 0, LOW);  // Stop motors
  delay(10000);  // Pause for 10 seconds
}

void moveStraight() 
{
  int left_pwm, right_pwm, left_dir, right_dir;
  computeMotorCommands(MAX_LINEAR_VEL, 0, left_pwm, right_pwm, left_dir, right_dir);
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

  // Initialize encoders
  attachInterrupt(digitalPinToInterrupt(HALLSEN_1_A), updateEncoder_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALLSEN_2_A), updateEncoder_2, CHANGE);

  // Initialize ROS node
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.subscribe(command_sub);
}

void loop() 
{
  moveStraight(); // Default motion

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
