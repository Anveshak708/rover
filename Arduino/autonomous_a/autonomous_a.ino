#include <ros.h>
#include <std_msgs/String.h>

const int LEFT_MOTOR_PWM = 5;  
const int LEFT_MOTOR_DIR = 3;  
const int RIGHT_MOTOR_PWM = 6;  
const int RIGHT_MOTOR_DIR = 7;  

// Parameters
const int MAX_PWM = 255;        // Maximum PWM value
const float MAX_LINEAR_VEL = 4.0;  // Max linear velocity in m/s
const float MAX_ANGULAR_VEL = 2.0; // Max angular velocity in rad/s
const float WHEEL_BASE = 0.6;      // Distance between wheels in meters

// Velocity variables
float linear_vel = 2.0;
float angular_vel = 0.0;

int left_pwm, right_pwm;
int left_dir, right_dir;

// ROS NodeHandle
ros::NodeHandle nh;

// Callback to process cmd_vel messages
void commandCallback(const std_msgs::String& msg) {
  String command = msg.data;

  if (command== "left"){
    linear_vel=0.0;
    angular_vel=-1.0;
    computeMotorCommands(linear_vel, angular_vel, left_pwm, right_pwm, left_dir, right_dir);
    delay(500);
    linear_vel=0.0;
    angular_vel=0.0;
    computeMotorCommands(linear_vel, angular_vel, left_pwm, right_pwm, left_dir, right_dir);
    delay(1000);
    linear_vel = 2.0;
    angular_vel = 0.0;
    computeMotorCommands(linear_vel, angular_vel, left_pwm, right_pwm, left_dir, right_dir);

    
  }
  else if (command=="right"){
    linear_vel==0.0;
    angular_vel=1.0;
    computeMotorCommands(linear_vel, angular_vel, left_pwm, right_pwm, left_dir, right_dir);
    delay(500);
    linear_vel=0.0;
    angular_vel=0.0;
    computeMotorCommands(linear_vel, angular_vel, left_pwm, right_pwm, left_dir, right_dir);
    delay(1000);
    linear_vel = 2.0;
    angular_vel = 0.0;
    computeMotorCommands(linear_vel, angular_vel, left_pwm, right_pwm, left_dir, right_dir);

  }
}

// ROS Subscriber
ros::Subscriber<std_msgs::String> sub("detection", commandCallback);

// Function to compute PWM and direction
void computeMotorCommands(float linear, float angular, int& left_pwm, int& right_pwm, int& left_dir, int& right_dir) {
  float left_speed = linear - (angular * WHEEL_BASE / 2.0);
  float right_speed = linear + (angular * WHEEL_BASE / 2.0);

  // Normalize speeds to PWM values
  left_pwm = constrain(abs(left_speed) / MAX_LINEAR_VEL * MAX_PWM, 0, MAX_PWM);
  right_pwm = constrain(abs(right_speed) / MAX_LINEAR_VEL * MAX_PWM, 0, MAX_PWM);

  // Determine directions
  left_dir = left_speed >= 0 ? HIGH : LOW;
  right_dir = right_speed >= 0 ? HIGH : LOW;
}

void setup() {
  // Initialize motor driver pins
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);

  // Initialize ROS node
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  int left_pwm, right_pwm;
  int left_dir, right_dir;

  // Compute motor commands
  computeMotorCommands(linear_vel, angular_vel, left_pwm, right_pwm, left_dir, right_dir);

  // Send PWM and direction to motor drivers
  digitalWrite(LEFT_MOTOR_DIR, left_dir);
  analogWrite(LEFT_MOTOR_PWM, left_pwm);

  digitalWrite(RIGHT_MOTOR_DIR, right_dir);
  analogWrite(RIGHT_MOTOR_PWM, right_pwm);

  // Spin ROS node
  nh.spinOnce();

  // Delay for a short time
  delay(10);
}
