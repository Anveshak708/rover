#include <ros.h>
#include <geometry_msgs/Twist.h>

#define LEFT_MOTOR_PWM 5
#define LEFT_MOTOR_DIR 3
#define RIGHT_MOTOR_PWM 6
#define RIGHT_MOTOR_DIR 7

ros::NodeHandle nh;

float linear_vel = 0.0;
float angular_vel = 0.0;
const float wheel_sep = 0.95; 

const int MAX_PWM = 255;
const int MIN_PWM = -255;

int constrainPWM(int value) {
  if (value > MAX_PWM) return MAX_PWM;
  if (value < MIN_PWM) return MIN_PWM;
  return value;
}

void cmdVelCallback(const geometry_msgs::Twist& cmd_msg) {
  linear_vel = cmd_msg.linear.x;
  angular_vel = cmd_msg.angular.z;

  float Right_velocity = ((linear_vel * 2) + (angular_vel * wheel_sep)) / 2.0;
  float Left_velocity = ((linear_vel * 2) - (angular_vel * wheel_sep)) / 2.0;

  int right_pwm = constrainPWM(static_cast<int>(Right_velocity * 10000));
  int left_pwm = constrainPWM(static_cast<int>(Left_velocity * 10000));

  int left_dir = (left_pwm >= 0) ? HIGH : LOW;
  left_pwm = abs(left_pwm);

  int right_dir = (right_pwm >= 0) ? HIGH : LOW;
  right_pwm = abs(right_pwm);

  digitalWrite(LEFT_MOTOR_DIR, left_dir);
  analogWrite(LEFT_MOTOR_PWM, left_pwm);

  digitalWrite(RIGHT_MOTOR_DIR, right_dir);
  analogWrite(RIGHT_MOTOR_PWM, right_pwm);

}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmdVelCallback);

void setup() {

  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);

  nh.initNode();
  nh.subscribe(cmd_vel_sub);
}

void loop() {
  nh.spinOnce();

  delay(10);
}
