#include <Servo.h>
#include <ros.h>
#include <std_msgs/Char.h>

Servo servo1;  // First camera servo (horizontal)
Servo servo2;  // Second camera servo (vertical)
int servoPin1 = 6;  // Pin for first camera servo
int servoPin2 = 5;  // Pin for second camera servo
int pos1 = 90;  // Initial position for first camera
int pos2 = 90;  // Initial position for second camera

ros::NodeHandle nh;

void commandCallback(const std_msgs::Char& msg) {
  char key = msg.data;

  if (key == 'q' && pos1 < 180) {
    pos1 += 30;  // Move clockwise
    servo1.write(pos1);
    //nh.loginfo("1st camera moved clockwise");
  } 
  else if (key == 'a' && pos1 > 0) {
    pos1 -= 30;  // Move anticlockwise
    servo1.write(pos1);
    //nh.loginfo("1st camera moved anticlockwise");
  } 
  else if (key == 'e' && pos2 < 180) {
    pos2 += 30;  // Move clockwise
    servo2.write(pos2);
    //nh.loginfo("2nd camera moved clockwise");
  } 
  else if (key == 'd' && pos2 > 0) {
    pos2 -= 30;  // Move anticlockwise
    servo2.write(pos2);
    nh.loginfo("2nd camera moved anticlockwise");
  } 
  else if (key == 's') {
    //nh.loginfo("Motion stopped, cameras hold position");
  }
  

}
ros::Subscriber<std_msgs::Char> sub("camera_control", commandCallback);

void setup() {
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo1.write(pos1);
  servo2.write(pos2);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}
