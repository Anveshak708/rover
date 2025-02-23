#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

// For motor 1
#define HALLSEN_1_A 2
#define HALLSEN_1_B 4

// For motor 2
#define HALLSEN_2_A 3
#define HALLSEN_2_B 5

volatile long encoderValue_1 = 0;
volatile long encoderValue_2 = 0;

boolean Direction_left = true;
boolean Direction_right = true;

const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);

std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

void updateEncoder_1() 
{
  encoderValue_1++;
}

void updateEncoder_2() 
{
  encoderValue_2++;
}

void right_wheel_tick() 
{
  int val = digitalRead(HALLSEN_2_B);

  Direction_right = (val == LOW);

  if (Direction_right) 
  {
    if (right_wheel_tick_count.data == encoder_maximum) 
    {
      right_wheel_tick_count.data = encoder_minimum;
    } 
    else 
    {
      right_wheel_tick_count.data++;
    }
  } 
  else 
  {
    if (right_wheel_tick_count.data == encoder_minimum)
    {
      right_wheel_tick_count.data = encoder_maximum;
    } 
    else 
    {
      right_wheel_tick_count.data--;
    }
  }
}

void left_wheel_tick() 
{
  int val = digitalRead(HALLSEN_1_B);

  Direction_left = (val != LOW);

  if (Direction_left) 
  {
    if (left_wheel_tick_count.data == encoder_maximum) 
    {
      left_wheel_tick_count.data = encoder_minimum;
    } 
    else 
    {
      left_wheel_tick_count.data++;
    }
  } 
  else 
  {
    if (left_wheel_tick_count.data == encoder_minimum) 
    {
      left_wheel_tick_count.data = encoder_maximum;
    } 
    else 
    {
      left_wheel_tick_count.data--;
    }
  }
}

void setup() 
{
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);

  attachInterrupt(digitalPinToInterrupt(HALLSEN_1_A), updateEncoder_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALLSEN_2_A), updateEncoder_2, CHANGE);
}

void loop() 
{
  nh.spinOnce();
  delay(10);
}
