/*
Author : Youssef Abbas
start  : 31/10/2021
Last   :   -
About  : Measure Count step of Rotary Encoder
*/
#include <ros.h>
#include <std_msgs/UInt32.h>

#define SIGNAL_A PA0
#define SIGNAL_B PA1

ros::NodeHandle nh;

std_msgs::UInt32 counter;
ros::Publisher pub("position", &counter);

void setup()
{
  counter.data = 0 ;
  nh.initNode();
  nh.advertise(pub);
  pinMode(SIGNAL_A, INPUT);
  pinMode(SIGNAL_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_A), ISR_A, RISING);
}

void loop()
{
  pub.publish( &counter );
  nh.spinOnce();
  delay(1000);

}

void ISR_A()
{
  if (digitalRead(SIGNAL_A) != digitalRead(SIGNAL_B))
    counter.data++  ;
  else
    counter.data--  ;
}

void ISR_B()
{
  if (digitalRead(SIGNAL_A) == digitalRead(SIGNAL_B))
    counter.data++   ;
  else
    counter.data--  ;
}
