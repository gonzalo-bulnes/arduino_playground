/* 
 * rosserial IR Ranger
 * 
 * This example is calibrated for the Sharp GP2Y0A02YK, and
 * is inspired in the ROS Wiki rosserial IR Ranger example.
 * See http://wiki.ros.org/rosserial_arduino/Tutorials/IR%20Ranger
 */

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>

/* Sensor typical response time in milliseconds */
const int TYPICAL_RESPONSE_TIME_MILLIS = 39;

ros::NodeHandle  nh;
std_msgs::Int16 raw_output;
ros::Publisher pub_range( "raw_data", &raw_output);

const int analog_pin = 0;
unsigned long range_timer;

/*
 * getRange() - samples the analog input from the ranger.
 */
int getRange(int pin_num){
  return analogRead(pin_num);
}

void setup()
{
  nh.initNode();
  nh.advertise(pub_range);
}

void loop()
{
  // publish the voltage value every 39 milliseconds
  //   since it takes that long for the sensor to stabilize
  if ( (millis()-range_timer) > TYPICAL_RESPONSE_TIME_MILLIS){
    raw_output.data = getRange(analog_pin);
    pub_range.publish(&raw_output);
    range_timer =  millis();
  }
  nh.spinOnce();
}
