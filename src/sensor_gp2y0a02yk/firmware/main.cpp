/* 
 * rosserial IR Ranger
 * 
 * This example is calibrated for the Sharp GP2Y0A02YK, and
 * is inspired in the ROS Wiki rosserial IR Ranger example.
 * See http://wiki.ros.org/rosserial_arduino/Tutorials/IR%20Ranger
 */

#include <math.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>

/* Sensor typical response time in milliseconds */
const int TYPICAL_RESPONSE_TIME_MILLIS = 39;

ros::NodeHandle  nh;
sensor_msgs::Range output;
std_msgs::Int16 raw_output;
ros::Publisher pub_range( "scan", &output);
ros::Publisher pub_raw_data( "raw_data", &raw_output);

const int analog_pin = 0;
unsigned long range_timer;
const char frame_id[] = "/scan";


/*
 * getRawData() - samples the analog input from the ranger.
 */
int getRawData(int pin_num){
  return analogRead(pin_num);
}

/*
 * getDistance() - converts the raw ranger data to a distance.
 */
float getDistance(int raw_data){
  float a, b;
  float point_a[2] = { 1/1.5, 72 };
  float point_b[2] = { 1/0.3, 338 };
  float point_c[2] = { 1/0.2, 505 };
  float point_d[2] = { 1/0.15, 553 };

  if (raw_data < 72){
    return INFINITY;
  } else if (raw_data >= 72 && raw_data < 338){
    b = (point_b[1] - point_a[1]) / (point_b[0] - point_a[0]);
    a = point_a[1] - (b * point_a[0]);
    return b/(raw_data - a);
  } else if (raw_data >= 338 && raw_data < 505){
    b = (point_c[1] - point_b[1]) / (point_c[0] - point_b[0]);
    a = point_b[1] - (b * point_b[0]);
    return b/(raw_data - a);
  } else if (raw_data >= 505 && raw_data < 553){
    b = (point_d[1] - point_c[1]) / (point_d[0] - point_c[0]);
    a = point_c[1] - (b * point_c[0]);
    return b/(raw_data - a);
  } else {
    return NAN;
  }
}

void setup()
{
  nh.initNode();
  nh.advertise(pub_range);
  nh.advertise(pub_raw_data);

  output.header.frame_id = frame_id;
  output.radiation_type = output.INFRARED;
  output.field_of_view = 0.01;
  output.min_range = 0.15;
  output.max_range = 1.5;
}

void loop()
{
  // publish the voltage value every 39 milliseconds
  //   since it takes that long for the sensor to stabilize
  if ( (millis()-range_timer) > TYPICAL_RESPONSE_TIME_MILLIS){
    int raw_data = getRawData(analog_pin);
    raw_output.data = raw_data;
    output.header.stamp = nh.now();
    output.range = getDistance(raw_data);
    pub_raw_data.publish(&raw_output);
    pub_range.publish(&output);
    range_timer =  millis();
  }
  nh.spinOnce();
}
