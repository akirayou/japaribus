/*
get "cmd_vel"
set "servo1" as direction
set "servo2" as accel
 */


#include "ros/ros.h"
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include  <std_msgs/Int8MultiArray.h>
#include <cmath>
using namespace std;

ros::Publisher servo_pub;
void cmdCallback (const geometry_msgs::Twist &cmd) {
  double acc=cmd.linear.x;
  double dir=cmd.angular.z;
  std_msgs::Int8MultiArray msg;
  msg.data.clear();

  acc=acc*2;
  msg.data.push_back((char)acc);
  dir*=-20;
  msg.data.push_back((char)dir);
  servo_pub.publish(msg);
}




int main(int argc, char **argv)
{

  ros::init(argc, argv, "cont_to_servo");
  ros::NodeHandle node_handle;
  ros::Rate loop_rate(30);
  ros::Subscriber sub = node_handle.subscribe("cmd_vel",2,cmdCallback);


  servo_pub = node_handle.advertise<std_msgs::Int8MultiArray>("indicator", 1);


  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

