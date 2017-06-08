/*
get "cmd_vel"
set "servo1" as direction
set "servo2" as accel
 */


#include "ros/ros.h"
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include  <std_msgs/UInt16.h>
#include <cmath>
using namespace std;

ros::Publisher servo1_pub,servo2_pub;
void cmdCallback (const geometry_msgs::Twist &cmd) {
  double acc=cmd.linear.x;
  double dir=cmd.angular.z;
  std_msgs::UInt16 msg;
  acc=90+acc*30;
  msg.data=max(60.0,min(120.0,acc));
  servo1_pub.publish(msg);
  dir=90+dir*20;
  msg.data=max(60.0,min(120.0,dir));
  servo2_pub.publish(msg);
}




int main(int argc, char **argv)
{

  ros::init(argc, argv, "cont_to_servo");
  ros::NodeHandle node_handle;
  ros::Rate loop_rate(30);
  ros::Subscriber sub = node_handle.subscribe("cmd_vel",2,cmdCallback);


  servo1_pub = node_handle.advertise<std_msgs::UInt16>("servo1", 1);
  servo2_pub = node_handle.advertise<std_msgs::UInt16>("servo2", 1);


  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

