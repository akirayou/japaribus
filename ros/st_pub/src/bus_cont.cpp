/*
get "target"/TF for get direction 


set direc and acc



 */


#include "ros/ros.h"
#include <ros/package.h>
#include <sstream>
#include <fstream>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt16.h>
double oldDir=0;
bool isSafe=false;
ros::Publisher cmd_velPub;
void dangerCallback(const std_msgs::UInt16 msg){
  geometry_msgs::Twist cmd;
  cmd.linear.x  = cmd.angular.z = 0;
  if(9<msg.data){
    isSafe=false;   
    cmd.linear.x  = 0;
    cmd.angular.z = oldDir;
    cmd_velPub.publish(cmd);
  }else{
    isSafe=true;
  }
}

void initBus(tf2_ros::TransformBroadcaster  & br){
  geometry_msgs::TransformStamped brtf; 
  brtf.header.stamp = ros::Time::now();
  brtf.header.frame_id = "world";
  brtf.child_frame_id ="bus";
  brtf.transform.translation.x = 0;
  brtf.transform.translation.y = 0;
  brtf.transform.translation.z = 0.0;
  brtf.transform.rotation.x = 0;
  brtf.transform.rotation.y = 0;
  brtf.transform.rotation.z = 0;
  brtf.transform.rotation.w = 1;
  br.sendTransform(brtf);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "bus_cont");
  ros::NodeHandle node_handle;
  ros::Rate loop_rate(30);
  static tf2_ros::TransformBroadcaster br;
  initBus(br);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfl(tfBuffer);
  geometry_msgs::TransformStamped tf;
  geometry_msgs::TransformStamped brtf; 
  cmd_velPub = node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber danger_sub = node_handle.subscribe("danger_level",2,dangerCallback);
  geometry_msgs::Twist cmd;
  cmd.linear.x  = cmd.angular.z = 0;
  
  int count = 0;
  while (ros::ok())
  {

    
    try{
      tf = tfBuffer.lookupTransform("bus", "target",ros::Time(0));
      brtf = tfBuffer.lookupTransform("world", "bus",ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      initBus(br);
      ros::Duration(1.0).sleep();
      continue;
    }
    

    //target direction
    double d = -1* atan2(tf.transform.translation.x,
		      tf.transform.translation.y);    
    brtf.header.stamp = ros::Time::now();
    brtf.header.frame_id = "world";
    brtf.child_frame_id ="bus";
    
    tf2::Transform  tr;
    tf2::fromMsg(brtf.transform,tr);
    
    cmd.linear.x  = (isSafe)?1:0;
    cmd.angular.z = oldDir=d;
    cmd_velPub.publish(cmd);
    
    ros::spinOnce();
    loop_rate.sleep();
    ++count;    

  }


  return 0;
}

