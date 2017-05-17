/*
get "target"/TF for get direction 
get "bus"/TF as me
set "bus"/TF as new my position

publish "road clean message" for dummy input(must be switched off )


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

  ros::init(argc, argv, "sim");
  ros::NodeHandle node_handle;
  ros::Rate loop_rate(30);
  static tf2_ros::TransformBroadcaster br;
  initBus(br);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfl(tfBuffer);
  geometry_msgs::TransformStamped tf;
  geometry_msgs::TransformStamped brtf; 

  
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
    
    std::cerr <<  tf.transform.translation<< "d:" << 180/3.141592*d<< std::endl;
    const double dLimit=3.141592*1/180;
    if(dLimit<fabs(d))d=d<0?-dLimit:dLimit;
    
    
    brtf.header.stamp = ros::Time::now();
    brtf.header.frame_id = "world";
    brtf.child_frame_id ="bus";
    
    tf2::Transform  tr;
    tf2::fromMsg(brtf.transform,tr);
    

    
    tr.setRotation( tr.getRotation()*tf2::Quaternion(0,0,sin(d/2),cos(d/2)));
    tr.setOrigin(tr.getOrigin()+ tf2::quatRotate(tr.getRotation(),tf2::Vector3(0,0.02,0)));
    brtf.transform=tf2::toMsg(tr);
     
    br.sendTransform(brtf);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;    

  }


  return 0;
}

