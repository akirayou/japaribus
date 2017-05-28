/*


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
#include <sensor_msgs/Joy.h>

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

double acc=0;
double direc=0;
void joyCallback(const sensor_msgs::Joy& msg)
{
  acc=msg.axes[1];
  direc=msg.axes[0];
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
  geometry_msgs::TransformStamped brtf;
  
  ros::Subscriber sub = node_handle.subscribe("joy", 1000, joyCallback);
  
  int count = 0;
  while (ros::ok())
  {    
    try{
      brtf = tfBuffer.lookupTransform("world", "bus",ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      initBus(br);
      ros::Duration(1.0).sleep();
      continue;
    }
    

    //target direction
    double d =   acc*direc*2/180.0*3.141592 ;

    
    
    brtf.header.stamp = ros::Time::now();
    brtf.header.frame_id = "world";
    brtf.child_frame_id ="bus";
    
    tf2::Transform  tr;
    tf2::fromMsg(brtf.transform,tr);
    

    
    tr.setRotation( tr.getRotation()*tf2::Quaternion(0,0,sin(d/2),cos(d/2)));
    tr.setOrigin(tr.getOrigin()+acc* tf2::quatRotate(tr.getRotation(),tf2::Vector3(0,0.02,0)));
    brtf.transform=tf2::toMsg(tr);
     
    br.sendTransform(brtf);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;    

  }


  return 0;
}

