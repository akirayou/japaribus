/*
get "bus"/TF position
set "target"/TF position -- for real bus

internal use:
read "tlog.xyz" as planed trajectory
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

static std::vector< geometry_msgs::Vector3> tlog;
static int tlogIdx=0;
void loadTlog(void){
  std::string path = ros::package::getPath("st_pub");
  path+="/data/tlog.xyz"; 
  std::ifstream f;
  f.open(path.c_str(), std::ios::in);
  while (!f.eof()){
    geometry_msgs::Vector3 p;    
    f >> p.x >>p.y >>p.z;
    if(!f.good())break;
    p.z*=0;
    tlog.push_back(p);
  }
  tlogIdx=tlog.size()/2;
}
double nowX,nowY;
void nextTlog( tf2_ros::TransformBroadcaster  & br,int step=1,bool doPublish=true){
  if(tlog.empty())return;

  tlogIdx+=step;
  if(tlog.size()<=tlogIdx)tlogIdx=0;
  
  geometry_msgs::TransformStamped brtf; 
  brtf.header.stamp = ros::Time::now();
  brtf.header.frame_id = "world";
  brtf.child_frame_id ="target";
  nowX=brtf.transform.translation.x = tlog[tlogIdx].x;
  nowY=brtf.transform.translation.y = tlog[tlogIdx].y;
  brtf.transform.translation.z = 0.0;
  brtf.transform.rotation.x = 0;
  brtf.transform.rotation.y = 0;
  brtf.transform.rotation.z = 0;
  brtf.transform.rotation.w = 1;
  if(doPublish)br.sendTransform(brtf);

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "sim");
  ros::NodeHandle node_handle;
  loadTlog();
  ros::Rate loop_rate(30);
  static tf2_ros::TransformBroadcaster br;
  nextTlog(br,0);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfl(tfBuffer);
  geometry_msgs::TransformStamped tf;
  
  int count = 0;
  while (ros::ok())
  {    
    try{
      tf = tfBuffer.lookupTransform("world", "bus",ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("IN plan.cc : %s",ex.what());
      nextTlog(br,0);
      ros::Duration(1.0).sleep();
      continue;
    }
    
    //skip next pos
    if(hypot(tf.transform.translation.x-nowX,tf.transform.translation.y-nowY)<0.8){
      nextTlog(br,1,false);
      ros::spinOnce();
      continue;
    }else{
      nextTlog(br,0);
    }

    ros::spinOnce();
    loop_rate.sleep();
    ++count;    

  }


  return 0;
}

