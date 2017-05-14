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
void nextTlog( tf2_ros::TransformBroadcaster  & br,int step=1){
  if(tlog.empty())return;

  tlogIdx+=step;
  if(tlog.size()<=tlogIdx)tlogIdx=0;
  
  geometry_msgs::TransformStamped brtf; 
  brtf.header.stamp = ros::Time::now();
  brtf.header.frame_id = "world";
  brtf.child_frame_id ="target";
  brtf.transform.translation.x = tlog[tlogIdx].x;
  brtf.transform.translation.y = tlog[tlogIdx].y;
  brtf.transform.translation.z = 0.0;
  brtf.transform.rotation.x = 0;
  brtf.transform.rotation.y = 0;
  brtf.transform.rotation.z = 0;
  brtf.transform.rotation.w = 1;
  br.sendTransform(brtf);

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

  ros::init(argc, argv, "sim");
  ros::NodeHandle node_handle;
  loadTlog();
  ros::Rate loop_rate(30);
  static tf2_ros::TransformBroadcaster br;
  initBus(br);
  nextTlog(br,0);

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
      nextTlog(br,0);
      ros::Duration(1.0).sleep();
      continue;
    }
    
    //skip next pos
    if(hypot(tf.transform.translation.x,tf.transform.translation.y)<0.5){
      nextTlog(br,1);
      brtf.header.stamp = ros::Time::now();
      br.sendTransform(brtf);//need dummy bus pos for listen  bus to target
      ros::spinOnce();
      continue;
    }else{
      nextTlog(br,0);
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

