#include "ros/ros.h"
#include <ros/package.h>
#include "visualization_msgs/Marker.h"
#include <sstream>
#include <fstream>

void setMarkerDefault(visualization_msgs::Marker &marker){    
  marker.id = 0;
  marker.type= visualization_msgs::Marker::MESH_RESOURCE;
  marker.ns = "static_marker";
  marker.header.stamp = ros::Time();
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;
  marker.color.a = 1; // Don't forget to set the alpha!
  marker.color.r = 1;
  marker.color.g = 1;
  marker.color.b = 0.5;
  marker.lifetime = ros::Duration();
  marker.header.frame_id = "mesh_map";
  marker.mesh_use_embedded_materials=true;
  
}

void setMarkerLine(visualization_msgs::Marker &marker){
  setMarkerDefault(marker);
  marker.id=2;
  marker.scale.x = 0.1;

  marker.type=visualization_msgs::Marker::LINE_STRIP;
  {
    std::string path = ros::package::getPath("st_pub");
    path+="/data/tlog.xyz"; 
    std::ifstream f;
    f.open(path.c_str(), std::ios::in);
    while (!f.eof()){
      geometry_msgs::Point p;
      
      f >> p.x >>p.y >>p.z;
      if(!f.good())break;
      p.z*=0;
      marker.points.push_back(p);
    }    
  }
}
void setMarkerMap(visualization_msgs::Marker &marker){
  setMarkerDefault(marker);
  marker.id=0;
  marker.color.a = 0.5;
  marker.mesh_resource = "package://st_pub/data/mesh_gen.dae";
}
void setMarkerBus(visualization_msgs::Marker &marker){
  setMarkerDefault(marker);
  marker.id=1;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.header.frame_id = "bus";
  marker.mesh_resource = "package://st_pub/data/bus.dae";
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 1.0;
  marker.pose.orientation.w = 0.0;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "st_pub_node");
  ros::NodeHandle node_handle;
  
  ros::Publisher marker_tlog_pub = node_handle.advertise<visualization_msgs::Marker>( "marker_tlog", 0 );
  ros::Publisher marker_map_pub = node_handle.advertise<visualization_msgs::Marker>( "marker_map", 0 );
  ros::Publisher marker_bus_pub = node_handle.advertise<visualization_msgs::Marker>( "marker_bus", 0 );
  ros::Rate loop_rate(10);
  
  visualization_msgs::Marker marker_line,marker_map,marker_bus;
  setMarkerLine(marker_line);
  setMarkerMap(marker_map);
  setMarkerBus(marker_bus);
  

  
  int count = 0;
  while (ros::ok())
  {
    visualization_msgs::Marker marker;


    marker_bus_pub.publish(marker_bus);
    marker_map_pub.publish(marker_map);
    marker_tlog_pub.publish(marker_line);
    
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}

