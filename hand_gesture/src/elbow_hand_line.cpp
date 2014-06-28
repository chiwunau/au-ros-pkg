#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
using namespace std;

void draw_marker(const tf::StampedTransform &tr);
ros::Publisher visualpub;

int main(int argc, char** argv){
  ros::init(argc, argv, "elbow_hand_line");
  ros::NodeHandle nh;
  tf::TransformListener tfl;
  tf::StampedTransform transform;

  visualpub = nh.advertise<visualization_msgs::Marker>("elbow_hand_line", 0);
  
  ros::Rate rate(10.0);
  while(ros::ok()){
    try{
      tfl.lookupTransform("/right_elbow_1","/right_hand_1",
			  ros::Time(0),transform);
    }
    catch(tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    draw_marker(transform);
    rate.sleep();
  }
  return 0;
}

void draw_marker(const tf::StampedTransform &tr){
  cout<<tr.frame_id_<<endl<<tr.getOrigin().x()<<endl<<tr.getOrigin().y()<<endl<<tr.getOrigin().z()<<endl;


  visualization_msgs::Marker marker;
  marker.header.frame_id = tr.frame_id_;
  marker.header.stamp = tr.stamp_;
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  
  geometry_msgs::Point p;
  p.x = 0; p.y = 0; p.z =0;
  marker.points.push_back(p);
  p.x = 10 * tr.getOrigin().x();
  p.y = 10 * tr.getOrigin().y();
  p.z = 10 * tr.getOrigin().z();
  marker.points.push_back(p);
  marker.scale.x = 0.01;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  visualpub.publish(marker);
}

  
