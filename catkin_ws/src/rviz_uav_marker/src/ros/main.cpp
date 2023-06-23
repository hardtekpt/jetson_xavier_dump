#include <ros/ros.h>
#include <vector>
#include <drone_utils_cpp/Utils.h>
#include "geometry_msgs/PoseStamped.h"
#include <iostream>
#include "visualization_msgs/Marker.h"


ros::Publisher vis_uav_pub;

void updateMarkerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "uav";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = msg->pose.position.y;
    marker.pose.position.y = msg->pose.position.x;
    marker.pose.position.z = msg->pose.position.z;
    marker.pose.orientation.x = msg->pose.orientation.x;
    marker.pose.orientation.y = msg->pose.orientation.y;
    marker.pose.orientation.z = msg->pose.orientation.z;
    marker.pose.orientation.w = msg->pose.orientation.w;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 0.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.mesh_resource = "package://drone_description/models/iris/meshes/iris.stl";
    marker.mesh_use_embedded_materials = true;
    vis_uav_pub.publish( marker );
}


int main(int argc, char ** argv){

    std::vector<std::vector<geometry_msgs::Point> > test_list;
    std::vector<geometry_msgs::Point> aux_test;
    geometry_msgs::Point aux;


    ros::init(argc, argv,"rviz_uav_marker_node");
    ros::NodeHandle n;


    ros::Subscriber sub = n.subscribe("iris2/mavros/local_position/pose", 1000, updateMarkerCallback);
    //ros::Subscriber sub = n.subscribe("iris0/mavros/local_position/pose", 1000, updateMarkerCallback);
    vis_uav_pub = n.advertise<visualization_msgs::Marker>( "uav_marker", 0 );

    ros::spin();
    return 0;
}