#include "waypoint_following/inspectStrategy.h"

inspectStrategy::inspectStrategy(DroneLib::UAV * _uav, std::vector<double> _kp,  std::vector<double> _ki,  std::vector<double> _kd){    
    vis_pub = nh->advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 10);
    vis_pub_ref = nh->advertise<visualization_msgs::Marker>( "uav_marker_ref", 0 );

    // Drone and Drone params
	ROS_INFO("Pre drone creation %p",  uav);
    uav = _uav;
	ROS_INFO("Pos drone creation %p",  uav);
    kp = _kp; ki = _ki; kd = _kd;
}

void inspectStrategy::initParameters(Waypoint node_to_check){
    Waypoint aux;
    t = 0;

    aux.x = node_to_check.x;
    aux.y = node_to_check.y;
    aux.z = node_to_check.z;
    ROS_INFO("waypoint %f %f",aux.x,aux.y);
    
    memcpy(p, uav->ekf.pos, sizeof(uav->ekf.pos));

    p_now[0] = p[0][0];
    p_now[1] = p[1][0];
    p_avg[0] = (aux.x+p_now[0]) * 0.5;
    p_avg[1] = (aux.y+p_now[1]) * 0.5;

    if(abs(p_now[0] - p_avg[0]) > abs(p_now[1] - p_avg[1])){
        a1[0] = a; //Acelleration on the acellerated movement phase
        a2[0] = -a; //Acelleration on the slowdown movement phase

        t1 = sqrt(abs(p_avg[0]-p_now[0])/(0.5 * abs(a1[0])));
        a1[1] = (p_avg[1]-p_now[1])/(0.5 * t1*t1);
        a1[0] = ((p_avg[0] > p_now[0]) - (p_avg[0] < p_now[0]))*a1[0];
        v01[0] = 0;
        v01[1] = 0;

        t2 = sqrt(abs(aux.x-p_avg[0])/(0.5 * abs(a2[0])));
        a2[1] = -(aux.y-p_avg[1])/(0.5 * t2*t2);
        a2[0] = ((aux.x > p_avg[0]) - (aux.x < p_avg[0]))*a2[0];
        v02[0] = - a2[0] * t2;
        v02[1] = - a2[1] * t2;

    }else if(p_now[1] != p_avg[1]){
        a1[1] = a; //Acelleration on the acellerated movement phase
        a2[1] = -a; //Acelleration on the slowdown movement phase

        t1 = sqrt(abs(p_avg[1]-p_now[1])/(0.5 * abs(a1[1])));
        a1[0] = (p_avg[0]-p_now[0])/(0.5 * t1*t1);
        a1[1] = ((p_avg[1] > p_now[1]) - (p_avg[1] < p_now[1]))*a1[1];
        v01[0] = 0;
        v01[1] = 0;

        t2 = sqrt(abs(aux.y-p_avg[1])/(0.5 * abs(a2[1])));
        a2[0] = -(aux.x-p_avg[0])/(0.5 * t2*t2);
        a2[1] = ((aux.y > p_avg[1]) - (aux.y < p_avg[1]))*a2[1];
        v02[0] = - a2[0] * t2;
        v02[1] = - a2[1] * t2;
    } else{
        t1 = 0;
        t2 = 0;
    }

    t_total = t1 + t2 + node_to_check.inspectionTime;
}

bool inspectStrategy::fly(std::vector<Waypoint>& nodes_to_check, double flightTime){
    followingStrategyBaseClass::fly(nodes_to_check, flightTime);
    
    //rvizPublish(nodes_to_check);

    if(!init){
        if(nodes_to_check.size() == 0)
            return false;

        initParameters(nodes_to_check[0]);
        t0 = ros::Time::now().toSec();
        t = ros::Time::now().toSec()-t0;
        init = true;
    }

    double T;
    double p_ref[3][1]= {0}, v_ref[3][1]= {0}, a_ref[3][1]= {0};
    double p[3][1]= {0}, v[3][1] = {0};
    double att[3][1]= {0}, att_q[4][1]= {0}, u[3][1]= {0};

    memcpy(p, uav->ekf.pos, sizeof(uav->ekf.pos));
    memcpy(v, uav->ekf.vel, sizeof(uav->ekf.vel));
    
    if(t<=t1){
        //p_now is the initial point for this movement
        p_ref[0][0]= p_now[0] + v01[0]*t + 0.5*a1[0]*t*t; p_ref[1][0]=p_now[1] + v01[1]*t + 0.5*a1[1]*t*t; p_ref[2][0]=nodes_to_check[0].z;
        v_ref[0][0]= v01[0] + a1[0]*t; v_ref[1][0]=v01[1] + a1[1]*t; v_ref[2][0]=0;
        a_ref[0][0]=a1[0]; a_ref[1][0]=a1[1]; a_ref[2][0]=0;
    } else if(t<=t1+t2){
        p_ref[0][0]= p_avg[0] + v02[0]*(t-t1) + 0.5*a2[0]*(t-t1)*(t-t1); p_ref[1][0]=p_avg[1] + v02[1]*(t-t1) + 0.5*a2[1]*(t-t1)*(t-t1); p_ref[2][0]=nodes_to_check[0].z;
        v_ref[0][0]= v02[0] + a2[0]*(t-t1); v_ref[1][0]=v02[1] + a2[1]*(t-t1); v_ref[2][0]=0;
        a_ref[0][0]=a2[0]; a_ref[1][0]=a2[1]; a_ref[2][0]=0;
    } else if(t<=t_total){
        p_ref[0][0]= nodes_to_check[0].x; p_ref[1][0]= nodes_to_check[0].y; p_ref[2][0]=nodes_to_check[0].z;
        v_ref[0][0]= 0; v_ref[1][0]=0; v_ref[2][0]=0;
        a_ref[0][0]= 0; a_ref[1][0]=0; a_ref[2][0]=0;
    } else{
        init = false;
        wpVectorHanlder(nodes_to_check);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time();
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::DELETEALL;
        vis_pub_ref.publish( marker );
        return true;
    }
    
    for (int i=0; i<=2; i++){
        u[i][0] = -kp[i]*(p[i][0]-p_ref[i][0]) -kd[i]*(v[i][0]-v_ref[i][0]) -ki[i]*integral[i][0] + uav->info.mass*a_ref[i][0];
        // ROS_INFO("u[%i] = %f  (-%f*(%f) -%f*(%f) -%f*(%f) + %f*%f)", i, u[i][0], kp[i], (p[i][0]-p_ref[i][0]), kd[i], v[i][0]-v_ref[i][0], ki[i], integral[i][0], uav->info.mass, a_ref[i][0]);
        // ROS_INFO("\n");

    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "uav";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.pose.position.x = p_ref[0][0];
    marker.pose.position.y = p_ref[1][0];
    marker.pose.position.z = -p_ref[2][0];
    //marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.mesh_resource = "package://drone_description/models/iris/meshes/iris.stl";
    marker.mesh_use_embedded_materials = true;
    vis_pub_ref.publish( marker );
    
    droneController(u, att, &T, uav->info.mass);

    for (int i=0; i<=2; i++)
        integral[i][0] = integral[i][0] + (p[i][0]-p_ref[i][0])*1.0/50.0; 
    // ROS_INFO("Ref p[%f, %f, %f] v[%f, %f, %f] a[%f, %f, %f]", p_ref[0][0], p_ref[1][0], p_ref[2][0], v_ref[0][0], v_ref[1][0], v_ref[2][0], a_ref[0][0], a_ref[1][0], a_ref[2][0]);
    uav->set_att_thrust(att, att_q, "euler", T, 50.0);
    // ROS_INFO("att = %f %f %f T = %f", att[0][0], att[1][0], att[2][0], T);
    debugTopicPublication(p, p_ref, v, v_ref, att, flightTime, T);

    t = ros::Time::now().toSec()-t0;

    return true;
}

// Receives a vector of waypoints (currently assumed to be in NED frame) and publish it to be shown in rviz 
// rviz uses ENU frame 
void inspectStrategy::rvizPublish(std::vector<Waypoint> nodes_to_check){
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray markerArray;

    if(nodes_to_check.size()>=1){
    for(int i = 0; i < nodes_to_check.size()-1; i++){
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time();
        marker.ns = "waypoint";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = nodes_to_check[i].x; 
        marker.pose.position.y = nodes_to_check[i].y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 0.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        markerArray.markers.push_back(marker);
    }}
    //ROS_WARN("teste %d %d",nodes_to_check.size()-1, markerArray.markers.size());
    vis_pub.publish( markerArray );
}

std::vector<Waypoint> inspectStrategy::wpVectorHanlder(std::vector<Waypoint>& nodes_to_check){
    // ROS_INFO("size = %li   [%f %f] = [%f %f]?", nodes_to_check.size(), nodes_to_check[nodes_to_check.size()-1].x, nodes_to_check[nodes_to_check.size()-1].y, landingPosition.x, landingPosition.y);
    // if ((nodes_to_check[nodes_to_check.size()-1].x != landingPosition.x) || (nodes_to_check[nodes_to_check.size()-1].y != landingPosition.y)){
    //     Waypoint aux;
    //     aux.x = landingPosition.x;
    //     aux.y = landingPosition.y;
    //     nodes_to_check.push_back(aux);
    //     ROS_WARN("Add the initial position to the list of points to visit as the final position");
    // }

    // ROS_INFO("size = %li   [%f %f]?", nodes_to_check.size(), nodes_to_check[nodes_to_check.size()-1].x, nodes_to_check[nodes_to_check.size()-1].y);

    nodes_to_check.erase(nodes_to_check.begin());

    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray markerArray;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::DELETEALL;
    markerArray.markers.push_back(marker);
    vis_pub.publish( markerArray );

    rvizPublish(nodes_to_check);
    // ROS_INFO("size = %li   [%f %f]?", nodes_to_check.size(), nodes_to_check[nodes_to_check.size()-1].x, nodes_to_check[nodes_to_check.size()-1].y);

    return nodes_to_check;
}