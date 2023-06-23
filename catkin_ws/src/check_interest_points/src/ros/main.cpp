#include <drone_gimmicks_library/DroneGimmicks.h>
#include <ros/ros.h>
#include <drone_utils_cpp/DroneInfo.h>
#include <mavros_cpp/UAV.h>
#include "check_interest_points/StartMission.h"
#include <Eigen/Dense>
#include <vector>
#include "geometry_msgs/Point.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "check_interest_points/DebugMSG.h"
#include <unsupported/Eigen/Polynomials>

bool start = false;
bool first_run = true;
bool first_waypoint = true;
bool mission_reset = false;
struct Point {
    float x;
    float y;
    float z;
};
int count=0;

ros::Publisher vis_pub;

std::vector<Point> nodes_to_check;
int max_num_waypoints;
std::vector<double> world_limits_x;
std::vector<double> world_limits_y;
bool travelling = false;
std::vector<double> initial_pos {0.0 ,0.0 ,0.0};
visualization_msgs::MarkerArray markerArray;

void controller(double u[3][1], double att[3][1], double *T, double mass){

	double g = 9.8066, T_aux;
	double Re3[3][1], att_aux[3][1];

	T_aux = sqrt(pow(-u[0][0], 2) + pow(-u[1][0], 2) + pow(mass*g-u[2][0], 2));

	Re3[0][0]=-u[0][0]/T_aux; Re3[1][0]=-u[1][0]/T_aux; Re3[2][0]=(mass*g-u[2][0])/T_aux;

	att_aux[0][0]=asin(-Re3[1][0]); att_aux[1][0]=atan(Re3[0][0]/Re3[2][0]) ; att_aux[2][0]=0;

	*T = T_aux;
	memcpy(att, att_aux, sizeof(att_aux));

	return;
}

bool StartMissionService(check_interest_points::StartMission::Request  &req,
          check_interest_points::StartMission::Response &res){

    count = 0;
    markerArray.markers.clear();
    if (start){
        nodes_to_check.clear();
        
        travelling = false;
        first_waypoint = false;
        
    }
    Point aux;
    // Save the mission
    if (req.points.size() > max_num_waypoints || req.points.size() == 0){
        res.success = false;
        res.msg = "Please send between 1 and 10 waypoints!";
        return true;
    }
    for (size_t i = 0; i < req.points.size(); i++)
    {
        aux.x = req.points[i].x;
        aux.y = req.points[i].y;
        aux.z = req.points[i].z;
        if (aux.x >= world_limits_x[0] && aux.x <= world_limits_x[1] && aux.y >= world_limits_y[0] && aux.y <= world_limits_y[1] && aux.z >= 0){
            nodes_to_check.push_back(aux);

            visualization_msgs::Marker marker;
            marker.header.frame_id = "base_link";
            marker.header.stamp = ros::Time();
            marker.ns = "waypoint";
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = aux.x;
            marker.pose.position.y = aux.y;
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
            vis_pub.publish( markerArray );
        }
    }
    
    if (nodes_to_check.size() == 0){
        res.success = false;
        res.msg = "The waypoints received do not comply with the world limits";
        return true;
    } else {
        aux = nodes_to_check.back();
        if ((aux.x != initial_pos[0]) && (aux.y != initial_pos[1])){
            aux.x = initial_pos[0];
            aux.y = initial_pos[1];
            nodes_to_check.push_back(aux);
        }
        start = true;
        res.success = true;
        return true;
    }
}

ros::NodeHandle * nh;
ros::NodeHandle * nh_p;

DroneLib::UAV * uav = nullptr;
DroneLib::DroneInfo drone_info;

int main (int argc, char ** argv){
    /* Initiate the node */
    ros::init(argc, argv, "check_interest_points_node");
    nh = new ros::NodeHandle();
    nh_p = new ros::NodeHandle("~");

    ros::Rate r(30.0);
    double pos[3][1];
    double yaw;
    Point aux;
    std::vector<double> kp, kd, ki;
    check_interest_points::DebugMSG debug_msg;

    /* Get the namespace of the drone and other parameters */
    drone_info.drone_ns = DroneGimmicks::getParameters<std::string>(*nh, "namespace");
    drone_info.mass = DroneGimmicks::getParameters<double>(*nh, "drone_params/mass");
    drone_info.radius = DroneGimmicks::getParameters<double>(*nh, "drone_params/radius");
    drone_info.height = DroneGimmicks::getParameters<double>(*nh, "drone_params/height");
    drone_info.num_rotors = DroneGimmicks::getParameters<int>(*nh, "drone_params/num_motors");
    drone_info.g = DroneGimmicks::getParameters<double>(*nh, "drone_params/g");
    drone_info.thrust_curve = DroneGimmicks::getParameters<std::string>(*nh, "drone_params/thrust_curve"); 

    ros::Publisher debug_pub = nh->advertise<check_interest_points::DebugMSG>("debug", 1);
    vis_pub = nh->advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );

    nh_p->getParam("drone/initial_pos", initial_pos);
    nh_p->getParam("control_parameters/max_num_waypoints", max_num_waypoints);
    nh_p->getParam("control_parameters/world_limits_x", world_limits_x);
    nh_p->getParam("control_parameters/world_limits_y", world_limits_y);

    nh_p->getParam("pid/kp", kp);
    nh_p->getParam("pid/kd", kd);
    nh_p->getParam("pid/ki", ki);

    std::string start_mission_topic = "topics/services/start_mission";
    ros::ServiceServer mission_start_srv = nh->advertiseService(start_mission_topic, StartMissionService);

    /* Create the drone object */
    uav = new DroneLib::UAV(drone_info.drone_ns, 
        drone_info.mass, 
        drone_info.radius, 
        drone_info.height, 
        drone_info.num_rotors, 
        drone_info.thrust_curve,
        nh, nh_p); 

    int i;
	double t0, t, T, t00;
	double integral[3][1];
	double p[3][1], v[3][1], p_ref[3][1], v_ref[3][1], a_ref[3][1], att[3][1], att_q[4][1], u[3][1];
    double dist, t_total, d_speed, d;
    p_ref[0][0] = 0;
    p_ref[1][0] = 0;
    
    double p_aux[3], p_avg[3], a1[3], a2[3], v01[3], v02[3];
    double dx;
    double dy;

    double a = 0.5,t1,t2,t11;
    double v_max = 2;

    
    ROS_INFO("Waiting for instructions");
    t00 = ros::Time::now().toSec();

    /* Start the infinite loop */
    while(ros::ok()) {
        //ROS_INFO("looping");
        if (start) {
            
            
            if (first_run){
                ROS_INFO("Starting Mission!");

                pos[0][0]=initial_pos[0]; pos[1][0]=initial_pos[1]; pos[2][0]=initial_pos[2];
	            yaw = 0.0;
                uav->start_offboard_mission();
                uav->set_pos_yaw(pos, yaw, 8.0);

                first_run = false;

                integral[0][0]=0.0; integral[1][0]=0.0; integral[2][0]=0.0;

                t0 = ros::Time::now().toSec();
                t = ros::Time::now().toSec()-t0;

            }

            if(nodes_to_check.size() != 0){

                if(!travelling){
                    aux.x = nodes_to_check[0].x;
                    aux.y = nodes_to_check[0].y;
                    aux.z = nodes_to_check[0].z;
                    ROS_WARN("waypoint %f %f",aux.x,aux.y);

                    p_aux[0] = p_ref[0][0];
                    p_aux[1] = p_ref[1][0];
                    p_avg[0] = (aux.x+p_aux[0]) * 0.5;
                    p_avg[1] = (aux.y+p_aux[1]) * 0.5;

                    a1[0] = a;
                    a2[0] = -a;

                    t1 = sqrt(abs(p_avg[0]-p_aux[0])/(0.5 * abs(a1[0])));
                    a1[1] = (p_avg[1]-p_aux[1])/(0.5 * t1*t1);
                    a1[0] = ((p_avg[0] > p_aux[0]) - (p_avg[0] < p_aux[0]))*a1[0];
                    v01[0] = 0;
                    v01[1] = 0;

                    t2 = sqrt(abs(aux.x-p_avg[0])/(0.5 * abs(a2[0])));
                    a2[1] = -(aux.y-p_avg[1])/(0.5 * t2*t2);
                    a2[0] = ((aux.x > p_avg[0]) - (aux.x < p_avg[0]))*a2[0];
                    v02[0] = - a2[0] * t2;
                    v02[1] = - a2[1] * t2;

                    /*if(abs(v02[0])>v_max || abs(v02[1])>v_max){
                        a1[0] = a;
                        a1[1] = a;
                        t1 = v_max/a1[0];
                        a1[1] = v_max/t1;
                        v01[0] = 0;
                        v01[1] = 0;   

                        dist = (t1*v_max)/2;
                        d = sqrt(pow(aux.x-p_aux[0],2)+pow(aux.y-p_aux[1],2));
                        d = d - 2*dist;
                        t2 = d/v_max;

                        a2[0] = -a;
                        a2[1] = -v_ref[1][0]/t2;
                        v02[0] = v_ref[0][0];
                        v02[1] = v_ref[1][0];   

                        t_total = 2*t1 + t2;
                    }*/ 


                    if((abs(v_ref[0][0])>0.3) || (abs(v_ref[1][0])>0.3)){
                        ROS_WARN("going to stop");
                        a2[0] = -1,5*a;
                        a2[1] = -1,5*a;
                        if (abs(v_ref[0][0]) > abs(v_ref[1][0])){
                            t2 = -v_ref[0][0]/a2[0];
                            a2[1] = -v_ref[1][0]/t2;
                        } else {
                            t2 = -v_ref[1][0]/a2[1];
                            a2[0] = -v_ref[0][0]/t2;
                        }
                        v02[0] = v_ref[0][0];
                        v02[1] = v_ref[1][0];   
                        t1 = 0;                     
                        
                        mission_reset = true;
                    }
                    t_total = t1 + t2;

                    t0 = ros::Time::now().toSec();
                    t = 0;

                    travelling = true;
                }
                memcpy(p, uav->ekf.pos, sizeof(uav->ekf.pos));
                if (t<t_total){
                    
                    memcpy(v, uav->ekf.vel, sizeof(uav->ekf.vel));
                    if(mission_reset){

                        p_ref[0][0]= p_aux[0] + v02[0]*(t-t1) + 0.5*a2[0]*(t-t1)*(t-t1); p_ref[1][0]=p_aux[1] + v02[1]*(t-t1) + 0.5*a2[1]*(t-t1)*(t-t1); p_ref[2][0]=-1;
                        v_ref[0][0]= v02[0] + a2[0]*(t-t1); v_ref[1][0]=v02[1] + a2[1]*(t-t1); v_ref[2][0]=0;
                        a_ref[0][0]= a2[0]; a_ref[1][0]= a2[1]; a_ref[2][0]=0;
                    } else {
                        
                        if(t<=t1){
                            p_ref[0][0]= p_aux[0] + v01[0]*t + 0.5*a1[0]*t*t; p_ref[1][0]=p_aux[1] + v01[1]*t + 0.5*a1[1]*t*t; p_ref[2][0]=-1;
                            v_ref[0][0]= v01[0] + a1[0]*t; v_ref[1][0]=v01[1] + a1[1]*t; v_ref[2][0]=0;
                            a_ref[0][0]=a1[0]; a_ref[1][0]=a1[1]; a_ref[2][0]=0;
                        } else{
                            p_ref[0][0]= p_avg[0] + v02[0]*(t-t1) + 0.5*a2[0]*(t-t1)*(t-t1); p_ref[1][0]=p_avg[1] + v02[1]*(t-t1) + 0.5*a2[1]*(t-t1)*(t-t1); p_ref[2][0]=-1;
                            v_ref[0][0]= v02[0] + a2[0]*(t-t1); v_ref[1][0]=v02[1] + a2[1]*(t-t1); v_ref[2][0]=0;
                            a_ref[0][0]=a2[0]; a_ref[1][0]=a2[1]; a_ref[2][0]=0;
                        }
                    }

                    debug_msg.pos[0] = p[0][0];debug_msg.pos[1] = p[1][0];debug_msg.pos[2] = p[2][0];
                    debug_msg.pos_ref[0] = p_ref[0][0];debug_msg.pos_ref[1] = p_ref[1][0];debug_msg.pos_ref[2] = p_ref[2][0];
                    debug_msg.v[0] = v[0][0];debug_msg.v[1] = v[1][0];debug_msg.v[2] = v[2][0];
                    debug_msg.v_ref[0] = v_ref[0][0];debug_msg.v_ref[1] = v_ref[1][0];debug_msg.v_ref[2] = v_ref[2][0];
                    debug_msg.t = ros::Time::now().toSec()-t00;
                    debug_pub.publish(debug_msg);

                    for (i=0; i<=2; i++)
                        u[i][0] = -kp[i]*(p[i][0]-p_ref[i][0]) -kd[i]*(v[i][0]-v_ref[i][0]) -ki[i]*integral[i][0] + uav->info.mass*a_ref[i][0];

                    controller(u, att, &T, uav->info.mass);

                    for (i=0; i<=2; i++)
                        integral[i][0] = integral[i][0] + (p[i][0]-p_ref[i][0])*1.0/50.0; 

                    uav->set_att_thrust(att, att_q, "euler", T, 50.0);
                    t = ros::Time::now().toSec()-t0;
                } else {
                    // Stay in position
                    if((t < t_total + aux.z)&& !mission_reset){
                        memcpy(p, uav->ekf.pos, sizeof(uav->ekf.pos));
                        memcpy(v, uav->ekf.vel, sizeof(uav->ekf.vel));
                        p_ref[0][0]= aux.x; p_ref[1][0]=aux.y; p_ref[2][0]=-1;
                        v_ref[0][0]= 0; v_ref[1][0]=0; v_ref[2][0]=0;
                        a_ref[0][0]=0; a_ref[1][0]=0; a_ref[2][0]=0;

                        debug_msg.pos[0] = p[0][0];debug_msg.pos[1] = p[1][0];debug_msg.pos[2] = p[2][0];
                        debug_msg.pos_ref[0] = p_ref[0][0];debug_msg.pos_ref[1] = p_ref[1][0];debug_msg.pos_ref[2] = p_ref[2][0];
                        debug_msg.v[0] = v[0][0];debug_msg.v[1] = v[1][0];debug_msg.v[2] = v[2][0];
                        debug_msg.v_ref[0] = v_ref[0][0];debug_msg.v_ref[1] = v_ref[1][0];debug_msg.v_ref[2] = v_ref[2][0];
                        debug_msg.t = ros::Time::now().toSec()-t00;
                        debug_pub.publish(debug_msg);

                        for (i=0; i<=2; i++)
                            u[i][0] = -kp[i]*(p[i][0]-p_ref[i][0]) -kd[i]*(v[i][0]-v_ref[i][0]) -ki[i]*integral[i][0] + uav->info.mass*a_ref[i][0];
                        controller(u, att, &T, uav->info.mass);
                        for (i=0; i<=2; i++)
                            integral[i][0] = integral[i][0] + (p[i][0]-p_ref[i][0])*1.0/50.0; 
                        uav->set_att_thrust(att, att_q, "euler", T, 50.0);
                        t = ros::Time::now().toSec()-t0;
                    } else {
                        if (!mission_reset)
                        {
                            nodes_to_check.erase(nodes_to_check.begin());
                            markerArray.markers[count].action = visualization_msgs::Marker::DELETE;
                            count++;
                            vis_pub.publish( markerArray );
                            //first_waypoint = true;
                        }
                        
                        travelling = false;
                        mission_reset = false;
                        
                        ROS_WARN("arrived to waypoint");
                    }
                }
            
            } else {
                start = false;
                first_run = true;
                ROS_INFO("Mission Finished!");
            }
            //uav->auto_land();
        }
        /* Check for callbacks */
        ros::spinOnce(); 
    } 
    /* Free the memory for the UAV object */
    delete uav;
    delete nh;
    delete nh_p;

    return 0;
}