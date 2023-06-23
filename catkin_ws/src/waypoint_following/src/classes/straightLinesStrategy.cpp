#include "waypoint_following/straightLinesStrategy.h"

straightLinesStrategy::straightLinesStrategy(DroneLib::UAV * _uav, std::vector<double> _kp,  std::vector<double> _ki,  std::vector<double> _kd){
    // vis_pub = nh->advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 10);

    // Drone and Drone params
	ROS_INFO("Pre drone creation %p",  uav);
    uav = _uav;
	ROS_INFO("Pos drone creation %p",  uav);
    kp = _kp; ki = _ki; kd = _kd;
}

void straightLinesStrategy::initParameters(Waypoint node_to_check){
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

    t_total = t1 + t2;
}

bool straightLinesStrategy::fly(std::vector<Waypoint>& nodes_to_check, double flightTime){
    followingStrategyBaseClass::fly(nodes_to_check, flightTime);
    
    if(nodes_to_check.size() == 0)
            return false;

    wpVectorHanlder(nodes_to_check);

    if(!init){
        wpVectorHanlder(nodes_to_check);

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
    } else if(t<=t_total){
        p_ref[0][0]= p_avg[0] + v02[0]*(t-t1) + 0.5*a2[0]*(t-t1)*(t-t1); p_ref[1][0]=p_avg[1] + v02[1]*(t-t1) + 0.5*a2[1]*(t-t1)*(t-t1); p_ref[2][0]=nodes_to_check[0].z;
        v_ref[0][0]= v02[0] + a2[0]*(t-t1); v_ref[1][0]=v02[1] + a2[1]*(t-t1); v_ref[2][0]=0;
        a_ref[0][0]=a2[0]; a_ref[1][0]=a2[1]; a_ref[2][0]=0;
    } else{
        init = false;
        nodes_to_check.erase(nodes_to_check.begin());
        return true;
    }
    
    for (int i=0; i<=2; i++){
        u[i][0] = -kp[i]*(p[i][0]-p_ref[i][0]) -kd[i]*(v[i][0]-v_ref[i][0]) -ki[i]*integral[i][0] + uav->info.mass*a_ref[i][0];
        // ROS_INFO("u[%i] = %f  (-%f*(%f) -%f*(%f) -%f*(%f) + %f*%f)", i, u[i][0], kp[i], (p[i][0]-p_ref[i][0]), kd[i], v[i][0]-v_ref[i][0], ki[i], integral[i][0], uav->info.mass, a_ref[i][0]);
        // ROS_INFO("\n");
    }
    
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

std::vector<Waypoint> straightLinesStrategy::wpVectorHanlder(std::vector<Waypoint>& nodes_to_check){
    if(nodes_to_check.size() > 1){
        for(size_t i = 0; i < nodes_to_check.size()-1; i++){
            nodes_to_check.erase(nodes_to_check.begin() + i);
        }

        init = false;
    }
    return nodes_to_check;
}