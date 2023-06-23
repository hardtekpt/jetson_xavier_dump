#include <drone_gimmicks_library/DroneGimmicks.h>
#include <ros/ros.h>
#include <drone_utils_cpp/DroneInfo.h>
#include <mavros_cpp/UAV.h>

#include "waypoint_following/WaypointRegistration.h"
#include "waypoint_following/DebugMSG.h"
#include "waypoint_following/StartStopMission.h"
#include "waypoint_following/followingStrategyBaseClass.h"
#include "waypoint_following/inspectStrategy.h"
#include "waypoint_following/straightLinesStrategy.h"
#include "waypoint_following/bezierCurvesStrategy.h"
#include "waypoint_following/SpiralParameters.h"

bool start = false; //Mission running - Enough information to perform a mission
bool first_run = true; //First drone iteration when a mission starts
bool flying = false; //Drone is flying

bool mission_reset = false;

followingStrategyBaseClass * followingStrategy = nullptr;

std::vector<Waypoint> wp_to_check; //Save waypoints in NED frame 
std::vector<double> world_limits_x;
std::vector<double> world_limits_y;
std::vector<double> initialPosition;

void enuToNed(double v_enu[3][1], double v_ned[3][1]){
    double v_aux[3][1] = {{v_enu[1][0]},{v_enu[0][0]},{-v_enu[2][0]}};
	memcpy(v_ned, v_aux, sizeof(v_aux));
}

bool WaypointRegistrationService(waypoint_following::WaypointRegistration::Request  &req,
          waypoint_following::WaypointRegistration::Response &res){
    //count = 0; //??
    ROS_INFO("Dentro do WaypointRegistrationService with %li waypoints to register", req.waypoints.size());
    Waypoint aux;

    if (start){
        wp_to_check.pop_back();
        
        //first_run = true;
        mission_reset = true;
    }

    // Save the mission
    for (size_t i = 0; i < req.waypoints.size(); i++)
    {
        // ROS_INFO("Processing waypoint %li", i);
        aux.x = req.waypoints[i].x;
        aux.y = req.waypoints[i].y;
        aux.z = req.waypoints[i].z;
        aux.inspectionTime = req.waypoints[i].inspectTime;

        if(req.waypoints[i].frame.compare("enu") == 0){
            // ENU to NED conversion
            // ROS_INFO("Waypoint in NED frame. Changing to ENU frame.");
            double enuPoint[3][1];
            enuPoint[0][0] = aux.x; enuPoint[1][0] = aux.y; enuPoint[2][0] = aux.z;
            double nedPoint[3][1];
            enuToNed(enuPoint, nedPoint);
            aux.x = nedPoint[0][0];
            aux.y = nedPoint[1][0];
            aux.z = nedPoint[2][0];
            aux.frame = "ned";
        }
        else{
            if(req.waypoints[i].frame.compare("ned") != 0){
                ROS_WARN("Waypoint #%li = [x=%f y=%f z=%f] was ignored, becausa its frame was either 'enu' not 'ned'", i, aux.x, aux.y, aux.z);
                continue;
            }
        }

        // !! world limits are in enu and aux already in ned !!
        if (aux.y >= world_limits_x[0] && aux.y <= world_limits_x[1] && aux.x >= world_limits_y[0] && aux.x <= world_limits_y[1] && aux.z < 0){
            wp_to_check.push_back(aux);
            ROS_INFO("Waypoint OK");
        }
        else
            ROS_WARN("Waypoint #%li = [x=%f y=%f z=%f] NED was ignored, becausa it is out of boundouries y[%f %f] x[%f %f]", i, aux.x, aux.y, aux.z, world_limits_x[0], world_limits_x[1], world_limits_y[0], world_limits_y[1]);

    }

    if (wp_to_check.size() >= 1){// if (wp_to_check.size() != 0){
        aux.x=0;aux.y=0;aux.z=-1;
        wp_to_check.push_back(aux);
        start = true;
        flying = true;
        ROS_INFO("New mission");
        res.success = true;
        return true;
    }
    if(wp_to_check.size() != 0){
        res.success = true;
        return true;
    }
    else{
        start = false;
        first_run = true;
        flying = false;
        res.msg = "No valid waypoints to register";
        return false;  
    }
}

bool StartStopMissionService(waypoint_following::StartStopMission::Request  &req,
          waypoint_following::StartStopMission::Response &res){

    if(req.startStopRequest.compare("start") == 0){
        ROS_WARN("Start Mission Command Received");
        flying = true;
        start = true;
        first_run = true;
        res.success = true;
        return true;
    }
    else if(req.startStopRequest.compare("stop") == 0){
        ROS_WARN("Stop Mission Command Received");
        flying = false;
        start = false;
        first_run = false;
        res.success = true;
        return true;
    }
    else{
        ROS_WARN("Unkown Start/Stop Mission Command Received. Received cmd = %s", req.startStopRequest.c_str());
        res.success = false;
        return false;
    }
}

int main (int argc, char ** argv){
    /* Initiate the node */
    ros::init(argc, argv, "waypoint_following_node");
    ros::NodeHandle * nh;
    ros::NodeHandle * nh_p;
    nh = new ros::NodeHandle();
    nh_p = new ros::NodeHandle("~");

    ros::Rate r(30.0);
    // double pos[3][1];
    // double yaw;
    // Point aux;
    std::vector<double> kp, kd, ki;
    ROS_INFO("0");
    /* Get the namespace of the drone and other parameters */
    DroneLib::DroneInfo drone_info;
    drone_info.drone_ns = DroneGimmicks::getParameters<std::string>(*nh, "namespace");
    drone_info.mass = DroneGimmicks::getParameters<double>(*nh, "drone_params/mass");
    drone_info.radius = DroneGimmicks::getParameters<double>(*nh, "drone_params/radius");
    drone_info.height = DroneGimmicks::getParameters<double>(*nh, "drone_params/height");
    drone_info.num_rotors = DroneGimmicks::getParameters<int>(*nh, "drone_params/num_motors");
    drone_info.g = DroneGimmicks::getParameters<double>(*nh, "drone_params/g");
    drone_info.thrust_curve = DroneGimmicks::getParameters<std::string>(*nh, "drone_params/thrust_curve"); 

    ROS_INFO("teste");

    nh_p->getParam("control_parameters/world_limits_x", world_limits_x);
    nh_p->getParam("control_parameters/world_limits_y", world_limits_y);
    ROS_INFO("world_limits_x = [%f %f]", world_limits_x[0], world_limits_x[1]);
    ROS_INFO("world_limits_y = [%f %f]", world_limits_y[0], world_limits_y[1]);

    nh_p->getParam("drone/initial_pos", initialPosition); //In enu frame

    nh_p->getParam("pid/kp", kp);
    nh_p->getParam("pid/kd", kd);
    nh_p->getParam("pid/ki", ki);

    int inspectWaypointOn = 1, bezierOn = 0;
    //nh->getParam("inspectWaypointOn", inspectWaypointOn);
    //nh->getParam("bezierOn", bezierOn);
    
    // Services declaration
    std::string waypointRegistrationServiceName = "services/waypoint_registration";
    std::string StartStopMissionServiceName = "services/start_stop_mission";
    ros::ServiceServer wp_registration_srv = nh->advertiseService(waypointRegistrationServiceName, WaypointRegistrationService);
    ros::ServiceServer mission_start_stop_srv = nh->advertiseService(StartStopMissionServiceName, StartStopMissionService);

    // Topics declaration

    /* Create the drone object */
    DroneLib::UAV * uav = nullptr;
    uav = new DroneLib::UAV(drone_info.drone_ns, 
        drone_info.mass, 
        drone_info.radius, 
        drone_info.height, 
        drone_info.num_rotors, 
        drone_info.thrust_curve,
        nh, nh_p); 
    
    if(inspectWaypointOn){
        ROS_WARN("Following Strategy: Inspection");
        followingStrategy = new inspectStrategy(uav, kp, ki, kd);
    }
    else{
         if(bezierOn){
            // Bezier
            followingStrategy = new bezierCurvesStrategy(uav, kp, ki, kd);
            ROS_WARN("Following Strategy: Bezier");
        }
        else{
            ROS_WARN("Following Strategy: Straight Line");
            followingStrategy = new straightLinesStrategy(uav, kp, ki, kd);
        }
    }

    double flightTime = 0, t0 = 0;
    double hoverTime = 0; 
    while(ros::ok()) {
        //ROS_INFO("If logs --> start %i / flying %i / first_run %i / size %i", start, flying, first_run, node_to_check.size());
        if (start){ 
            if(flying){    
                //This initiates the drone -> Take off, keep same position and moves to initial mission position
                if (first_run){
                    ROS_INFO("Starting Mission!");
                    double yaw = 0;
                    double pos[3][1];

                    followingStrategy->rvizPublish(wp_to_check);
                    followingStrategy->takeOff(5, yaw, CONSTANT_DRONE_ALTITUDE);

                    //Fly drone to initial mission position
                    //pos[0][0] = initialPosition[0]; pos[1][0] = initialPosition[1]; pos[2][0] = 1;
                    //followingStrategy->moveToInitialMissionPosition(pos, "enu", 0);

                    followingStrategy->resetIntegral();

                    t0 = ros::Time::now().toSec(); 
                    first_run = false;
                }

                if(hoverTime != 0){ROS_WARN("t0 += %f", hoverTime);}
                // t0 += hoverTime;
                hoverTime = 0;
                flightTime = ros::Time::now().toSec()-t0;
                flying = followingStrategy->fly(wp_to_check, flightTime);
            } //end of 'if flying'
            else{
                //hoverTime = followingStrategy->hover();
                //if(hoverTime > 5){
                    //Hover Time out
                //    ROS_WARN("Hover timeout. Drone will land");
                    start = false;
                    followingStrategy->isHover = false;
                //} // end of 'if hoverTime > 5'

            } //end of 'else (if flying)'

        } // end of 'if start'
        else{
            if(!first_run){
                //followingStrategy->land();
                uav->auto_land();
                //ros::shutdown();
                first_run = true;
            }
        }

        /* Check for callbacks */
        ros::spinOnce(); 
        // ROS_INFO("If logs --> start %i / flying %i / first_run %i / size %i", start, flying, first_run, node_to_check.size());
        r.sleep();
    }

    free(uav);
    ROS_WARN("End of node!");
}