#pragma once

#include <ros/ros.h>
#include <mavros_cpp/UAV.h>
#include <vector>
#include <numeric>
#include <cmath>
#include <cstring>

#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "waypoint_following/DebugMSG.h"
#include "waypoint_following/StopWpGenerationMSG.h"

/**
 * @brief  A class to keep the flight modes
 *
 * @author    Jose Coelho
 * @version   1.0a
 * @date      2022
 * @copyright GPLv3
 */

struct Waypoint{
    float x;
    float y;
    float z;

    float inspectionTime = 0;

    std::string frame = "ned";
};

#define CONSTANT_DRONE_ALTITUDE 1

class followingStrategyBaseClass{

    public:
        ros::NodeHandle* nh;
        DroneLib::UAV* uav;
        double hoverPosition[3][1] = {0.0};
        double integral[3][1] = {0};
        std::vector<double> kp; std::vector<double> ki; std::vector<double> kd;
        Waypoint landingPosition;
        bool isHover = false;
        double hoverStartTime = 0.0;
        ros::Publisher debug_pub;
        ros::Publisher stopWpGeneration_pub;
        bool wpGenerationIsStoped = false;

        followingStrategyBaseClass();
        
        virtual bool fly(std::vector<Waypoint>& nodes_to_check, double flightTime);
        virtual void rvizPublish(std::vector<Waypoint> nodes_to_check);
        void droneController(double u[3][1], double att[3][1] , double *T, double mass);
        void enuToNed(double v_enu[3][1], double v_ned[3][1]);
        double hover();
        void updateLandingPosition(Waypoint landingPoint, std::string frame);
        void updateLandingPosition(double landingPoint[3][1], std::string frame);
        void resetIntegral();
        void land();
        void takeOff(double hoverTime, double yaw, double altitude);
        void moveToInitialMissionPosition(double pos[3][1], std::string posFrame, double yaw);

    protected:
        virtual std::vector<Waypoint> wpVectorHanlder(std::vector<Waypoint>& nodes_to_check);
        void updateLastThrust(double thrust);
        void debugTopicPublication(double realPos[3][1], double refPos[3][1], double realVel[3][1], double refVel[3][1], double _att[3][1], double time, double thrust);
        void blockWpGeneration();
        void unblockWpGeneration();
};