#pragma once

#include "waypoint_following/followingStrategyBaseClass.h"

/**
 * @brief  A Base class to all the local Strategies
 *
 * @author    Jose Coelho
 * @version   1.0a
 * @date      2022
 * @copyright GPLv3
 */

class straightLinesStrategy: public followingStrategyBaseClass{
    double a = 0.5,t1 = 0,t2 = 0, t = 0, t0 = 0;
    double t_total = 0;
    double p_now[3]= {0}, p_avg[3]= {0}; 
    double a1[3]= {0}; //Acelleration on the acellerated movement phase
    double a2[3]= {0}; //Acelleration on the slowdown movement phase
    double v01[3]= {0}, v02[3]= {0};
    double p[3][1]= {0};
    // ros::Publisher vis_pub;

    public:
        straightLinesStrategy(DroneLib::UAV * _uav, std::vector<double> _kp,  std::vector<double> _ki,  std::vector<double> _kd);
        bool fly(std::vector<Waypoint>& nodes_to_check, double flightTime);
        // void rvizPublish(std::vector<Waypoint> nodes_to_check);

    private:
        bool init = false;

        void initParameters(Waypoint node_to_check);
        std::vector<Waypoint> wpVectorHanlder(std::vector<Waypoint>& nodes_to_check);

};