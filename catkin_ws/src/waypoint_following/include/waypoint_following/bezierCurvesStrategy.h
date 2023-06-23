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

class bezierCurvesStrategy: public followingStrategyBaseClass{

    std::vector<std::vector<double>> M;
    int floorTime = 0, lastFloorTime = 0;
    // double initialComputationTime = -1;
    double gamma = 0.01, gammaDot = 0;
    // double bezierFlightTime = 0;
    double VDesejado = 1.5;
    double lastSpeedProfile = 0;
    double K = 0.2; //Speed Profile Constant
    double offset = 0.1;
    double lastVelRefNorm = 0;
    double lastAccRefNorm = 0; 

    public:
        bezierCurvesStrategy(DroneLib::UAV * _uav, std::vector<double> _kp,  std::vector<double> _ki,  std::vector<double> _kd);
        bool fly(std::vector<Waypoint>& nodes_to_check, double flightTime);

    private:
        bool init = false;

        void initParameters(Waypoint node_to_check);
        std::vector<Waypoint> wpVectorHanlder(std::vector<Waypoint>& nodes_to_check);
        double getCurrentSpeedProfile(std::vector<double> curveVelocity, std::vector<double> curveAcceleration);
        double getCurrentSpeedProfileGradient(std::vector<double> curveVel, std::vector<double> curveAcc, std::vector<double> curve3rdGrad);
        void updateGamma(double delta_t, double gammaMax);
        std::vector<double> getCurvePosition(std::vector<double> nodes_to_check_x, std::vector<double> nodes_to_check_y, double gammaNormalized);
        std::vector<double> getCurveVelocity(std::vector<double> nodes_to_check_x, std::vector<double> nodes_to_check_y, double gammaNormalized);
        std::vector<double> getCurveAcceleration(std::vector<double> nodes_to_check_x, std::vector<double> nodes_to_check_y, double gammaNormalized);
        std::vector<double> getCurve3rdGrad(std::vector<double> nodes_to_check_x, std::vector<double> nodes_to_check_y);
        void velAccSaturation(double (*velocity)[3][1], double (*acceleration)[3][1]);
};
