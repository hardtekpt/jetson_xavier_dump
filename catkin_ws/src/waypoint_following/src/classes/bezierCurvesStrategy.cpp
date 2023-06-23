#include "waypoint_following/bezierCurvesStrategy.h"

bezierCurvesStrategy::bezierCurvesStrategy(DroneLib::UAV * _uav, std::vector<double> _kp,  std::vector<double> _ki,  std::vector<double> _kd){  
    floorTime = 0;

    // Drone and Drone params
    uav = _uav;
    kp = _kp; ki = _ki; kd = _kd;

    //M matrix for the bezier curves
    std::vector<double> MLine;
    MLine.push_back(-1/6.0); MLine.push_back(3/6.0); MLine.push_back(-3/6.0); MLine.push_back(1/6.0);
    M.push_back(MLine); MLine.clear();
    MLine.push_back(3/6.0); MLine.push_back(-6/6.0); MLine.push_back(0/6.0); MLine.push_back(4/6.0);
    M.push_back(MLine); MLine.clear();
    MLine.push_back(-3/6.0); MLine.push_back(3/6.0); MLine.push_back(3/6.0); MLine.push_back(1/6.0);
    M.push_back(MLine); MLine.clear();
    MLine.push_back(1/6.0); MLine.push_back(0/6.0); MLine.push_back(0/6.0); MLine.push_back(0/6.0);
    M.push_back(MLine); MLine.clear();

}

bool bezierCurvesStrategy::fly(std::vector<Waypoint>& nodes_to_check, double flightTime){
    double T;
    double p_ref[3][1]= {0}, v_ref[3][1]= {0}, a_ref[3][1]= {0};
    double p[3][1]= {0}, v[3][1] = {0};
    double att[3][1]= {0}, att_q[4][1]= {0}, u[3][1]= {0};
    std::vector<double> gammaVectPos; std::vector<double> gammaVectVel; std::vector<double> gammaVectAcc;
    std::vector<double> posB; 
    std::vector<double> velB;  
    std::vector<double> accB;

    followingStrategyBaseClass::fly(nodes_to_check, flightTime);

    ROS_INFO("Buffer size = %li", nodes_to_check.size());
    if(nodes_to_check.size() <= 4)
        return 0;
    
    if(nodes_to_check.size() >= 8)
        blockWpGeneration();
    else
        unblockWpGeneration();

    //bezierFlightTime = flightTime - lastFlyCallTime;
    // double gammaMax = std::floor((nodes_to_check.size()-1)/4);
    // if(nodes_to_check.size() == 0) gammaMax = 0;
    double gammaMax = std::floor(gamma+1);
    updateGamma(1.0/30.0, gammaMax);

    double gammaNormalized = gamma - std::floor(gamma); // between 0-->1

    floorTime = std::floor(gamma);
    wpVectorHanlder(nodes_to_check);

    // ROS_INFO("flightTime = %f floorTime = %i", gamma, floorTime);

    std::vector<double> nodes_to_check_x;
    nodes_to_check_x.push_back(nodes_to_check[0].x); nodes_to_check_x.push_back(nodes_to_check[1].x); nodes_to_check_x.push_back(nodes_to_check[2].x); nodes_to_check_x.push_back(nodes_to_check[3].x);
    std::vector<double> nodes_to_check_y;
    nodes_to_check_y.push_back(nodes_to_check[0].y); nodes_to_check_y.push_back(nodes_to_check[1].y); nodes_to_check_y.push_back(nodes_to_check[2].y); nodes_to_check_y.push_back(nodes_to_check[3].y);

    // Position Reference
    std::vector<double> posRef = getCurvePosition(nodes_to_check_x, nodes_to_check_y, gammaNormalized);

    // Velocity Reference
    std::vector<double> curveVel = getCurveVelocity(nodes_to_check_x, nodes_to_check_y,gammaNormalized);

    // Acceleration Reference
    std::vector<double> curveAcc = getCurveAcceleration(nodes_to_check_x, nodes_to_check_y, gammaNormalized);
    
    gammaDot = getCurrentSpeedProfile(curveVel, curveAcc);   
    double dSpeedprofile_dt = getCurrentSpeedProfileGradient(curveVel, curveAcc, getCurve3rdGrad(nodes_to_check_x, nodes_to_check_y));

    ROS_INFO("Waypoints = [%f %f] [%f %f] [%f %f] [%f %f]", nodes_to_check_x[0], nodes_to_check_y[0], nodes_to_check_x[1], nodes_to_check_y[1], nodes_to_check_x[2], nodes_to_check_y[2], nodes_to_check_x[3], nodes_to_check_y[3]);

    p_ref[0][0] = posRef[0];
    v_ref[0][0] = gammaDot*curveVel[0];
    a_ref[0][0] = gammaDot*gammaDot*curveAcc[0] + curveVel[0]*dSpeedprofile_dt; //curveVel[0]*(gammaDot - lastSpeedProfile)/30; 

    p_ref[1][0] = posRef[1];
    v_ref[1][0] = gammaDot*curveVel[1];
    a_ref[1][0] = gammaDot*gammaDot*curveAcc[1]; +curveVel[1]*dSpeedprofile_dt; //curveVel[1]*(gammaDot - lastSpeedProfile)/30; ;
    // ROS_WARN("(gammaDot - lastSpeedProfile)/30 = %f", (gammaDot - lastSpeedProfile)/30);
    // ROS_WARN("Real gradient = %f", dSpeedprofile_dt);
    lastSpeedProfile = gammaDot;

    p_ref[2][0]=-1; v_ref[2][0]=0; a_ref[2][0]=0;

    //Saturações
    ROS_INFO("Acceleration = [%f %f %f]", a_ref[0][0], a_ref[1][0], a_ref[2][0]);
    ROS_WARN("Before Sat =%f  =%f", std::sqrt(v_ref[0][0]*(v_ref[0][0]) + v_ref[1][0]*(v_ref[1][0])), std::sqrt(a_ref[0][0]*(a_ref[0][0]) + a_ref[1][0]*(a_ref[1][0])));
    velAccSaturation(&v_ref, &a_ref);
    ROS_WARN("After Sat =%f  =%f", std::sqrt(v_ref[0][0]*(v_ref[0][0]) + v_ref[1][0]*(v_ref[1][0])), std::sqrt(a_ref[0][0]*(a_ref[0][0]) + a_ref[1][0]*(a_ref[1][0])));
    ROS_INFO("Acceleration = [%f %f %f]", a_ref[0][0], a_ref[1][0], a_ref[2][0]);

    a_ref[0][0] = std::max(a_ref[0][0], -1.0);
    a_ref[0][0] = std::min(a_ref[0][0], 1.0);
    a_ref[1][0] = std::max(a_ref[1][0], -1.0);
    a_ref[1][0] = std::min(a_ref[1][0], 1.0);

    ROS_INFO("p=[%f %f] \n v=[%f %f] \n a=[%f %f]", p_ref[0][0], p_ref[1][0], v_ref[0][0], v_ref[1][0], a_ref[0][0], a_ref[1][0]);
    memcpy(p, uav->ekf.pos, sizeof(uav->ekf.pos));
    memcpy(v, uav->ekf.vel, sizeof(uav->ekf.vel));

    for (int i=0; i<=2; i++){
        u[i][0] = -kp[i]*(p[i][0]-p_ref[i][0]) -kd[i]*(v[i][0]-v_ref[i][0]) -ki[i]*integral[i][0] + uav->info.mass*a_ref[i][0];
    }
    
    droneController(u, att, &T, uav->info.mass);

    for (int i=0; i<=2; i++)
        integral[i][0] = integral[i][0] + (p[i][0]-p_ref[i][0])*1.0/30.0; 
    
    uav->set_att_thrust(att, att_q, "euler", T, 100.0);
    // ROS_INFO("att = %f %f %f T = %f", att[0][0], att[1][0], att[2][0], T);
    ROS_WARN("p_ref = [%f %f %f]", p_ref[0][0], p_ref[1][0], p_ref[2][0]);
    debugTopicPublication(p, p_ref, v, v_ref, att, flightTime, T);

    // lastFlyCallTime = ros::Time::now().toSec();
    return 1;
}

std::vector<Waypoint> bezierCurvesStrategy::wpVectorHanlder(std::vector<Waypoint>& nodes_to_check){

    ROS_WARN("floorTime = %i    lasFloorTime = %i delete = %i", floorTime, lastFloorTime, floorTime != lastFloorTime);
    if(floorTime != lastFloorTime){
        ROS_WARN("Waypoint Deleted!");
        nodes_to_check.erase(nodes_to_check.begin());
        lastFloorTime = floorTime;
    }

    return nodes_to_check;
}

double bezierCurvesStrategy::getCurrentSpeedProfile(std::vector<double> curveVelocity, std::vector<double> curveAcceleration){
    double curveVelocityNorm = 1, curveAccelerationNorm = 1;
    ROS_INFO("gammaDot = %f / (sqrt(%f^2 + %f^2)*(1+K*sqrt(%f^2 + %f^2))", VDesejado, curveVelocity[0], curveVelocity[1], curveAcceleration[0], curveAcceleration[1]);
    if(curveVelocity.size() == 2){
        curveVelocityNorm = offset + std::sqrt(curveVelocity[0]*curveVelocity[0] + curveVelocity[1]*curveVelocity[1]);
        curveAccelerationNorm = offset + std::sqrt(curveAcceleration[0]*curveAcceleration[0] + curveAcceleration[1]*curveAcceleration[1]);
        //if(curveVelocityNorm == 0) return 0;
    }

    else if(curveVelocity.size() == 3){
        curveVelocityNorm = offset + std::sqrt(curveVelocity[0]*curveVelocity[0] + curveVelocity[1]*curveVelocity[1] + curveVelocity[2]*curveVelocity[2]);
        curveAccelerationNorm = offset + std::sqrt(curveAcceleration[0]*curveAcceleration[0] + curveAcceleration[1]*curveAcceleration[1] + curveAcceleration[2]*curveAcceleration[2]);

        //if(curveVelocityNorm == 0) return 0;
    }
    else{
        return -1;
    }

    return VDesejado/(curveVelocityNorm*(1+K*curveAccelerationNorm));
}

void bezierCurvesStrategy::updateGamma(double delta_t, double gammaMax){
    ROS_INFO("Gamma updated from %f", gamma);
    gamma += delta_t*gammaDot;
    gamma = std::min(gamma, gammaMax);
    ROS_INFO("    %f         %f        to %f", gammaDot, gammaMax, gamma);

}

std::vector<double> bezierCurvesStrategy::getCurvePosition(std::vector<double> nodes_to_check_x, std::vector<double> nodes_to_check_y, double gammaNormalized){
    std::vector<double> gammaVectPos, posB, curvePosition;
    gammaVectPos.push_back(pow(gammaNormalized, 3)); gammaVectPos.push_back(pow(gammaNormalized, 2)); gammaVectPos.push_back(pow(gammaNormalized, 1)); gammaVectPos.push_back(1);

    for(int i = 0; i < 4; i++){
        posB.push_back(std::inner_product(gammaVectPos.begin(), gammaVectPos.end(), M[i].begin(), 0.0));
    }

    curvePosition.push_back(std::inner_product(posB.begin(), posB.end(), &nodes_to_check_x[0], 0.0));
    curvePosition.push_back(std::inner_product(posB.begin(), posB.end(), &nodes_to_check_y[0], 0.0));

    return curvePosition;
}

std::vector<double> bezierCurvesStrategy::getCurveVelocity(std::vector<double> nodes_to_check_x, std::vector<double> nodes_to_check_y, double gammaNormalized){
    std::vector<double> gammaVectVel, velB, curveVelocity;
    gammaVectVel.push_back(pow(gammaNormalized, 2)*3); gammaVectVel.push_back(gammaNormalized*2); gammaVectVel.push_back(1); gammaVectVel.push_back(0);
    // ROS_INFO("gammaVectVel = [%f %f %f %f]", gammaVectVel[0], gammaVectVel[1], gammaVectVel[2], gammaVectVel[3]);
    
    for(int i = 0; i < 4; i++){
        velB.push_back(std::inner_product(gammaVectVel.begin(), gammaVectVel.end(), M[i].begin(), 0.0));
        // ROS_INFO("gamma * [%f %f %f %f] = %f", M[i][0], M[i][1], M[i][2], M[i][3], std::inner_product(gammaVectVel.begin(), gammaVectVel.end(), M[i].begin(), 0.0));
    }

    curveVelocity.push_back(std::inner_product(velB.begin(), velB.end(), &nodes_to_check_x[0], 0.0));
    curveVelocity.push_back(std::inner_product(velB.begin(), velB.end(), &nodes_to_check_y[0], 0.0));

    // ROS_INFO("curveVelocity = [%f %f]", curveVelocity[0], curveVelocity[1]);
    return curveVelocity;
}

std::vector<double> bezierCurvesStrategy::getCurveAcceleration(std::vector<double> nodes_to_check_x, std::vector<double> nodes_to_check_y, double gammaNormalized){
    std::vector<double> gammaVectAcc, accB, curveAcceleration;
    gammaVectAcc.push_back(gammaNormalized*6); gammaVectAcc.push_back(2); gammaVectAcc.push_back(0); gammaVectAcc.push_back(0); 

    for(int i = 0; i < 4; i++){
        accB.push_back(std::inner_product(gammaVectAcc.begin(), gammaVectAcc.end(), M[i].begin(), 0.0));
    }

    curveAcceleration.push_back(std::inner_product(accB.begin(), accB.end(), &nodes_to_check_x[0], 0.0));
    curveAcceleration.push_back(std::inner_product(accB.begin(), accB.end(), &nodes_to_check_y[0], 0.0));

    return curveAcceleration;
}

std::vector<double> bezierCurvesStrategy::getCurve3rdGrad(std::vector<double> nodes_to_check_x, std::vector<double> nodes_to_check_y){
    std::vector<double> gammaVect3rdGrad, B, curve3rdGrad;
    gammaVect3rdGrad.push_back(6); gammaVect3rdGrad.push_back(0); gammaVect3rdGrad.push_back(0); gammaVect3rdGrad.push_back(0); 

    for(int i = 0; i < 4; i++){
        B.push_back(std::inner_product(gammaVect3rdGrad.begin(), gammaVect3rdGrad.end(), M[i].begin(), 0.0));
    }

    curve3rdGrad.push_back(std::inner_product(B.begin(), B.end(), &nodes_to_check_x[0], 0.0));
    curve3rdGrad.push_back(std::inner_product(B.begin(), B.end(), &nodes_to_check_y[0], 0.0));

    return curve3rdGrad;
}

double bezierCurvesStrategy::getCurrentSpeedProfileGradient(std::vector<double> curveVel, std::vector<double> curveAcc, std::vector<double> curve3rdGrad){
    double curveVelNorm = offset + std::sqrt(curveVel[0]*curveVel[0] + curveVel[1]*curveVel[1]);
    double curveAccNorm = offset + std::sqrt(curveAcc[0]*curveAcc[0] + curveAcc[1]*curveAcc[1]);

    double dNorm_dGamma = std::inner_product(curveVel.begin(), curveVel.end(), curveAcc.begin(), 0.0)/curveVelNorm * (1+K*curveAccNorm) + K*std::inner_product(curveAcc.begin(), curveAcc.end(), curve3rdGrad.begin(), 0.0)/curveAccNorm * curveVelNorm; //gradient of curveVel norm
    
    double dSpeedprofile_dGamma = -VDesejado*dNorm_dGamma/std::pow(curveVelNorm*(1+K*curveAccNorm), 2.0);
    double dSpeedprofile_dt = dSpeedprofile_dGamma*gammaDot;

    ROS_INFO("curveVelNorm = %f  curveAccNorm=%f  dNorm_dGamma=%f  dSpeedprofile_dGamma=%f  dSpeedprofile_dt=%f", curveVelNorm, curveAccNorm, dNorm_dGamma, dSpeedprofile_dGamma, dSpeedprofile_dt);

    return dSpeedprofile_dt;
}

void bezierCurvesStrategy::velAccSaturation(double (*velocity)[3][1], double (*acceleration)[3][1]){
    ROS_INFO("[Sat] Acceleration = [%f %f %f]", (*acceleration)[0][0], (*acceleration)[1][0], (*acceleration)[2][0]);
    double VelNorm = offset + std::sqrt((*velocity)[0][0]*(*velocity)[0][0] + (*velocity)[1][0]*(*velocity)[1][0]);
    double AccNorm = offset + std::sqrt((*acceleration)[0][0]*(*acceleration)[0][0] + (*acceleration)[1][0]*(*acceleration)[1][0]);
    
    double auxVariation = VelNorm - lastVelRefNorm;
    auxVariation = std::min(auxVariation, 2.0);
    auxVariation = std::max(auxVariation, -2.0);
    double correctionFactor = (lastVelRefNorm + auxVariation) / VelNorm;
    if(correctionFactor!= 1) ROS_WARN("!!Vel!!");
    (*velocity)[0][0] *= correctionFactor; (*velocity)[1][0] *= correctionFactor;

    auxVariation = AccNorm - lastAccRefNorm;
    auxVariation = std::min(auxVariation, 1.0/30.0);
    auxVariation = std::max(auxVariation, -1.0/30.0);
    correctionFactor = (lastAccRefNorm + auxVariation) / AccNorm;
    if(correctionFactor!= 1) ROS_WARN("!!Acc!! %f %f %f %f %f %f", AccNorm, lastAccRefNorm, AccNorm - lastAccRefNorm,  auxVariation, correctionFactor);
    (*acceleration)[0][0] *= correctionFactor; (*acceleration)[1][0] *= correctionFactor;

    lastVelRefNorm = offset + std::sqrt((*velocity)[0][0]*(*velocity)[0][0] + (*velocity)[1][0]*(*velocity)[1][0]);;
    lastAccRefNorm = offset + std::sqrt((*acceleration)[0][0]*(*acceleration)[0][0] + (*acceleration)[1][0]*(*acceleration)[1][0]);
    ROS_WARN("Saved acc = %f %f", lastAccRefNorm, offset + std::sqrt((*acceleration)[0][0]*(*acceleration)[0][0] + (*acceleration)[1][0]*(*acceleration)[1][0]));
}