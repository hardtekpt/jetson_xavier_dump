#include "waypoint_following/followingStrategyBaseClass.h"

followingStrategyBaseClass::followingStrategyBaseClass(){
	// Ros node pointer
    nh = new ros::NodeHandle();

    debug_pub = nh->advertise<waypoint_following::DebugMSG>("debug", 10);
	landingPosition.x = 0; landingPosition.y = 0; landingPosition.z = 0;

	//Topic publishing
	stopWpGeneration_pub = nh->advertise<waypoint_following::StopWpGenerationMSG>("stopWpGeneration", 10);
	
	blockWpGeneration();
}

bool followingStrategyBaseClass::fly(std::vector<Waypoint>& nodes_to_check, double flightTime){
	isHover = false;
}

void followingStrategyBaseClass::rvizPublish(std::vector<Waypoint> nodes_to_check){

}

std::vector<Waypoint> followingStrategyBaseClass::wpVectorHanlder(std::vector<Waypoint> &nodes_to_check){


}

void followingStrategyBaseClass::updateLandingPosition(Waypoint landingPoint, std::string frame){
	if(frame.compare("enu") == 0){
		// ENU to NED conversion
		ROS_INFO("Landing Point in NED frame. Changing to ENU frame.");
		double enuPoint[3][1];
		enuPoint[0][0] = landingPoint.x; enuPoint[1][0] = landingPoint.y; enuPoint[2][0] = landingPoint.z;
		double nedPoint[3][1];
		enuToNed(enuPoint, nedPoint);
		landingPoint.x = nedPoint[0][0];
		landingPoint.y = nedPoint[1][0];
		landingPoint.z = nedPoint[2][0];
	}
	else{
		if(frame.compare("ned") != 0){
			ROS_WARN("Landing Position updadate was ignored, becausa frame was either 'enu' not 'ned'");
			return;
		}
	}

	landingPosition.x = landingPoint.x; landingPosition.y = landingPoint.y; landingPosition.z = landingPoint.z;

	landingPosition.x = landingPoint.x; landingPosition.y = landingPoint.y; landingPosition.z = landingPoint.z;
	ROS_INFO("Landing location updated to [%f %f %f]", landingPosition.x, landingPosition.y, landingPosition.z);
}

void followingStrategyBaseClass::updateLandingPosition(double landingPoint[3][1], std::string frame){
	Waypoint newLandingPoint;
	newLandingPoint.x = landingPoint[0][0]; newLandingPoint.y = landingPoint[1][0]; newLandingPoint.z = landingPoint[2][0];

	updateLandingPosition(newLandingPoint, frame);
}

void followingStrategyBaseClass::droneController(double u[3][1], double att[3][1] , double *T, double mass){
    double g = 9.8066, T_aux;
	double Re3[3][1]  = {0.0, 0.0, 0.0}, att_aux[3][1] = {0.0, 0.0, 0.0};
	T_aux = sqrt(pow(-u[0][0], 2) + pow(-u[1][0], 2) + pow(mass*g-u[2][0], 2));

	Re3[0][0]=-u[0][0]/T_aux; Re3[1][0]=-u[1][0]/T_aux; Re3[2][0]=(mass*g-u[2][0])/T_aux;

	att_aux[0][0]=asin(-Re3[1][0]); att_aux[1][0]=atan(Re3[0][0]/Re3[2][0]) ; att_aux[2][0]=0;

	*T = T_aux;
	memcpy(att, att_aux, sizeof(att_aux));

	return;
}

void followingStrategyBaseClass::enuToNed(double v_enu[3][1], double v_ned[3][1]){
    double v_aux[3][1] = {{v_enu[1][0]},{v_enu[0][0]},{-v_enu[2][0]}};
	memcpy(v_ned, v_aux, sizeof(v_aux));
}

void followingStrategyBaseClass::resetIntegral(){
	integral[0][0] = 0.0; integral[1][0] = 0.0; integral[2][0] = 0.0;
}

void followingStrategyBaseClass::land(){
	double t1, t2;
	double a=0.5;
	double p_aux[3]= {0}, p_avg[3]= {0}, p[3][1] = {0};

	blockWpGeneration();

	memcpy(p, uav->ekf.pos, sizeof(uav->ekf.pos));

	double yaw = 0;
	double pos[3][1] = {landingPosition.x, landingPosition.y, landingPosition.z};

	ROS_INFO("Drone is going to land position. It will take %f seconds", t1+t2);
	while(abs(p[0][0]-landingPosition.x) > 0.1 && abs(p[1][0]-landingPosition.y) > 0.1){
		uav->set_pos_yaw(pos, yaw, 1.0);
		memcpy(p, uav->ekf.pos, sizeof(uav->ekf.pos));
	}
	uav->set_pos_yaw(pos, yaw, 2.0);
	uav->auto_land();
}

//Returns total time drone is hovering
double followingStrategyBaseClass::hover(){
	if(!isHover){
		memcpy(hoverPosition, uav->ekf.pos, sizeof(uav->ekf.pos));
		ROS_INFO("Drone will hover at [%f %f %f]]!", hoverPosition[0][0], hoverPosition[1][0], hoverPosition[2][0]);
		isHover = true;
		hoverStartTime = ros::Time::now().toSec();
	}
	
	// ROS_INFO("Hover!");	
	double yaw = 0;
	// ROS_INFO("hover at [%f %f %f]]!", hoverPosition[0][0], hoverPosition[1][0], hoverPosition[2][0]);
	// uav->set_vel_yaw()
    uav->set_pos_yaw(hoverPosition, yaw, 1.0/40);

	return ros::Time::now().toSec() - hoverStartTime;
}

void followingStrategyBaseClass::debugTopicPublication(double realPos[3][1], double refPos[3][1], double realVel[3][1], double refVel[3][1], double _att[3][1], double time, double thrust){
    waypoint_following::DebugMSG debug_msg;

    debug_msg.pos[0] = realPos[0][0];debug_msg.pos[1] = realPos[1][0];debug_msg.pos[2] = realPos[2][0];
	debug_msg.pos_ref[0] = refPos[0][0];debug_msg.pos_ref[1] = refPos[1][0];debug_msg.pos_ref[2] = refPos[2][0];
	debug_msg.v[0] = realVel[0][0];debug_msg.v[1] = realVel[1][0];debug_msg.v[2] = realVel[2][0];
	debug_msg.v_ref[0] = refVel[0][0];debug_msg.v_ref[1] = refVel[1][0];debug_msg.v_ref[2] = refVel[2][0];
	debug_msg.att[0] = _att[0][0];debug_msg.att[1] = _att[1][0];debug_msg.att[2] = _att[2][0];
	debug_msg.t = time;
	debug_msg.thrust = thrust;
	
	debug_pub.publish(debug_msg);	
}

void followingStrategyBaseClass::blockWpGeneration(){
	if(!wpGenerationIsStoped){
		wpGenerationIsStoped = true;
		waypoint_following::StopWpGenerationMSG stopWpGenerationToSend;
		stopWpGenerationToSend.stop = true;
		stopWpGeneration_pub.publish(stopWpGenerationToSend);
		ROS_INFO("Waypoint generation requested to block");
	}
}

void followingStrategyBaseClass::unblockWpGeneration(){
	if(wpGenerationIsStoped){
		wpGenerationIsStoped = false;
		waypoint_following::StopWpGenerationMSG stopWpGenerationToSend;
		stopWpGenerationToSend.stop = false;
		stopWpGeneration_pub.publish(stopWpGenerationToSend);
		ROS_INFO("Waypoint generation requested to unblock");
	}
}

// Take off and keep same position with desired altitude and yaw angle for specified time [in sec]
// Side efect: blocks code for hoverTime seconds
void followingStrategyBaseClass::takeOff(double hoverTime, double yaw, double altitude){
	double pos[3][1];

	ROS_INFO("Drone will Take-off");
	ROS_INFO("drone is at %p", uav);
	memcpy(pos, uav->ekf.pos, sizeof(uav->ekf.pos));
	pos[2][0] = -1;
	ROS_INFO("Drone Initial Position = [%f %f %f]", pos[0][0], pos[1][0], pos[2][0]);

	updateLandingPosition(pos, "ned");

	uav->start_offboard_mission();
	uav->set_pos_yaw(pos, yaw, hoverTime);

}

//Fly drone to initial mission position
void followingStrategyBaseClass::moveToInitialMissionPosition(double pos[3][1], std::string posFrame, double yaw){
	ROS_INFO("Drone will flight to initial mission position");
	
	if(posFrame.compare("enu") == 0){
		// ENU to NED conversion
		enuToNed(pos, pos);
	}

	double p[3][1], v[3][1], att[3][1] = {0.0};
	memcpy(p, uav->ekf.pos, sizeof(uav->ekf.pos));
	memcpy(v, uav->ekf.vel, sizeof(uav->ekf.vel));

	while(abs(p[0][0]-pos[0][0]) > 0.1 && abs(p[1][0]-pos[1][0]) > 0.1){
		uav->set_pos_yaw(pos, yaw, 1.0);
		memcpy(p, uav->ekf.pos, sizeof(uav->ekf.pos));
	}

	uav->set_pos_yaw(pos, yaw, 2.0);

	ROS_INFO("Drone is at initial position and ready do fly a mission");

	//Waypoints generation can now start
	unblockWpGeneration();

	debugTopicPublication(pos, p, v, v, att, 0, 0);
}