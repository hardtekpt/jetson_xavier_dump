#include "drone_utils_cpp/Utils.h"
#include <ros/ros.h> //TODO - remove - only used for debugging

/* Method to convert from ENU to NED convention */
void DroneLib::enu_to_ned(double v_enu[3][1], double v_ned[3][1]){
  double v_aux[3][1] = {{v_enu[1][0]},{v_enu[0][0]},{-v_enu[2][0]}};
	memcpy(v_ned, v_aux, sizeof(v_aux));
}

/* Method to convert from NED to ENU convention*/
void DroneLib::ned_to_enu(double v_ned[3][1], double v_enu[3][1]){
	double v_aux[3][1] = {{v_ned[1][0]}, {v_ned[0][0]}, {-v_ned[2][0]}};
	memcpy(v_enu, v_aux, sizeof(v_aux));
}

/* Method to convert a SI quaternion in NED coordinated to a ROS quaternion in ENU coordinates */
void DroneLib::SI_quaternion_to_ROS_quaternion(double q_si[4][1], double q_ros[4]){
	
  double q_aux[4] = {q_si[2][0], q_si[1][0], -q_si[3][0], q_si[0][0]};
	memcpy(q_ros, q_aux, sizeof(q_aux));
}

/* Method to convert a SI quaternion in ENU coordinates to a SI quaternion in NED coordinates */
void DroneLib::ROS_quaternion_to_SI_quaternion(double q_ros[4], double q_si[4][1]){
	double q_aux[4][1] = {{q_ros[3]},{q_ros[1]},{q_ros[0]},{-q_ros[2]}};
	memcpy(q_si, q_aux, sizeof(q_aux));
}


/* Method to obtained normalized thrust to apply given a thrust curve */
double DroneLib::normalize_thrust(double thrust_newtons, std::string thrust_curve, double vel){
	
  double norm_thrust = 0.0;
 
	if (thrust_newtons <= 0) {
		return 0.0;
  } else if (thrust_curve.compare("iris") == 0) {
    		norm_thrust = (thrust_newtons/(1.0-vel/25.0) - 1.52*9.80665)/(2.0*34.068*0.561 + 7.1202) + 0.561;
  } else if (thrust_curve.compare("intel_aero") == 0) {
		norm_thrust = (tan((thrust_newtons-10.37)/8.84)+1.478)/2.955;
  } else if (thrust_curve.compare("snap_dragon") == 0) {
		norm_thrust = 0.1377*exp(0.02976*thrust_newtons)*sqrt(thrust_newtons) + 0.003759*thrust_newtons - 0.05973;
  } else if (thrust_curve.compare("typhoon") == 0) {
                norm_thrust = sqrt(thrust_newtons/(6 * 8.54858E-6)) / 1500;  
  }

	if (norm_thrust <= 0) {
		return 0.0;
	} else if (norm_thrust >= 1) {
		return 1.0;
  } else {
		return norm_thrust;
  }
}

/* Convert a list message into an array */
/*
void ROS_list_to_array(const std_msgs::Float64MultiArray::ConstPtr& msg, double rel_param[100][3][1]){
  
  double p_aux[100][3][1];

	for(int i=0; i<100; i++){
		try {p_aux[i][0][0]=msg->data[i*3];  p_aux[i][1][0]=msg->data[i*3+1]; p_aux[i][2][0]=msg->data[i*3+2];}
		catch (...) {break;} 
	}
	memcpy(rel_param, p_aux, sizeof(p_aux));
}
*/
