#include "mavros_cpp/UAV.h"

/* Constructor for the UAV class */
DroneLib::UAV::UAV(std::string drone_ns, std::string mass, std::string radius, std::string height, std::string num_rotors, std::string thrust_curve, ros::NodeHandle * nh, ros::NodeHandle * nh_p) {
  

  /* Save the drone intrinsic parameters */
	info.drone_ns = drone_ns;
	info.mass = std::stod(mass);
	info.radius = std::stod(radius);
	info.height = std::stod(height);
	info.num_rotors = std::stoi(num_rotors);
	info.thrust_curve = thrust_curve;

  this->nh = *nh;
  this->nh_p = *nh_p;

  /* Create a new thread in the background to fetch data from the relevant publishers */
  std::thread background_thread(&UAV::init_telemetry, this);
	background_thread.detach();

  ROS_INFO("[MAVROS_CPP] - Connecting to the drone...");
	while(!status.is_connected){
		ros::Rate rate_u(2.0); rate_u.sleep(); 
	}
	
  ROS_INFO("[MAVROS_CPP] - Connection established!");
	ros::Rate rate_d(1.0/5.0); rate_d.sleep();
}

/* Second constructor for the UAV class */
DroneLib::UAV::UAV(std::string drone_ns, double mass, double radius, double height, int num_rotors, std::string thrust_curve, ros::NodeHandle * nh, ros::NodeHandle * nh_p) {
  
  /* Save the drone intrinsic parameters */
  info.drone_ns = drone_ns;
  info.mass = mass;
  info.radius = radius;
  info.height = height;
  info.num_rotors = num_rotors;
  info.thrust_curve = thrust_curve;
  
  this->nh = *nh;
  this->nh_p = *nh_p;

  /* Create a new thread in the background to fetch data from the relevant publishers */
  std::thread background_thread(&UAV::init_telemetry, this);
	background_thread.detach();

  ROS_INFO("[MAVROS_CPP] - Connecting to the drone...");
	while(!status.is_connected){
		ros::Rate rate_u(2.0); rate_u.sleep(); 
	}
	
  ROS_INFO("[MAVROS_CPP] - Connection established!");
	ros::Rate rate_d(1.0/5.0); rate_d.sleep();
}
