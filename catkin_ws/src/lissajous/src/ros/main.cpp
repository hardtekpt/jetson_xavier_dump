#include <drone_gimmicks_library/DroneGimmicks.h>
#include <ros/ros.h>
#include <drone_utils_cpp/DroneInfo.h>
#include <mavros_cpp/UAV.h>


ros::NodeHandle * nh;
ros::NodeHandle * nh_p;

DroneLib::UAV * uav = nullptr;

DroneLib::DroneInfo drone_info;


int main (int argc, char ** argv){
    /* Initiate the node */
    ros::init(argc, argv, "lissajous_node");
    nh = new ros::NodeHandle();
    nh_p = new ros::NodeHandle("~");

    /* Get the namespace of the drone and other parameters */
    drone_info.drone_ns = DroneGimmicks::getParameters<std::string>(*nh, "namespace");
    drone_info.mass = DroneGimmicks::getParameters<double>(*nh, "drone_params/mass");
    drone_info.radius = DroneGimmicks::getParameters<double>(*nh, "drone_params/radius");
    drone_info.height = DroneGimmicks::getParameters<double>(*nh, "drone_params/height");
    drone_info.num_rotors = DroneGimmicks::getParameters<int>(*nh, "drone_params/num_motors");
    drone_info.g = DroneGimmicks::getParameters<double>(*nh, "drone_params/g");
    drone_info.thrust_curve = DroneGimmicks::getParameters<std::string>(*nh, "drone_params/thrust_curve"); 


    /* Create the drone object */
    uav = new DroneLib::UAV(drone_info.drone_ns, 
        drone_info.mass, 
        drone_info.radius, 
        drone_info.height, 
        drone_info.num_rotors, 
        drone_info.thrust_curve,
        nh, nh_p); 


    double pos[3][1];
	double yaw, t0, t;

    ros::Duration(20).sleep();
    ROS_WARN_STREAM("Controller will start");

    

    pos[0][0]=0.0; pos[1][0]=1.6; pos[2][0]=-0.6;
	yaw = 0.0;
	//uav->set_pos_yaw(pos, yaw, 6);

    uav->start_offboard_mission();

    uav->set_pos_yaw(pos, yaw, 10);

    t0 = ros::Time::now().toSec();
	t = ros::Time::now().toSec()-t0;

    while(t<60){
		pos[0][0]=2.6*sin(2*0.55*t); pos[1][0]=1.6*sin(3*0.55*t+M_PI/2); pos[2][0]=-1.2+0.6*cos(0.55*t);
		(t<40) ? yaw=yaw+4.0/30.0*M_PI/180.0 : yaw=yaw-16.0/30.0*M_PI/180.0;
		uav->set_pos_yaw(pos, yaw, 1.0/30.0);
		t = ros::Time::now().toSec()-t0;

        ROS_WARN_STREAM(uav->ekf.pos[0][0] << uav->ekf.pos[1][0] << uav->ekf.pos[2][0]);
	}

	uav->set_pos_yaw(pos, yaw, 1);
	uav->auto_land();



    return 0;
}