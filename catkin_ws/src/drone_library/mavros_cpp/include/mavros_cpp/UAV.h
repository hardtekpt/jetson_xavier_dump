#pragma once

/* C++ libraries */
#include <thread>
#include <math.h>

/* Custome libraries */
#include <drone_utils_cpp/DroneInfo.h>
#include <drone_utils_cpp/DroneStatus.h>
#include "Actuators.h"
#include "EKF.h"
#include "Sensors.h"

/* ROS libraries */
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/* ROS messages*/
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/Altitude.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <mavros_msgs/ExtendedState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ActuatorControl.h>

/**
 * @brief Namespace DroneLib to be used with every library inside the drone_library package
 */
namespace DroneLib {

  /**
   * @brief Auxiliary class to store the data of the UAV
   */
  class UAV_AUX {

    public:
      /**
       * @brief Stores the state of the vehicle provided by the extended Kalman filter of the PX4 autopilot.
       */
      EKF ekf;

      /**
       * @brief Stores the raw sensor measurements and the MOCAP pose of the drone.
       */
      Sensors sen;

      /**
       * @brief Stores the current values applied to the mixers and/or actuators (motors and control devices) of the vehicle.
       */
      Actuators act;

      /**
       * @brief Stores physical properties and the flight status of the vehicle.
       */
      DroneInfo info;

      /**
       * @brief Stores the flight status of the vehicle
       */
      DroneStatus status; 

      /**
       * @brief ROS nodehandle to set publishers, subscribers, services and parameters
       */
      ros::NodeHandle nh;
      ros::NodeHandle nh_p;
  };


  /**
   * @brief Stores the methods that send offboard commands and offboard control references to the PX4 autopilot of the vehicle.
   */
  class OFFBOARD: public UAV_AUX {

    public:
      /**
       * @brief Set the publishers for seting references for the controller
       */
      ros::Publisher set_pos_pub, set_vel_pub, set_att_pub, set_ang_vel_pub, set_act_pub;

      /**
       * @brief Arms the drone, if it is not already armed.
       */
      void arm_drone();

      /**
       * @brief Changes the flight mode of the PX4 autopilot of the drone to offboard.
       */
      void start_offboard_mode();

      /**
       * @brief Makes the vehicle ready for an offboard experiment by arming it and by changing the flight mode of its PX4 autopilot to offboard.
       */
      void start_offboard_mission();

      /**
       * @brief Offboard method that sends position and yaw references to the PX4 autopilot of the the drone.
       */
      void set_pos_yaw(double pos[3][1], double yaw, double time);

      /**
       * @brief Offboard method that sends velocity and yaw references to the PX4 autopilot of the the vehicle.
       */
      void set_vel_yaw(double vel[3][1], double yaw, double freq);

      /**
       * @brief Offboard method that sends velocity_body and yaw_rate references to the PX4 autopilot of the the drone.
       */
      void set_vel_body_yaw_rate(double vel_body[3][1], double yaw_rate, double freq);

      /**
       * @brief Offboard method that sends attitude and thrust references to the PX4 autopilot of the the vehicle.
       */
      void set_att_thrust(double att_euler[3][1], double att_q[4][1], std::string att_type, double thrust, double freq);

      /**
       * @brief Offboard method that sends angular velocity and thrust references to the PX4 autopilot of the the drone.
       */
      void set_ang_vel_thrust(double ang_vel[3][1], double thrust, double freq);

      /**
       * @brief Offboard method that sets the values of the mixers and/or actuators of the vehicle.
       */
      void set_act(int group, double output[8], double freq);

      /**
       * @brief Disarms the vehicle, if it is not already disarmed.
       */
      void disarm_drone();

      /**
       * @brief Lands the drone, changing its flight mode to auto-land.
       */
      void auto_land();
  };


  /**
   * @brief Stores the methods responsible for keeping the variables of the UAV class up to date.
   */
  class TELEMETRY: public OFFBOARD {

    public:
      /**
       * @brief Manages topic subscriptions.
       */
      void init_telemetry();

      /**
       * @brief Updates the variable that stores the position of the drone, in local NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.
       */
      void update_position(const geometry_msgs::PoseStamped::ConstPtr& msg);

      /**
       * @brief Updates the variable that stores the linear velocity of the vehicle, in local NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.
       */
      void update_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg);

      /**
       * @brief Updates the variable that stores the linear velocity of the vehicle, in body NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.
       */
      void update_velocity_body(const geometry_msgs::TwistStamped::ConstPtr& msg);

      /**
       * @brief Updates the variables that store the attitude of the vehicle provided by the extended Kalman filter of the PX4 autopilot.
       */
      void update_attitude(const geometry_msgs::PoseStamped::ConstPtr& msg);

      /**
       * @brief Updates the variable that stores the angular velocity of the vehicle provided by the extended Kalman filter of the PX4 autopilot.
       */
      void update_angular_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg);

      /**
       * @brief Updates the variables that store the linear acceleration and the angular velocity of the drone measured by the IMU.
       */
      void update_imu(const sensor_msgs::Imu::ConstPtr& msg);

      /**
       * @brief Updates the variable that stores the magnetic field vector, in body NED coordinates, measured by the IMU.
       */
      void update_mag(const sensor_msgs::MagneticField::ConstPtr& msg);

      /**
       * @brief Updates the variables that store the position and attitude of the drone provided by the motion capture system.
       */
      void update_mocap(const geometry_msgs::PoseStamped::ConstPtr& msg);

      /**
       * @brief Updates the variable that stores the raw measurements provided by the GPS sensor.
       */
      void update_gps(const sensor_msgs::NavSatFix::ConstPtr& msg);

      /**
       * @brief Updates the variable that stores the static pressure measured by the barometer.
       */
      void update_baro_pressure(const sensor_msgs::FluidPressure::ConstPtr& msg);

      /**
       * @brief Updates the variable that stores the temperature, in degrees Kelvin, measured by the thermometer integrated in the barometer.
       */
      void update_baro_temperature(const sensor_msgs::Temperature::ConstPtr& msg);

      /**
       * @brief Updates the variable that stores the altitude of the vehicle, above mean sea level (MSL standard - different than ROS standard), computed through the barometric atmospheric pressure and the temperature.
       */
      void update_baro_altitude(const mavros_msgs::Altitude::ConstPtr& msg);

      /**
       * @brief Updates the variables that store the group and the current normalized values (0 to 1 or -1 to 1) applied to the mixer and/or motors and servos of the vehicle.
       */
      void update_actuator(const mavros_msgs::ActuatorControl::ConstPtr& msg);

      /**
       * @brief Updates the variables that store the current flight mode of the PX4 autopilot, the system status, and the armed state of the vehicle.
       */
      void update_status(const mavros_msgs::State::ConstPtr& msg);

      /**
       * @brief Updates the variable that stores the landed state of the drone.
       */
      void update_landed(const mavros_msgs::ExtendedState::ConstPtr& msg);

      /**
       * @brief Updates the variable that stores the remaining battery percentage.
       */
      void update_battery(const sensor_msgs::BatteryState::ConstPtr& msg);
  };

  /**
   * @brief Class used to represent the UAV
   */
  class UAV: public TELEMETRY {

    public:
      /**
       * @brief Constructor of the UAV class. Starts a background thread responsible for keeping all the variables of the UAV class up to date. 
       */
      UAV(std::string drone_ns, std::string mass, std::string radius, std::string height, std::string num_rotors, std::string thrust_curve, ros::NodeHandle * nh, ros::NodeHandle * nh_p);
  
      /**
       * @brief A second constructor for the UAV class 
       */
      UAV(std::string drone_ns, double mass, double radius, double height, int num_rotors, std::string thrust_curve, ros::NodeHandle * nh, ros::NodeHandle * nh_p); 
  };

};
