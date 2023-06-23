#pragma once

#include "DroneInfo.h"
#include "DroneStatus.h"

#include <string>
#include <std_msgs/Float64MultiArray.h>

/**
 * @brief Namespace DroneLib to be used with every library inside the drone_library package
 */
namespace DroneLib {

  /**
   * @brief Function to convert from ned frame to enu frame
   *
   * @param v_ned An array with the coordinates in ned frame
   * @param v_enu An array with the coordinates in enu frame
   */
  void ned_to_enu(double v_ned[3][1], double v_enu[3][1]);

  /**
   * @brief Function to convert from enu frame to ned frame
   *
   * @param v_enu An array with the coordinates in enu frame
   * @param v_ned An array with the coordinates in ned frame
   */
  void enu_to_ned(double v_enu[3][1], double v_ned[3][1]);

  /**
   * @brief Converts a SI quaternion in NED coordinates to a ROS quaternion in ENU coordinates
   * 
   * @param q_si  An array with Quaternion in SI unit
   * @param q_ros An array with Quaternion in ROS units
   */
  void SI_quaternion_to_ROS_quaternion(double q_si[4][1], double q_ros[4]);

  /**
   * @brief Converts a ROS quaternion in ENU coordinates to a SI quaternion in NED coordinates.
   *
   * @param q_ros Quaternion in the ROS format and ENU coordinates.
   * @param q_si Variable that will store the quaternion in the SI format and NED coordinates.
   */
  void ROS_quaternion_to_SI_quaternion(double q_ros[4], double q_si[4][1]);

  /**
   * @brief Converts the thrust in newtons to a normalized value between 0 and 1 through the mathematical expression of the thrust curve of the drone.
   *
   * @param thrust_newtons The thrust to be applied in newtons
   * @param thrust_curve A string with the thrust curve
   * @param vel Norm of the linear velocity of the drone
   *
   * @return Normalized thrust, between 0 and 1
   */
  double normalize_thrust(double thrust_newtons, std::string thrust_curve, double vel);

  /**
   * @brief Converts a one dimensional ROS message with 3*n elements into an array with dimension nx3x1.
   *
   * @param msg One dimensional ROS message with 3*n elements containing a list of the relative position or velocity of each of the n neighbour vehicles, in NED coordinates.
   * @param rel_param Relative position or velocity of each of the n neighbour vehicles, in NED coordinates.
   */
  void ROS_list_to_array(const std_msgs::Float64MultiArray::ConstPtr& msg, double rel_param[100][3][1]);

};
