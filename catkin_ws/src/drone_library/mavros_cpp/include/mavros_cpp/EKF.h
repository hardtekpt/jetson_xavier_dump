#pragma once


/**
 * @brief Namespace DroneLib to be used with every library inside the drone_library package
 */
namespace DroneLib {

  /**
   * @brief Class used to store the state of the vehicle provided by the extended Kalman filter of the PX4 autopilot. All variables are in SI units.
   */
  class EKF {

    public:
      /**
       * @brief Position of the vehicle, in local NED coordinates, provided by the extended Kalman filter of the PX3 autopilot.
       */
      double pos[3][1];

      /**
       * @brief Linear velocity of the drone, in local NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.
       */
      double vel[3][1];

      /**
       * @brief Linear velocity of the vehicle, in body NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.
       */
      double vel_body[3][1];

      /**
       * @brief Attitude of the vehicle, expressed in quaternions, provided by the extended Kalman filter of the PX4 autopilot.
       */
      double att_q[4][1];

      /**
       * @brief Attitude of the drone, expressed in Euler angles, provided by the extended Kalman filter of the PX4 autopilot.
       */
      double att_euler[3][1];

      /**
       * @brief Angular velocity of the vehicle provided by the extended Kalman filter of the PX4 autopilot.
       */
      double ang_vel[3][1];
  };
};
