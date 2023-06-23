#pragma once

#include <string>

/**
 * @brief Namespace DroneLib to be used with every library inside the drone_library package
 */
namespace DroneLib {

  /**
   * @brief class used to store physical properties and the flight status of the drone. all variables are in si units. ros namespace where the data from the px4 autopilot and the mocap system is encapsulated.
   */
  class DroneInfo {

    public:

      /**
       * @brief Drone namespace in ROS
       */
      std::string drone_ns;

      /**
       * @brief Drone ID, usefull when considering multiple vehicles
       */
      int ID;

      /**
       * @brief Mass of the drone
       */
      double mass;

      /**
       * @brief Radius of the drone
       */
      double radius;

      /**
       * @brief Height of the drone
       */
      double height;

      /**
       * @brief Number of rotor of the drone
       */
      int num_rotors;

      /**
       * @brief The gravity acceleration value
       */
      double g;

      /**
       * @brief Thrust curve of the drone
       */
      std::string thrust_curve;
  };
};
