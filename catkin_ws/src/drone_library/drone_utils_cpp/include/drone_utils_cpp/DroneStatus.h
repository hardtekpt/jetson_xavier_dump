#pragma once

#include <string>

/**
 * @brief Namespace DroneLib to be used with every library inside the drone_library package
 */
namespace DroneLib {
  /**
   * @brief class used to store the flight status of the drone. 
   */
  class DroneStatus {

    public: 
      /**
       * @brief Current flight mode of the drone.
       * The list of flight modes is available at 
       * http://wiki.ros.org/mavros/CustomModes.
       */
      std::string flight_mode{""};

      /**
       * @brief States if the system is connected to the PX4 autopilot. 
       */
      bool is_connected{false};

      /**
       * @brief Stores the armed state of the vehicle. If True, the drone is armed.
       */
      bool is_armed{false};

      /**
       * @brief Stores the landed state of the vehicle. If True, the drone is landed.
       */
      bool is_landed{true};

      /**
       * @brief Remaining battery percentage.
       */
      double battery;
  };

};
