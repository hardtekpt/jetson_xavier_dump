#pragma once

/**
 * @brief Namespace DroneLib to be used with every library inside the drone_library package
 */
namespace DroneLib {

  /**
   * @brief Class used to store the current values applied to the mixer and/or actuators (motors and control devices) of the drone.
   * More information available at https://dev.px4.io/v1.9.0/en/concept/mixing.html.
   */
  class Actuators {

    public:
      /**
       * @brief States the group of the active motors and servos of the drone.
       */
      int group;

      /**
       * @brief Stores the normalized values (0 to 1 or -1 to 1) applied to the mixer and/or motors and servos of the vehicle.
       */
      double output[8];
  };

};
