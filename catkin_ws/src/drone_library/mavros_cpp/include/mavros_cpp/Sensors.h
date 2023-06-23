#pragma once


/**
 * @brief Namespace DroneLib to be used with every library inside the drone_library package
 */
namespace DroneLib {

  /**
   * @brief Class used to store the raw measurements provided by the barometer. All variables are in SI units
   */
  class BAROMETER {

    public:
      /**
       * @brief Static pressure measured by the barometer
       */
      double pressure;

      /**
       * @brief Temperature, in degrees Kelvin, measured by the thermometer integrated in the barometer.
       */
      double temperature;

      /**
       * @brief Altitude of the vehicle, above mean sea level (MSL standard - differente than ROS standard), computed through the barometric atmospheric pressured and temperature
       */
      double alt;
  };

  /**
   * @brief Class used to store the raw measurements provided by the GPS sensor. All variables are in SI units.
   */
  class GPS {

    public:
      /**
       * @brief Position of the vehicle, in gps coordinates, provided by the GPS sensor
       */
      double pos[3][1];
  };

  /**
   * @brief Class used to store the raw measurements provided by the IMU. All variables are in SI units.
   */
  class IMU {

    public:
      /**
       * @brief Linear acceleration of the vehicle, in body NED coordinates, measured by the IMU.
       */
      double acc_body[3][1];

      /**
       * @brief Angular velocity of the drone measured by the IMU.
       */
      double ang_vel[3][1];

      /**
       * @brief Magnetic field vector, in body NED coordinates, measured by the IMU. Expressed in Teslas.
       */
      double mag[3][1];
  };

  /**
   * @brief Class used to store the position and attitude of the drone provided by the motion capture system. All variables are in SI units.
   */
  class MOCAP {

    public:
      /**
       * @brief Position of the vehicle, in local NED coordinates, provided by the motion capture system.
       */
      double pos[3][1];

      /**
       * @brief Attitude of the vehicle, expressed in quaternions, provided by the motion capture system.
       */
      double att_q[4][1];

      /**
       * @brief Attitude of the drone, expressed in Euler angles, provided by the motion capture system.
       */
      double att_euler[3][1];
  };

  /**
   * @brief Class to store data from neighbour vehicles
   */
  class EMULATED {

    public:
      /**
       * @brief array of doubles with dimension nx3x1. 
       * Relative position of each of the n neighbour vehicles, in local NED coordinates, 
       * provided by the emulated relative position sensor.
       */
      double rel_pos[100][3][1];

      /**
       * @brief rel_vel : array of doubles with dimension nx3x1.
       * Relative velocity of each of the n neighbour vehicles, in local NED coordinates, 
       * provided by the emulated relative velocity sensor.
       */
      double rel_vel[100][3][1];
  };


  /**
   * @brief Class used to store the raw sensors measurements and the MOCAP pose of the drone.
   */
  class Sensors {

    public:
      /**
       * @brief Stores the raw measurements provided by the IMU.
       */
      IMU imu;

      /**
       * @brief Stores the position and attitude of the drone provided by the motion capture system.
       */
      MOCAP mocap;

      /**
       * @brief Stores the raw measurements provided by the GPS sensor.
       */
      GPS gps;

      /**
       * @brief Stores the raw measurements provided by the barometer.
       */
      BAROMETER baro;

      /**
       * @brief Stores the raw measurements provided by emulated sensors.
       */
      EMULATED emu;
  };
};
