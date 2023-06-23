#ifndef CATKIN_WS_DRONEGIMMICKSNODE_H
#define CATKIN_WS_DRONEGIMMICKSNODE_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

/**
 * @brief DroneGimmicks namespace
 * 
 * @note why the code of templates is here -> because linkage problems see https://stackoverflow.com/a/1353981
 */
namespace DroneGimmicks{

	/**
	 * @brief Get the Parameters object
	 * 
	 * @tparam T the type of data of a desired parameter
	 * @param _nh ros nodehandle
	 * @param parameter_name string with paramenter name
	 * @return T parameter value
	 * 
	 * @note Option not considering default value, so the config file must have the parameter;
	 */
	template <typename T> T getParameters(ros::NodeHandle &_nh, std::string const &parameter_name){
		T parameter;
		
		if (!_nh.getParam(parameter_name, parameter)) {
			ROS_ERROR("No parameter [%s] shutting down", parameter_name.c_str());
			ros::shutdown();
		}
		
		// +.+ Note: this was giving problems with vectors
		_nh.getParam(parameter_name, parameter);

		return parameter;
	}

	/**
	 * @brief Get the Parameters object
	 * 
	 * @tparam T the type of data of a desired parameter
	 * @param _nh ros nodehandle
	 * @param parameter_name string with parameter name
	 * @param default_value default value of the parameter
	 * @return T parameter value
	 * 
	 *  @note Option considering default value. Even if the parameter doesn't exist in config file it is possible to use a default value.
	 */
	template <typename T> T getParameters(ros::NodeHandle &_nh, std::string const &parameter_name, T default_value){
		
		T parameter;
		
		if (!_nh.getParam(parameter_name, parameter)) {
			parameter = default_value;
		}
		else{
			_nh.getParam(parameter_name, parameter);
		}
		return parameter;
	}

	/**
	 * @brief Get the Parameters object
	 * 
	 * @tparam T the type of data of a desired parameter
	 * @param _nh  ros nodehandle 
	 * @param parameter_name string with parameter name
	 * @param default_value default value of the parameter
	 * @param delete_param boolean to delete or not the parameter from parameter server
	 * @return T parameter value
	 * 
	 * @note Option considering default value. Even if the parameter doesn't exist in config file it is possible to use a default value. Removes parameter from parameter server
	 */
	template <typename T> T getParameters(ros::NodeHandle &_nh, std::string const &parameter_name, T default_value, bool delete_param){
		
		T parameter;
		
		if (!_nh.getParam(parameter_name, parameter)) {
			parameter = default_value;
		}
		else{
			_nh.getParam(parameter_name, parameter);
		}

		// +.+ Delete the param if flag is set true
		if (delete_param){
			_nh.deleteParam(parameter_name);
		}

		return parameter;
	}
	
	/**
	 * @brief 
	 * 
	 * @tparam A value type
	 * @tparam B publisher type
	 * @param pub publisher
	 * @param value value
	 */
	template <typename A, typename B>
	void publishValue(ros::Publisher& pub, B& value)
	{
		A aux_;
		aux_.data = value;
		pub.publish(aux_);
	}

};

#endif //CATKIN_WS_DRONEGIMMICKSNODE_H
