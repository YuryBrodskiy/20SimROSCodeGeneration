#pragma once

/**********************************************************
 * This file is generated by 20-sim C++ Code Generator
 *
 *  file:  %FILE_NAME%
 *  subm:  %SUBMODEL_NAME%
 *  model: %MODEL_NAME%
 *  expmt: %EXPERIMENT_NAME%
 *  date:  %GENERATION_DATE%
 *  time:  %GENERATION_TIME%
 *  user:  %USER_NAME%
 *  from:  %COMPANY_NAME%
 *  build: %GENERATION_BUILD%
 *
 **********************************************************/

/* This file describes the model functions
 that are supplied for computation.

 The model itself is the %SUBMODEL_NAME%Model.cpp file
 */

#include "%SUBMODEL_NAME%Model.hpp"

/* ROS include files */
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "Adapter20Sim.h"

//#define COMPUTATION_TIME_MEASUREMENT 1

#ifdef COMPUTATION_TIME_MEASUREMENT
//#include <rtt/os/TimeService.hpp>
//#include <rtt/Time.hpp>
#endif

namespace %MODEL_NAME%
{
	using namespace common20sim;

	class %SUBMODEL_NAME%: private %SUBMODEL_NAME%Model
	{
	public:
		/**
		 * %SUBMODEL_NAME% constructor
		 */
		%SUBMODEL_NAME%(std::string name = "%SUBMODEL_NAME%");

		/**
		 * %SUBMODEL_NAME% destructor
		 */
		virtual ~%SUBMODEL_NAME%(void);

    /**
     * %SUBMODEL_NAME% startUp code
     */
    void start();

		/**
		 * ROS main-loop.
		 */
		void run();

		/**
		 * %SUBMODEL_NAME% Terminate code
		 */
		void stop();

		double getTime(void);

		virtual bool setPeriod(double period);

	protected:
    virtual void CopyInputsToVariables();

    virtual void CopyVariablesToOutputs();

    void setupComponentInterface();

    ros::NodeHandle m_node_handle;

		/**
		 * ROS topics for input and ouput
		 */
		std::vector< Adapter20Sim<ros::Subscriber>* > m_input_ports;
		std::vector< Adapter20Sim<ros::Publisher>* > m_output_ports;
//		std::vector< Adapter20Sim<RTT::Property<RTT::types::carray<double> > > > propertyPorts;

	private:
		std::string m_config_file;
		ros::Rate m_loop_rate;

#ifdef COMPUTATION_TIME_MEASUREMENT
	  RTT::Seconds m_cum_avg;
	  unsigned int m_cum_avg_counter;
#endif

	};

}
