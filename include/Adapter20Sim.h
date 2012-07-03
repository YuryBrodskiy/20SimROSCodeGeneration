#pragma once

#include <vector>
#include <string>

#include "configuration/XVMatrix.hpp"

#include <std_msgs/Float64MultiArray.h>

namespace common20sim {

	using namespace ros;

	typedef std_msgs::Float64MultiArray flat_matrix_t;
	typedef std_msgs::Float64MultiArray::_data_type flat_matrix_internal_t;

	/**
	 * @brief Removes illegal characters from 20sim generated names.
	 * This was necessary for interpretation based software, like the TaskBrowser, because they
	 * use things as '.' to access sub-properties.
	 */
	std::string replaceIllegalCharacter(std::string str);
	std::string replaceIllegalCharacter(std::string str, std::string pattern, std::string replacement);

	template<class T>
	class Adapter20Sim
	{

	public:
		Adapter20Sim(XVMatrix& mat, std::string topic_name) :
			m_port(NULL), m_full_name(topic_name), m_matrix(mat), m_xx_data(mat.storage.mat), m_size(0)
		{
			// setup/resize m_data
			if(m_matrix.storage.columns != 0 && m_matrix.storage.rows != 0)
			{
			  m_size = m_matrix.storage.columns * m_matrix.storage.rows;
				m_port_data.data.resize(m_size, 0);
			}
			else
			{
				ROS_ERROR("XXMatrix settings unknown.");
				throw std::out_of_range("XXMatrix size unknown.");
			}

			assert(m_xx_data != NULL);
			assert(m_size != 0);
		}

//		Adapter20Sim(const Adapter20Sim& copy) :
//		  m_port(copy.m_port), m_matrix(copy.m_matrix), m_xx_data(copy.m_xx_data)
//		{
//			m_full_name = copy.m_full_name;
//			m_port_data = copy.m_port_data;
//			m_size = copy.m_size;
//		}

		virtual ~Adapter20Sim()
		{
         assert(m_xx_data == m_matrix.storage.mat);
		}

		std::string getTopicName()
		{
		  return m_full_name;
		}

		flat_matrix_t& getPortData()
		{
		  return m_port_data;
		}

		double* getXXData()
		{
		  return m_xx_data;
		}

		std::size_t getSize()
		{
		  return m_size;
		}

		T* getPort()
		{
			return m_port;
		}

    void setPort(T* port)
    {
      m_port = port;
    }

		// For ROS
		void subscribe_callback(const flat_matrix_t& data)
		{
		  m_port_data.data = data.data;
		}

		void copyPortToVariable()
		{
		  if(m_port_data.data.size() == m_size)
		    memcpy(m_xx_data, m_port_data.data.data(), m_size * sizeof(double));
		  else
		  {
				throw std::out_of_range("Input data size does not match variable data size.");
		  }
		}

		void copyVariableToPort()
		{
		  // No lock needed
		  if(m_port_data.data.size() == m_size)
		  {
      	memcpy(m_port_data.data.data(), m_xx_data, m_size * sizeof(double));
		  }
			else
				throw std::out_of_range("Variable data size does not match port data size.");
		}

	private:
    T* m_port;
    std::string m_full_name;

    XVMatrix& m_matrix;

    flat_matrix_t m_port_data; // from/to ports
    XXDouble* m_xx_data; // 20sim internal matrix
    std::size_t m_size;

    Adapter20Sim& operator=(const Adapter20Sim& ass); // non-accessible
	};
}
