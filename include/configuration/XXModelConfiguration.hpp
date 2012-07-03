#pragma once

#include <vector>
#include <string>
#include <stdexcept>

#include "configuration/XVMatrix.hpp"
#include "common/xxmodel.h"

namespace common20sim
{
  class XXModelConfiguration
  {
    public:
      XXModelConfiguration(Submodel20sim* model);

      /**
       * @brief Reads the XML property files.
       */
      void load(std::string file) throw(std::invalid_argument);

      std::vector<XVMatrix>& getConfiguration();

      ~XXModelConfiguration();

    protected:
      XXType parseXXType(std::string type);
      std::vector<double> parseValues( std::string values_str, unsigned int rows, unsigned int columns);
      std::vector<double> parseRowValues(std::string row, unsigned int columns);
      double* parseContainer(std::string container);

    private:
      std::vector<XVMatrix> m_configuration;
      Submodel20sim* m_model;
      XXModelConfiguration();
  };
}
