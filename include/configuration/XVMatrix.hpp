#pragma once

#include "xxmatrix.h"
#include <vector>
#include <string>

namespace common20sim
{
  enum XXType {UNKNOWN, INPUT, OUTPUT, PARAMETER, STATE, VARIABLE, INTERNAL};

  typedef struct _XVMatrix
  {
      std::string name;
      std::string description;
      XXType type;

      XXMatrix storage;
      std::vector<double> values;
  } XVMatrix;
}
