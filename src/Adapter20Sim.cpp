#include "Adapter20Sim.h"

#include <boost/algorithm/string.hpp>

namespace common20sim {

	std::string replaceIllegalCharacter(std::string str)
	{
		str = replaceIllegalCharacter(str, "\\", "/");
		str = replaceIllegalCharacter(str, "[", "__");
		str = replaceIllegalCharacter(str, "]", "__");
		str = replaceIllegalCharacter(str, ".", "_");
		str = replaceIllegalCharacter(str, ",", "_");
		return str;
	}

	std::string replaceIllegalCharacter(std::string str, std::string pattern, std::string replacement)
	{
	  size_t found;
	  found=str.find_first_of(pattern);
    while (found != std::string::npos)
    {
      str.replace(found, 1, replacement);
      found=str.find_first_of(pattern, found+replacement.length());
    }
    return str;
	}

}

