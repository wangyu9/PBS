#include <string>

#include "extension_from_name.h"

const std::string extension_from_name(const std::string name_string)
{
	size_t last_dot = name_string.rfind('.');
	if (last_dot == std::string::npos)
	{
		// No file type determined
		printf("Error: No file extension found in %s\n", name_string);
		return std::string("");
	}
	std::string extension = name_string.substr(last_dot + 1);
	return extension;
}