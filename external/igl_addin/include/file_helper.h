#ifndef FILE_HELPER_H
#define FILE_HELPER_H

inline bool extract_extension(std::string file_name_string, std::string& extension)
{
	size_t last_dot = file_name_string.rfind('.');
	if (last_dot == std::string::npos)
	{
		// No file type determined
		printf("Error: No file extension found in %s\n", file_name_string);
		return false;
	}
	extension = file_name_string.substr(last_dot + 1);
	return true;
}

#endif /*FILE_HELPER_H*/