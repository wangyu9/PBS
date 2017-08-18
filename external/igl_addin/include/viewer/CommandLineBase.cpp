#include "CommandLineBase.h"

#include <viewer/path_anttweak.h>

#include "PluginManager.h"


#include "FileDialog.h"

CommandLineBase::CommandLineBase()
{
	PluginManager().register_plugin(this);
	bar = NULL;
}

bool CommandLineBase::Serialize(tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element)
{
	return igl::save_ReAntTweakBar(bar, doc);
}

bool CommandLineBase::Deserialize(tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element)
{
	return igl::load_ReAntTweakBar(bar, doc);
}

void CommandLineBase::init(Preview3D *preview)
{
	PreviewPlugin::init(preview);

	if (bar==NULL)
	{
		// Create a tweak bar
		bar = new igl::ReTwBar;
		bar->TwNewBar("Deformer&Picking");
		TwDefine(" Deformer&Picking size='250 250' color='76 76 127' position='500 16' label='Deformer Picking' "); // change default tweak bar size and color
		bar->TwAddButton("Load Command Lines", DIALOG_OF(load_command_line), this, "group='Fast Loading' key=c");
	}

}

inline bool path_from_dir(char* fname, std::string& path)
{// Note: the last \ is not included in the string.
	fname[0] = 0;
	get_open_file_path(fname);

	if (fname[0] == 0)
	{
		printf("No Path is give.\n");
		return false;
	}

	std::string file_name_string = std::string(fname);
	size_t last_slash = file_name_string.rfind('\\');
	if (last_slash == std::string::npos)
	{
		// No file type determined
		printf("Error: No file path found in %s\n", fname);
		return false;
	}
	path = file_name_string.substr(0, last_slash);

	return true;
}

void CommandLineBase::load_command_line()
{
	char fname[2048];
	std::string folder_path;
	std::string file_name_string;

	if (!path_from_dir(fname, folder_path))
		return;

	read_command_file(fname);
}


#include "Viewer.h"
#include "ViewerPlugin.h"

#include <plugins/DeformSkinning.h>
#include "plugins/DeformPhys.h"
#include "plugins/DeformDirect.h"
#include "plugins/HandlePlugin.h"
#include "plugins/PickingPlugin.h"

// TODO later, finish this general function parse.

typedef enum { FunctionArgument_INT, FunctionArgument_STRING, FunctionArgument_FLOAT, FunctionArgument_DOUBLE, FunctionArgument_UNDEFINED } FunctionArgumentType;

#include <functional>
class FunctionArgumment {
public:
	FunctionArgumentType type;
	int v_int;
	std::string v_string;
	float v_float;
	double v_double;
	FunctionArgumment() :
		type(FunctionArgument_UNDEFINED),
		v_int(0),
		v_string("Undefined"),
		v_float(0.),
		v_double(0.)
	{}
};

class FunctionUnit {
public:
	std::string name;
	std::vector<FunctionArgumentType> argutype;
	bool(*pointer)(void*);
	FunctionUnit(bool(*p)(void*), const std::string n) : name(n), pointer(p) {}
	bool call_function(const std::vector<FunctionArgumment>& argu)
	{
		if (this->argutype.size() != argu.size())
		{
			printf("Error: Incorrect number of arguments for function %s() !\n", name);
			return false;
		}

		switch (argu.size())
		{
		case 0:
			//return (*pointer)(void);
			return function_void();
			break;
		case 1:
		{
			if (argu[0].type != argutype[0])
			{
				printf("Error in FunctionUnit, type does not match!\n");
				return false;
			}

			switch (argu[0].type)
			{
			case FunctionArgument_INT:
				return function_int(argu[0].v_int);
			case FunctionArgument_STRING:
				return function_string(argu[0].v_string);
			case FunctionArgument_FLOAT:
				return function_float(argu[0].v_float);
			case FunctionArgument_DOUBLE:
				return function_double(argu[0].v_double);
			default:
				break;
			}
		}

		}
		return true;
	}

	std::function<bool(void)> function_void;
	std::function<bool(int)> function_int;
	std::function<bool(std::string)> function_string;
	std::function<bool(float)> function_float;
	std::function<bool(double)> function_double;

};

class FunctionList {
	std::vector<FunctionUnit> functions;
	void register_new(bool(*pointer)(void*), const std::string name)
	{
		functions.push_back(FunctionUnit(pointer, name));
	}
	bool call_function(const std::string& name, const std::vector<FunctionArgumment>& argu)
	{
		for (int i = 0; i < functions.size(); i++)
		{
			if (name == functions[i].name)
			{
				return functions[i].call_function(argu);
			}
		}
	}
};

bool function_caller_string(std::vector<std::string> name, bool(*f)(std::string))
{
	const int num_of_argu = 1;
	if (name.size() < num_of_argu + 1)
	{
		printf("Error: Not enough argument for %s.\n", name[0]);
		return false;
	}
	return (*f)(name[1]);
}

bool function_caller_void(std::vector<std::string> name, bool(*f)(void))
{
	const int num_of_argu = 0;
	if (name.size() >= num_of_argu + 1)
	{
		printf("Warning: Ignore unnecessary arguments for %s.\n", name[0]);
	}
	return (*f)();
}

void print_out_argv(const std::vector<std::string> &cl)
{
	for (size_t i = 0; i < cl.size(); i++)
	{
		printf("  %s", cl[i]);
	}
}


//#ifdef __APPLE__
//#   include <GLUT/glut.h>
//#else
//#   include <GL/glut.h>
//#endif
bool CommandLine(Preview3D& plugin, std::vector<std::string> cl)
{
	if (cl.size() < 1)
	{
		printf("Error: No Command Line for Viewer.\n");
		return false;
	}
	else
	{
		printf("Preview3D");
		print_out_argv(cl);
		printf("\n");
	}

	if (cl[0] == std::string("load_mesh_from_file"))
	{
		if (cl.size() < 2)
		{
			printf("Error: No mesh file name for load_mesh_from_file.\n");
			return false;
		}
		return plugin.load_mesh_from_file(cl[1].c_str());
	}
	else if (cl[0] == std::string("load_scene"))
	{
		if (cl.size() < 2)
		{
			printf("Error: No scene file name for load_scene.\n");
			return false;
		}
		return plugin.loadScene(cl[1].c_str());
	}
	else if (cl[0] == std::string("load_texture"))
	{
		if (cl.size() < 2)
		{
			printf("Error: No scene file name for load_scene.\n");
			return false;
		}
		plugin.GetMainMesh().SetTexFilename(cl[1].c_str());
		return true;
	}
	else if (cl[0] == std::string("load_texture_coordinates_from_file"))
	{
		if (cl.size() < 2)
		{
			printf("Error: No file name for load_texture_coordinates_from_file.\n");
			return false;
		}
		plugin.GetMainMesh().load_texture_coordinates_from_file(cl[1].c_str());
		return true;
	}
	else if (cl[0] == std::string("load_camera"))
	{
		if (cl.size() < 2)
		{
			printf("Error: No scene file name for load_camera.\n");
			return false;
		}
		plugin.LoadCamera(cl[1].c_str());
		return true;
	}
	else if (cl[0] == std::string("set_color_preset"))
	{
		if (cl.size() < 2)
		{
			printf("Error: No scene file name for set_color_preset.\n");
			return false;
		}
		plugin.set_color_preset((ColorPresetID)std::stoi(cl[1]));
		return true;
	}
	//else if (cl[0] == std::string("draw"))
	//{
	//	plugin.draw(glutGet(GLUT_ELAPSED_TIME));
	//	return true;
	//}
	else
	{
		printf("Error: Unknown Command Line Type for Viewer.\n");
		return false;
	}
}





bool CommandLineBase::command_line_parser(char* line)
{
	if (line[0] == '#')
	{
		return true;// ignore commented line
	}


	char tempBuf[1024];
	char * tok;

	std::string c;
	std::vector<std::string> cl;

	// http://stackoverflow.com/questions/19280402/read-unknown-number-of-integers-in-n-lines
	//std::getline(mesh_file, groupline);
	int num = 0;
	tok = strtok(line, " \t\n,;");
	while (tok)
	{
		strcpy(tempBuf, tok);
		//number[num++] = atoi(tempBuf);
		if (num == 0)
		{
			c = tempBuf;
		}
		else
		{
			cl.push_back(tempBuf);
		}
		tok = strtok(NULL, " \t\n,;");
		num++;
	}

	if (c == std::string("Viewer"))
	{
		return CommandLine(*m_preview, cl);
	}
	else
	{
		for (unsigned int i = 0; i < PluginManager().plugin_list_.size(); ++i)
			if (PluginManager().plugin_list_[i]->commandLine(c,cl))
				return true;
	}


	printf("Warning: unknown plugin type.\n");
	return false;

}

bool CommandLineBase::read_command_file(const std::string fname)
{
	FILE * command_file = fopen(fname.c_str(), "r");

	if (NULL == command_file)
	{
		fprintf(stderr, "IOError: %s could not be opened...", fname.c_str());
		return false;
	}

#ifndef LINE_MAX
#  define LINE_MAX 2048
#endif

	char line[LINE_MAX];

	int i = 0;
	for (i = 0; i < LINE_MAX; i++)
	{
		if (NULL == fgets(line, LINE_MAX, command_file))// end of file
			return true;

		if (!command_line_parser(line))
			return false;
	}

	if (i == LINE_MAX)
	{
		printf("Warning: lines later than %d are ignored!\n", LINE_MAX);
	}
	return true;
}