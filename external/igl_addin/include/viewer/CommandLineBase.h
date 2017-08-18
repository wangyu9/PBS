#pragma once

#include "ViewerPlugin.h"

#include <anttweak_helper.h>

class CommandLineBase : public PreviewPlugin
{
public:

	CommandLineBase();

	// implement Serializable interface
	bool Serialize(tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element);
	bool Deserialize(tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element);

	// initialization (runs every time a mesh is loaded or cleared)
	void init(Preview3D *preview);

	// keyboard callback
	bool keyDownEvent(unsigned char key, int modifiers, int mouse_x, int mouse_y) { return false; }

	//mouse callback
	bool mouseDownEvent(int mouse_x, int mouse_y, int button, int modifiers) { return false; }
	bool mouseUpEvent(int mouse_x, int mouse_y, int button, int modifiers) { return false; }
	bool mouseMoveEvent(int mouse_x, int mouse_y) { return false; }
	bool mouseScrollEvent(int mouse_x, int mouse_y, float delta) { return false; }

	//stuff that is drawn by the plugin before the previewer has displayed the mesh
	//first draw 3d, then 2d
	void preDraw(int currentTime) {}
	//stuff that is drawn by the plugin after the previewer has displayed the mesh
	//first draw 3d, then 2d
	void postDraw(int currentTime) {}

	void load_command_line();

	bool read_command_file(const std::string fname);

private:

	bool command_line_parser(char* line);

	static void TW_CALL DIALOG_OF(load_command_line)(void *clientData)
	{
		static_cast<CommandLineBase *> (clientData)->load_command_line();
	}

};

void print_out_argv(const std::vector<std::string> &cl);