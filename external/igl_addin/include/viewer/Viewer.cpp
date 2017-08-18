#include "Viewer.h"
#include "PluginManager.h"

#include <vector>
using namespace std;

#ifdef __APPLE__
#   include <OpenGL/gl.h>
#   include <OpenGL/glu.h>
#   include <GLUT/glut.h>
#else
#   ifdef _WIN32
#       include <windows.h>
#       include <GL/glew.h>
#       include <GL/glut.h>
#   endif
#   include <GL/gl.h>
#   include <GL/glu.h>
#endif

void Preview3D::Draw(int current_time)
{
	std::vector<Camera*> pCameras;
	pCameras.push_back(&camera);
	//pCameras.push_back(&camera2);

	if (m_pause)
	{
		return;
	}

	ViewerBase::clear_draw();


	for (unsigned int i = 0; i <PluginManager().plugin_list_.size(); ++i)
		PluginManager().plugin_list_[i]->preDraw(current_time);

	ViewerBase::pre_draw(current_time);

	ViewerBase::main_draw();



	// start of postDraw

	for (size_t i = 0; i < pCameras.size(); i++)
	{
		glPushMatrix();
		pCameras[i]->SetGL();
		{
			if (false) // wangyu this is a temp hack
			{
				for (unsigned int i = 0; i < PluginManager().plugin_list_.size(); ++i)
					PluginManager().plugin_list_[i]->postDraw(current_time);
			}
			else
			{
				for (int i = PluginManager().plugin_list_.size() - 1; i >= 0; i--)// important for not using unsigned int
					PluginManager().plugin_list_[i]->postDraw(current_time);
			}
		}
		glPopMatrix();
	}

	// end of postDraw

	TwDraw();

	// redraw
	if (enable_autoRefresh)
		glutPostRedisplay();

	//added by wangyu recording
	if (m_recording_screen)
	{
		char anim_filename[256];
		sprintf_s(anim_filename, 256, "frame%04d.png", frame_index++);
		ViewerBase::grab_screen(anim_filename);
	}

	if (m_record_one_frame)
	{
		time_t t;
		time(&t);
		char anim_filename[256];
		sprintf_s(anim_filename, 256, "time%d.png", t);
		ViewerBase::grab_screen(anim_filename);

		m_record_one_frame = false;
	}
}

void TW_CALL Preview3D::load_mesh_and_init_CB(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_open_file_path(fname);

	if (fname[0] == 0)
		return;

	bool bLoad = static_cast<Preview3D *>(clientData)->load_mesh_from_file(fname);

	//if (bLoad)//wangyu
	//{
	//	static_cast<CLASS_NAME *>(clientData)->copy_working_folder(fname);
	//}
}

Preview3D::Preview3D(int start_time):
	ViewerBase(start_time)
{
	if (bar)
	{
		bar->TwAddButton("Load Mesh and Init P", load_mesh_and_init_CB, this,
			" group='Load & Save'"
			" label='Open mesh' key=o help='Load a mesh.'");

		bar->TwAddButton("Load Scene", LoadSceneCB, this,
			"group='Load & Save'"
			" label='Load Scene' help='Save a scene.'");
		bar->TwAddButton("Save Scene", SaveSceneCB, this,
			"group='Load & Save'"
			" label='Save Scene' help='Load a scene.'");

		bar->TwAddVarRW("Recording Screen", TW_TYPE_BOOLCPP, &m_recording_screen,
			" label='Turn on/off recording of screen.' group='Recording' key=r");
		bar->TwAddButton("Record One Frame", record_frame_CB, this,
			" group='Recording'"
			" label='Record One Frame.' ");

		// added by wagnyu
		bar->TwAddVarRW("Pause", TW_TYPE_BOOLCPP, &m_pause, " label='Turn on/off pause.' key=p ");

		bar->TwAddButton("Hide All TW Windows", HideAllTWWindowsCB, this, " group='Global' key=h");
	}

	// initialize scroll position to 0
	scroll_position = 0.0f;
	m_pause = false;//added by wangyu
	frame_index = 0;//added by wangyu
	m_recording_screen = false;//added by wangyu
	m_record_one_frame = false;//added by wangyu

	for (unsigned int i = 0; i<PluginManager().plugin_list_.size(); ++i)
		PluginManager().plugin_list_[i]->init(this);
}

void Preview3D::InitPlugins()
{
	for (unsigned int i = 0; i < PluginManager().plugin_list_.size(); ++i)
		PluginManager().plugin_list_[i]->init(this);
}

void Preview3D::Restart()
{
	for (unsigned int i = 0; i < PluginManager().plugin_list_.size(); ++i)
		PluginManager().plugin_list_[i]->init(this);
}

bool Preview3D::load_mesh_from_file(const char* mesh_file_name)
{
	bool r = ViewerBase::load_mesh_from_file(mesh_file_name);
	InitPlugins();
	return r;
}

bool Preview3D::saveScene()
{
	char fname[FILE_DIALOG_MAX_BUFFER];
	fname[0] = 0;
	get_save_file_path(fname);

	if (fname[0] == 0)
		return false;

	// open new XMLDocument
	tinyxml2::XMLDocument* doc = new tinyxml2::XMLDocument();

	// serialize previously added objects
	serializer->SaveToXMLDoc(doc);

	// serialize AntTweakBar
	igl::save_ReAntTweakBar(bar, doc);

	// serialize objects of all plugins
	for (unsigned int i = 0; i < PluginManager().plugin_list_.size(); ++i)
		PluginManager().plugin_list_[i]->Serialize(doc, NULL);

	// Save doc to xml file
	tinyxml2::XMLError error = doc->SaveFile(fname);
	if (error != tinyxml2::XML_NO_ERROR)
	{
		doc->PrintError();
		return false;
	}

	delete doc;

	return true;
}

bool Preview3D::loadScene(const char* fname)
{

	// load XMLDocument
	tinyxml2::XMLDocument* doc = new tinyxml2::XMLDocument();

	tinyxml2::XMLError error = doc->LoadFile(fname);
	if (error != tinyxml2::XML_NO_ERROR)
	{
		doc->PrintError();
		return false;
	}

	// deserialize previously added objects
	serializer->LoadFromXMLDoc(doc);

	// serialize AntTweakBar
	igl::load_ReAntTweakBar(bar, doc);

	// deserialize objects of all plugins
	for (unsigned int i = 0; i < PluginManager().plugin_list_.size(); ++i)
		PluginManager().plugin_list_[i]->Deserialize(doc, NULL);

	delete doc;

	return true;
}

void TW_CALL Preview3D::SaveSceneCB(void *clientData)
{
	static_cast<Preview3D *>(clientData)->saveScene();
}

void TW_CALL Preview3D::LoadSceneCB(void *clientData)
{
	char fname[FILE_DIALOG_MAX_BUFFER];
	fname[0] = 0;
	get_open_file_path(fname);

	if (fname[0] == 0)
		return;// false;

	static_cast<Preview3D *>(clientData)->loadScene(fname);
}

#define MOUSE_KEY_EVENT_ALL_RESPONSE false

bool Preview3D::key_down(unsigned char key, int modifiers, int mouse_x, int mouse_y)
{
	for (unsigned int i = 0; i <PluginManager().plugin_list_.size(); ++i)
		if (PluginManager().plugin_list_[i]->keyDownEvent(key, modifiers, mouse_x, mouse_y))
			return true;
	//wangyu changed this
	//for (unsigned int i = 0; i <PluginManager().plugin_list_.size(); ++i)
	//	PluginManager().plugin_list_[i]->keyDownEvent(key, modifiers, mouse_x, mouse_y);

	return ViewerBase::key_down(key, modifiers, mouse_x, mouse_y);
}

bool Preview3D::key_up(unsigned char key, int modifiers, int mouse_x, int mouse_y)
{
	return ViewerBase::key_up(key, modifiers, mouse_x, mouse_y);
}

bool Preview3D::mouse_down(int mouse_x,
	int mouse_y,
	int button,
	int modifiers)
{
	// First pass to AntTweakBar
	if (TwEventMouseButtonGLUT(button, 0, mouse_x, mouse_y))
		return true;

	// Then pass to all plugins
	switch (button)
	{
	case GLUT_LEFT_BUTTON:
	{
		if (MOUSE_KEY_EVENT_ALL_RESPONSE)
		{
			//This is changed by wangyu to be different from the original viewer, all response function will be called.
			bool any_response = false;
			for (unsigned int i = 0; i < PluginManager().plugin_list_.size(); ++i)
				any_response |= PluginManager().plugin_list_[i]->mouseDownEvent(mouse_x, mouse_y, button, modifiers);
			if (any_response)
				return true;
		}
		else
		{
			for (unsigned int i = 0; i < PluginManager().plugin_list_.size(); ++i)
				if (PluginManager().plugin_list_[i]->mouseDownEvent(mouse_x, mouse_y, button, modifiers))
					return true;
		}

		if (modifiers & SHIFT)
		{
		}

		else if (modifiers == NO_KEY)
		{
		}

		break;
	}

	case GLUT_RIGHT_BUTTON:
	{
		if (MOUSE_KEY_EVENT_ALL_RESPONSE)
		{
			bool any_true = false;
			for (unsigned int i = 0; i < PluginManager().plugin_list_.size(); ++i)
				any_true |= PluginManager().plugin_list_[i]->mouseDownEvent(mouse_x, mouse_y, button, modifiers);
			if (any_true) return true;
		}
		else
		{
			for (unsigned int i = 0; i <PluginManager().plugin_list_.size(); ++i)
				if (PluginManager().plugin_list_[i]->mouseDownEvent(mouse_x, mouse_y, button, modifiers))
					return true;
		}

		if (modifiers & SHIFT)
		{
		}
		else if (modifiers == NO_KEY)
		{
		}
		break;
	}

	case 3: // MouseWheel up
	{
	}
	break;

	case 4: // MouseWheel down
	{
	}
	break;

	}

	return ViewerBase::mouse_down(mouse_x, mouse_y, button, modifiers);
}

bool Preview3D::mouse_up(int mouse_x,
	int mouse_y,
	int button,
	int modifiers)
{

	if (TwEventMouseButtonGLUT(button, 1, mouse_x, mouse_y))
		return true;

	if (MOUSE_KEY_EVENT_ALL_RESPONSE)
	{
		bool any_response = false;
		for (unsigned int i = 0; i < PluginManager().plugin_list_.size(); ++i)
			any_response |= PluginManager().plugin_list_[i]->mouseUpEvent(mouse_x, mouse_y, button, modifiers);
		if (any_response)
			return true;
	}
	else
	{
		for (unsigned int i = 0; i < PluginManager().plugin_list_.size(); ++i)
			if (PluginManager().plugin_list_[i]->mouseUpEvent(mouse_x, mouse_y, button, modifiers))
				return true;
	}

	if (ViewerBase::mouse_up(mouse_x, mouse_y, button, modifiers))
		return true;

	return false;
}

bool Preview3D::mouse_move(int mouse_x, int mouse_y)
{
	if (TwEventMouseMotionGLUT(mouse_x, mouse_y))
		return true;

	if (MOUSE_KEY_EVENT_ALL_RESPONSE)
	{
		bool any_response = false;
		for (unsigned int i = 0; i < PluginManager().plugin_list_.size(); ++i)
			any_response |= PluginManager().plugin_list_[i]->mouseMoveEvent(mouse_x, mouse_y);
		if (any_response)
			return true;
	}
	else
	{
		for (unsigned int i = 0; i < PluginManager().plugin_list_.size(); ++i)
			if (PluginManager().plugin_list_[i]->mouseMoveEvent(mouse_x, mouse_y))
				return true;
	}

	return ViewerBase::mouse_move(mouse_x, mouse_y);
}

bool Preview3D::mouse_scroll(int mouse_x, int mouse_y, float delta_y)
{
	scroll_position += delta_y;
	bool in_bar = TwMouseMotion(mouse_x, mouse_y);

	if (TwMouseWheel(scroll_position) || in_bar)
		return true;

	for (unsigned int i = 0; i < PluginManager().plugin_list_.size(); ++i)
		if (PluginManager().plugin_list_[i]->mouseScrollEvent(mouse_x, mouse_y, delta_y))
			return true;

	return ViewerBase::mouse_scroll(mouse_x, mouse_y, delta_y);
}

void TW_CALL Preview3D::record_frame_CB(void *clientData)
{
	//time_t t;
	//time(&t);
	//char anim_filename[256];
	//sprintf_s(anim_filename, 256, "time%d.png", t);
	//static_cast<ViewerBase *>(clientData)->grab_screen(anim_filename);
	static_cast<Preview3D *>(clientData)->m_record_one_frame = true;
	return;
}

void Preview3D::hide_all_tw_windows()
{
	if (old_positions.size() == 0)
	{
		int pos[2];
		twGetWindowPos(bar, pos);
		old_positions.push_back(std::make_pair(pos[0], pos[1]));
		twSetWindowPos(bar, pos[0], 2000);

		for (unsigned int i = 0; i < PluginManager().plugin_list_.size(); ++i)
		{
			twGetWindowPos(PluginManager().plugin_list_[i]->bar, pos);
			old_positions.push_back(std::make_pair(pos[0], pos[1]));
			twSetWindowPos(PluginManager().plugin_list_[i]->bar, pos[0], 2000);
		}

		TwDefine(" TW_HELP visible=false ");  // help bar is hidden
	}
	else
	{
		assert(PluginManager().plugin_list_.size() + 1 == old_positions.size());

		twSetWindowPos(bar, old_positions[0].first, old_positions[0].second);

		for (unsigned int i = 0; i < PluginManager().plugin_list_.size(); ++i)
		{
			twSetWindowPos(PluginManager().plugin_list_[i]->bar, old_positions[i + 1].first, old_positions[i + 1].second);
		}
		old_positions.clear();

		TwDefine(" TW_HELP visible=true ");  // help bar is hidden
	}
}

void Preview3D::grab_screen()
{
	ViewerBase::grab_screen(frame_index);
}
