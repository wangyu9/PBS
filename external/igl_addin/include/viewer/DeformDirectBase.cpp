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

#include <unsupported/Eigen/OpenGLSupport>

#include "DeformDirectBase.h"

#include "FileDialog.h"
#include "PluginManager.h"

#include "./plugins/PickingPlugin.h"


#include "path_anttweak.h"

#include <draw_handle_struct.h>

#ifdef USING_IGL_HEADER_ONLY_MODE
#define IGL_HEADER_ONLY 
#endif


#include <igl/readDMAT.h>
#include "math_helper.h"

DeformDirectBase::DeformDirectBase() : 
keyFramer(frame_timer)
//keyFramer(world_timer)//decides what timer to use here
{

	control_radius = 91.;
	control_scaling = 1.0;
	

	draw_transwidget_center = true;
	draw_only_selected_transwidget = true;

}

DeformDirectBase::~DeformDirectBase()
{

}

void DeformDirectBase::initFromHandle(const HandleStructure& hs)
{
	controlStruct.initTest(hs);
}

void DeformDirectBase::initControlStructActive(const HandleStructure& hs)
{
	controlStruct.initActive(hs);
}

void DeformDirectBase::initControlStruct(const HandleStructure& hs, const Eigen::VectorXi& cs)
{
	controlStruct.init(hs,cs);
}

void DeformDirectBase::addControlStruct(const HandleStructure& hs, const Eigen::VectorXi& cs)
{
	controlStruct.add(hs, cs);
}

void DeformDirectBase::set_control_radius(const double r)
{
	control_radius = r;
	controlStruct.set_units_radius(r);

}

double DeformDirectBase::get_control_radius() const
{
	return control_radius;
}

void DeformDirectBase::set_control_scaling(const double s)
{
	control_scaling = s;

	std::vector<int> selected;
	std::vector<Eigen::MatrixXd> trans;

	controlStruct.set_units_scaling(s, selected, trans);
	WriteExternal(selected, trans);
}

double DeformDirectBase::get_control_scaling() const
{
	return control_scaling;
}

void DeformDirectBase::update_key_frame()
{
	if (keyFramer.Size()==0)
	{
		return;
	}
	ControlStructState blendframe = keyFramer.Current();
	
	std::vector<int> selected;
	std::vector<Eigen::MatrixXd> trans;
	setControlStructState(blendframe.csState,controlStruct,selected,trans,false);
	WriteExternal(selected, trans);
}

bool DeformDirectBase::load_key_frame_config(const char * config_file_name)
{
	return keyFramer.Load_Config(config_file_name);
}

bool DeformDirectBase::start_key_frame()
{
	return keyFramer.Start();
}

void DeformDirectBase::set_draw_widget_center(const bool t)
{
	draw_transwidget_center = t;
	for (int i = 0; i < controlStruct.controlUnits.size(); i++)
	{
		controlStruct.controlUnits[i].rotateWidget.draw_center = draw_transwidget_center;
	}
}

bool DeformDirectBase::get_draw_widget_center() const
{
	return draw_transwidget_center;
}

void DeformDirectBase::set_draw_only_selected_widget(const bool t)
{
	draw_only_selected_transwidget = t;
	for (int i = 0; i < controlStruct.controlUnits.size(); i++)
	{
		controlStruct.controlUnits[i].rotateWidget.draw_only_when_selected = draw_only_selected_transwidget;
	}
}

bool DeformDirectBase::get_draw_only_selected_widget() const
{
	return draw_only_selected_transwidget;
}

void DeformDirectBase::set_turn_on_key_framing(const bool t)
{
	keyFramer.SetOnOff(t);
}

bool DeformDirectBase::get_turn_on_key_framing() const
{
	return keyFramer.GetOnOff();
}

void DeformDirectBase::set_key_frame_time(const double t)
{
	keyFramer.SetTime(t);
	update_key_frame();
}

double DeformDirectBase::get_key_frame_time() const
{
	return keyFramer.GetTime();
}

void DeformDirectBase::set_draw_skeleton(const bool t)
{
	controlStruct.draw_skeleton = t;
}

bool DeformDirectBase::get_draw_skeleton() const
{
	return controlStruct.draw_skeleton;
}

void DeformDirectBase::save_control_struct_state(const char* fname)
{
	saveControlStructState(fname, controlStruct);
}

void DeformDirectBase::load_control_struct_state(const char* fname)
{
	std::vector<int> selected;
	std::vector<Eigen::MatrixXd> trans;
	loadControlStructState(fname, controlStruct, selected, trans);
	WriteExternal(selected, trans);
}









//void DeformDirectBaseUI::TranslateControlStructRoots(float *t)
//{
//	std::vector<int> selected;
//	std::vector<Eigen::MatrixXd> trans;
//	setControlStructTranslate( t, controlStruct, selected, trans);
//
//	WriteExternal(selected, trans);
//}

bool DeformDirectBaseUI::load_rotation_center_from_file(const char *fname)
{
	Eigen::MatrixXd RC;
	if (!igl::readDMAT(fname, RC))
	{
		printf("Error: fails to load the rotation center file!\n");
		return false;
	}
	if (RC.rows() < 1 || RC.cols() != 4)
	{
		printf("Error: the dimension of rotation center file (%d, %d) is incorrect!\n", RC.rows(), RC.cols());
		return false;
	}

}

DeformDirectBaseUI::DeformDirectBaseUI():DeformDirectBase()
{
	//check in with the manager
	PluginManager().register_plugin(this);

	bar = NULL;

	enabled_display = enabled_mouse = true;
}

DeformDirectBaseUI::~DeformDirectBaseUI()
{
	// Inverse order to call deconstructor function

	DeformDirectBase::~DeformDirectBase();
}

// initialization (runs every time a mesh is loaded or cleared)
void DeformDirectBaseUI::init(Preview3D *preview)
{
	PreviewPlugin::init(preview);

	// init menu bar
	if(bar == NULL)
	{
		// Create a tweak bar
		bar = new igl::ReTwBar;
		bar->TwNewBar("Direct");
		TwDefine(" Direct size='250 250' color='76 76 127' position='235 300' label='Direct Deformation' "); // change default tweak bar size and color

		bar->TwAddButton("Load State", dialog_load_control_struct_state, this, " group='Load & Save'");
		bar->TwAddButton("Save State", dialog_save_control_struct_state, this, " group='Load & Save'");

		
		bar->TwAddVarRW("Enable Mouse", TW_TYPE_BOOLCPP, &enabled_mouse, " group='Enable'");

		bar->TwAddVarRW("Display Widget", TW_TYPE_BOOLCPP, &enabled_display, " group='UI'");
		bar->TwAddVarCB("Radius of Rotate Widget", TW_TYPE_DOUBLE, SetControlRadiusCB, GetControlRadiusCB, this, " group='UI'");
		bar->TwAddVarCB("Scaling of Rotate Widget", TW_TYPE_DOUBLE, SetControlScalingCB, GetControlScalingCB, this, " group='UI' step=0.1");
		bar->TwAddVarCB("Draw Center of Widget", TW_TYPE_BOOLCPP, SetDrawCenterCB, GetDrawCenterCB, this, " group='UI'");
		bar->TwAddVarCB("Draw Only Selected", TW_TYPE_BOOLCPP, SetDrawOnlySelectedCB, GetDrawOnlySelectedCB, this, " group='UI'");
		bar->TwAddVarCB("Draw Skeleton", TW_TYPE_BOOLCPP, SetDrawSkeletonCB, GetDrawSkeletonCB, this, " group='UI'");

		// Key frame
		bar->TwAddButton("Load Keyframe Config", dialog_load_keyframing_config, this, " group='Keyframe' key=k");
		bar->TwAddButton("Restart Keyframe", dialog_start_keyframing, this, " group='Keyframe' key='s'");
		bar->TwAddVarRW("Slow Down Factor", TW_TYPE_DOUBLE, &keyFramer.keyframe_slow_down_factor, " group='Keyframe'");
		bar->TwAddVarCB("Time", TW_TYPE_DOUBLE, SetKeyFrameTimeCB, GetKeyFrameTimeCB, this, " group='Keyframe'");
		bar->TwAddVarCB("Turn on Keyframe", TW_TYPE_BOOLCPP, SetTurnOnKeyFramingCB, GetTurnOnKeyFramingCB, this, " group='Keyframe'");

#define TransitionTypeCount 6
		TwEnumVal transitionEV[TransitionTypeCount] = {
			{ LINEAR_TRANSITION, "LINEAR_TRANSITION" },
			{ EASE_TRANSITION, "EASE_TRANSITION" },
			{ ABRUPT_TRANSITION, "ABRUPT_TRANSITION" },
			{ ABRUPT_IN_TRANSITION, "ABRUPT_IN_TRANSITION" },
			{ ABRUPT_OUT_TRANSITION, "ABRUPT_OUT_TRANSITION" },
			{ CUBIC_SPLINE_TRANSITION, "CUBIC_SPLINE_TRANSITION" }
		};

		TwType transitionT = TwDefineEnum("Transition Type", transitionEV, TransitionTypeCount);
		bar->TwAddVarCB("Transition Type", transitionT, SetTransitionTypeCB, GetTransitionTypeCB, this, "group='Keyframe'");

	}

	//controlUnits.push_back(ControlUnit());
	//DeformDirectBase::init(preview);
}

bool DeformDirectBaseUI::Serialize(tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element)
{
	return igl::save_ReAntTweakBar(bar,doc);
}

bool DeformDirectBaseUI::Deserialize(tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element)
{
	return igl::load_ReAntTweakBar(bar,doc);
}

bool DeformDirectBaseUI::keyDownEvent(unsigned char key, int modifiers, int mouse_x, int mouse_y)
{
	//if (!enabled) return false;

	//float t[3];
	//t[0] = t[1] = t[2] = 0.;
	//float d = m_preview->avg_edge_length;
	//switch (key)
	//{

	//case 'j':
	//	t[0] = d;
	//	translate_cs_roots(t);
	//	return true;
	//case 'l':
	//	t[0] = -d;
	//	translate_cs_roots(t);
	//	return true;
	//case 'i':
	//	t[1] = d;
	//	translate_cs_roots(t);
	//	return true;
	//case 'k':
	//	t[1] = -d;
	//	translate_cs_roots(t);
	//	return true;
	//}
	return false;
}

//mouse callback
bool DeformDirectBaseUI::mouseDownEvent(int mouse_x, int mouse_y, int button, int modifiers)
{
	if (!enabled_mouse) return false;

	int height = m_preview->height;

	push_gl_settings();

	std::vector<int> selected;
	std::vector<Eigen::MatrixXd> trans;

	bool r = controlStruct.mouseDown(mouse_x,mouse_y,height,selected,trans);
	if (r)
		WriteExternal(selected, trans);

	pop_gl_settings();

	return r;
}

bool DeformDirectBaseUI::mouseUpEvent(int mouse_x, int mouse_y, int button, int modifiers)
{
	if (!enabled_mouse) return false;

	int height = m_preview->height;

	push_gl_settings();

	std::vector<int> selected;
	std::vector<Eigen::MatrixXd> trans;

	bool r = controlStruct.mouseUp(mouse_x, mouse_y, height, selected,trans);
	if (r)
		WriteExternal(selected, trans);

	pop_gl_settings();

	return r;
}

bool DeformDirectBaseUI::mouseMoveEvent(int mouse_x, int mouse_y)
{
	if (!enabled_mouse) return false;

	int height = m_preview->height;

	push_gl_settings();

	std::vector<int> selected;
	std::vector<Eigen::MatrixXd> trans;

	bool r = controlStruct.mouseMove(mouse_x, mouse_y,height,selected,trans);
	if (r)
		WriteExternal(selected, trans);

	pop_gl_settings();

	return r;
}

bool DeformDirectBaseUI::mouseScrollEvent(int mouse_x, int mouse_y, float delta)
{
	if (!enabled_mouse) return false;

	return false;
}

void DeformDirectBaseUI::preDraw(int currentTime)
{

	return;
}

#define TIMER_FRAME_RATE 30
void DeformDirectBaseUI::postDraw(int currentTime)
{
	//push_gl_settings();

	frame_timer.Update();

	if (enabled_display)
	{
		controlStruct.draw();
	}

	if (keyFramer.GetOnOff())
	{
#ifdef DEBUG_PLUGINS_SEQUENTIAL_ORDER
		printf("DEBUG_PLUGINS_SEQUENTIAL_ORDER: DeformDirectBaseUI update key frame.\n");
#endif

		update_key_frame();
	}

	//pop_gl_settings();

	return;
}

void DeformDirectBaseUI::push_gl_settings()
{
	glPushMatrix();
	m_preview->camera.SetGL();
}

void DeformDirectBaseUI::pop_gl_settings()
{
	glPopMatrix();
}

void TW_CALL DeformDirectBaseUI::dialog_save_control_struct_state(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_save_file_path(fname);

	if(fname[0] == 0)
		return;

	static_cast<DeformDirectBaseUI *>(clientData)->save_control_struct_state(fname);
}

void TW_CALL DeformDirectBaseUI::dialog_load_control_struct_state(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_open_file_path(fname);

	if(fname[0] == 0)
		return;

	static_cast<DeformDirectBaseUI *>(clientData)->load_control_struct_state(fname);
}

void TW_CALL DeformDirectBaseUI::dialog_load_keyframing_config(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_open_file_path(fname);

	if(fname[0] == 0)
		return;

	static_cast<DeformDirectBaseUI*>(clientData)->load_key_frame_config(fname);
}

void TW_CALL DeformDirectBaseUI::dialog_start_keyframing(void *clientData)
{
	static_cast<DeformDirectBaseUI*>(clientData)->start_key_frame();
}

void TW_CALL DeformDirectBaseUI::dialog_load_control_struct_rotation_center(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_open_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<DeformDirectBaseUI *>(clientData)->load_rotation_center_from_file(fname);
}
