#include "DeformARAPBaseUI.h"

#include "PluginManager.h"
#include "./plugins/PickingPlugin.h"

DeformARAPBaseUI::DeformARAPBaseUI() :DeformARAPBase()
{
	//check in with the manager
	PluginManager().register_plugin(this);

	bar = NULL;

}

DeformARAPBaseUI::~DeformARAPBaseUI()
{
	// Inverse order to call deconstructor function
	delete serializer;
	DeformARAPBase::~DeformARAPBase();
}

// initialization (runs every time a mesh is loaded or cleared)
void DeformARAPBaseUI::init(Preview3D *preview)
{
	PreviewPlugin::init(preview);

	// init menu bar
	if (bar == NULL)
	{
		serializer = new igl::XMLSerializer("DeformARAP");

		// Create a tweak bar
		bar = new igl::ReTwBar;
		bar->TwNewBar("DeformARAP");
		TwDefine(" DeformARAP size='250 500' color='76 76 127' position='480 580' label='Deform ARAP' "); // change default tweak bar size and color
		bar->TwAddVarRW("RunSolver", TW_TYPE_BOOLCPP, &run_solver, "label='Run Solver'");
	}
}

bool DeformARAPBaseUI::Serialize(tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element)
{
	//serializer->Add(with_dynamics, "with_dynamics");
	//serializer->Add(max_iteration, "max_iteration");

	// serialize previously added objects
	serializer->SaveToXMLDoc(doc);

	return igl::save_ReAntTweakBar(bar, doc);
}

bool DeformARAPBaseUI::Deserialize(tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element)
{
	return igl::load_ReAntTweakBar(bar, doc);
}

bool DeformARAPBaseUI::keyDownEvent(unsigned char key, int modifiers, int mouse_x, int mouse_y)
{
	return false;
}

//mouse callback
bool DeformARAPBaseUI::mouseDownEvent(int mouse_x, int mouse_y, int button, int modifiers)
{
	return false;
}

bool DeformARAPBaseUI::mouseUpEvent(int mouse_x, int mouse_y, int button, int modifiers)
{
	return false;
}

bool DeformARAPBaseUI::mouseMoveEvent(int mouse_x, int mouse_y)
{
	return false;
}

bool DeformARAPBaseUI::mouseScrollEvent(int mouse_x, int mouse_y, float delta)
{
	return false;
}

//stuff that is drawn by the plugin before the previewer has displayed the mesh
//first draw 3d, then 2d
void DeformARAPBaseUI::preDraw(int currentTime)
{
	DeformARAPBase::Update();
}

//stuff that is drawn by the plugin after the previewer has displayed the mesh
//first draw 3d, then 2d
void DeformARAPBaseUI::postDraw(int currentTime)
{
}