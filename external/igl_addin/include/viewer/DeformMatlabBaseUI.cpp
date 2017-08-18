#include "DeformMatlabBaseUI.h"

#include "PluginManager.h"
#include "./plugins/PickingPlugin.h"

DeformMatlabBaseUI::DeformMatlabBaseUI() :DeformMatlabBase()
{
	//check in with the manager
	PluginManager().register_plugin(this);

	bar = NULL;

}

DeformMatlabBaseUI::~DeformMatlabBaseUI()
{
	// Inverse order to call deconstructor function
	delete serializer;
	DeformMatlabBase::~DeformMatlabBase();
}

// initialization (runs every time a mesh is loaded or cleared)
void DeformMatlabBaseUI::init(Preview3D *preview)
{
	PreviewPlugin::init(preview);

	// init menu bar
	if (bar == NULL)
	{
		serializer = new igl::XMLSerializer("DeformMatlab");

		// Create a tweak bar
		bar = new igl::ReTwBar;
		bar->TwNewBar("DeformMatlab");
		TwDefine(" DeformMatlab size='250 500' color='76 76 127' position='235 580' label='Deform Matlab' "); // change default tweak bar size and color
		bar->TwAddVarRW("RunSolver", TW_TYPE_BOOLCPP, &run_solver, "label='Run Solver'");
	}
}

bool DeformMatlabBaseUI::Serialize(tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element)
{
	//serializer->Add(with_dynamics, "with_dynamics");
	//serializer->Add(max_iteration, "max_iteration");

	// serialize previously added objects
	serializer->SaveToXMLDoc(doc);

	return igl::save_ReAntTweakBar(bar, doc);
}

bool DeformMatlabBaseUI::Deserialize(tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element)
{
	return igl::load_ReAntTweakBar(bar, doc);
}

bool DeformMatlabBaseUI::keyDownEvent(unsigned char key, int modifiers, int mouse_x, int mouse_y)
{
	return false;
}

//mouse callback
bool DeformMatlabBaseUI::mouseDownEvent(int mouse_x, int mouse_y, int button, int modifiers)
{
	return false;
}

bool DeformMatlabBaseUI::mouseUpEvent(int mouse_x, int mouse_y, int button, int modifiers)
{
	return false;
}

bool DeformMatlabBaseUI::mouseMoveEvent(int mouse_x, int mouse_y)
{
	return false;
}

bool DeformMatlabBaseUI::mouseScrollEvent(int mouse_x, int mouse_y, float delta)
{
	return false;
}

//stuff that is drawn by the plugin before the previewer has displayed the mesh
//first draw 3d, then 2d
void DeformMatlabBaseUI::preDraw(int currentTime)
{
	DeformMatlabBase::Update();
}

//stuff that is drawn by the plugin after the previewer has displayed the mesh
//first draw 3d, then 2d
void DeformMatlabBaseUI::postDraw(int currentTime)
{
}