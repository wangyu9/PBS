#pragma once

#include "ViewerPlugin.h"
#include "./plugins/DeformerPicking.h"

#include "DeformMultiPhysBase.h"

class DeformMultiPhysBaseUI: public DeformMultiPhysBase, public PreviewPlugin
{
public:

	DeformMultiPhysBaseUI();
	~DeformMultiPhysBaseUI();

	// initialization (runs every time a mesh is loaded or cleared)
	void init(Preview3D *preview);

	igl::XMLSerializer* serializer;
	// implement Serializable interface
	bool Serialize(tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element);
	bool Deserialize(tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element);

	// keyboard callback
	bool keyDownEvent(unsigned char key, int modifiers, int mouse_x, int mouse_y);

	//mouse callback
	bool mouseDownEvent(int mouse_x, int mouse_y, int button, int modifiers);
	bool mouseUpEvent(int mouse_x, int mouse_y, int button, int modifiers);
	bool mouseMoveEvent(int mouse_x, int mouse_y);
	bool mouseScrollEvent(int mouse_x, int mouse_y, float delta);

	//stuff that is drawn by the plugin before the previewer has displayed the mesh
	//first draw 3d, then 2d
	void preDraw(int currentTime);
	//stuff that is drawn by the plugin after the previewer has displayed the mesh
	//first draw 3d, then 2d
	void postDraw(int currentTime);

};