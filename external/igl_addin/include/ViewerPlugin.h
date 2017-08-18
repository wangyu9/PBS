// IGL Viewer - Copyright (c) 2013 ETH Zurich. All rights reserved.

#ifndef ViewerPlugin_h
#define ViewerPlugin_h

#include "Viewer.h"
#include <libigl_removed/xml/XMLSerializer.h> //old path #include "igl/xml/XMLSerializer.h"

// Abstract class for plugins
// All plugins MUST have this class as their parent and implement all the callbacks
// For an example of a basic plugins see plugins/skeleton.h
//
// Return value of callbacks: returning true to any of the callbacks tells Preview3D that the event has been 
// handled and that it should not be passed to other plugins or to other internal functions of Preview3D

inline bool twSetWindowPos(igl::ReTwBar* bar, int x, int y)
{
	if (bar == NULL)
		return false;

	int pos[2];
	bar->TwGetParam(NULL, "position", TW_PARAM_INT32, 2, pos);

	pos[0] = x;
	pos[1] = y;

	bar->TwSetParam(NULL, "position", TW_PARAM_INT32, 2, pos);

	return true;
}

inline bool twGetWindowPos(igl::ReTwBar* bar, int *pos)
{
	if (bar == NULL)
		return false;

	bar->TwGetParam(NULL, "position", TW_PARAM_INT32, 2, pos);

	return true;
}


inline bool twSetVisiable(igl::ReTwBar* bar, bool visiable)
{
	if (bar == NULL)
		return false;
	
	bool pVisiable[1];
	pVisiable[0] = visiable;

	// Not finished

	//bar->TwSetParam(NULL, "visible", TW_TYPE_BOOLCPP, 1, pVisiable);
	//bar->TwDefine(" TW_HELP visible=false ");  // help bar is hidden
	TwDefine(" TW_HELP visible=false ");  // help bar is hidden

	return true;
}

class PreviewPluginAddin // this is added by wangyu
{
public:
	igl::ReTwBar* bar;

	PreviewPluginAddin():bar(NULL){}
};

class PreviewPlugin : public igl::XMLSerializable, public PreviewPluginAddin
{
public:
  // default implementation of serializer interface
  virtual bool Serialize(tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element) { return false; }
  virtual bool Deserialize(tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element) {return false; }
  virtual void Init() {}
  
  PreviewPlugin(){};
  ~PreviewPlugin(){};
  // Runs immediately after a new mesh had been loaded. 
  // Note: this callback is also called on startup if no mesh has been loaded, in this case the vertices and faces
  //       matrices of Preview3D will be empty
  virtual void init(Preview3D *preview)
  {
    m_preview = preview;
  }

  // It is called before the draw procedure of Preview3D
  virtual void preDraw(int currentTime) = 0;

  // It is called after the draw procedure of Preview3D
  virtual void postDraw(int currentTime)=0;

  // It is called when the mouse button is pressed
  // - button can be GLUT_LEFT_BUTTON, GLUT_MIDDLE_BUTTON or GLUT_RIGHT_BUTTON
  // - modifiers is a bitfield that might one or more of the following bits Preview3D::NO_KEY, Preview3D::SHIFT, Preview3D::CTRL, Preview3D::ALT;
  virtual bool mouseDownEvent(int mouse_x, int mouse_y, int button, int modifiers)=0;

  // It is called when the mouse button is released
  // - button can be GLUT_LEFT_BUTTON, GLUT_MIDDLE_BUTTON or GLUT_RIGHT_BUTTON
  // - modifiers is a bitfield that might one or more of the following bits Preview3D::NO_KEY, Preview3D::SHIFT, Preview3D::CTRL, Preview3D::ALT;
  virtual bool mouseUpEvent(int mouse_x, int mouse_y, int button, int modifiers)=0;

  // It is called every time the mouse is moved
  // - mouse_x and mouse_y are the new coordinates of the mouse pointer in screen coordinates
  virtual bool mouseMoveEvent(int mouse_x, int mouse_y)=0;

  // It is called every time the scroll wheel is moved
  // Note: this callback is not working with every glut implementation
  virtual bool mouseScrollEvent(int mouse_x, int mouse_y,  float delta_y)=0;

  // It is called when a keyboard key is pressed
  // - modifiers is a bitfield that might one or more of the following bits Preview3D::NO_KEY, Preview3D::SHIFT, Preview3D::CTRL, Preview3D::ALT;
  // - mouse_x and mouse_y are the current coordinates of the mouse pointer in screen coordinates 
  virtual bool keyDownEvent(unsigned char key, int modifiers, int mouse_x, int mouse_y)=0;

  virtual bool commandLine( std::string c, std::vector<std::string> cl) { return false; }
protected:
  // Pointer to the main Preview3D class
  Preview3D *m_preview;

};

#endif
