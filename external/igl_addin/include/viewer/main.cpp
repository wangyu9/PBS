// IGL Viewer - Copyright (c) 2013 ETH Zurich. All rights reserved.

#include "Viewer.h"

#include <cstdlib>
#include <cstdio>
#include <cmath>

#ifdef __APPLE__
#   include <GLUT/glut.h>
#else
#   include <GL/glut.h>
#endif

#include "./plugins/DeformerPicking.h"//wangyu

Preview3D * preview_3D;

// Callback function called by GLUT to render screen
void Display(void)
{
  preview_3D->Draw(glutGet(GLUT_ELAPSED_TIME));
  // Present frame buffer
  glutSwapBuffers();
  glutReportErrors();
}


// Callback function called by GLUT when window size changes
void Reshape(int width, int height)
{
  preview_3D->resize(width,height);
  glutPostRedisplay();
}

// Function called at exit
void Terminate(void)
{ 
  delete preview_3D;
}

void mouse(int glutButton, int glutState, int mouse_x, int mouse_y)
{
    int key = glutGetModifiers();
    int modifiers = Preview3D::NO_KEY;
    if(key & GLUT_ACTIVE_SHIFT)
        modifiers = modifiers | Preview3D::SHIFT;
    if(key & GLUT_ACTIVE_CTRL)
        modifiers = modifiers | Preview3D::CTRL;
    if(key & GLUT_ACTIVE_ALT)
        modifiers = modifiers | Preview3D::ALT;
    
  if(glutState==1)
  {
    preview_3D->mouse_up(mouse_x,mouse_y, glutButton,modifiers);
  }
  else if(glutState==0)
  {
    preview_3D->mouse_down(mouse_x,mouse_y, glutButton, modifiers);
  }

  glutPostRedisplay();
}

void mouse_move(int mouse_x, int mouse_y)
{
  if(!preview_3D->mouse_move(mouse_x,mouse_y))
  {
  }
  glutPostRedisplay();
}

void keyboard(unsigned char k, int x, int y)
{

    int key = glutGetModifiers();
    int modifiers = Preview3D::NO_KEY;
    if(key & GLUT_ACTIVE_SHIFT)
        modifiers = modifiers | Preview3D::SHIFT;
    if(key & GLUT_ACTIVE_CTRL)
        modifiers = modifiers | Preview3D::CTRL;
    if(key & GLUT_ACTIVE_ALT)
        modifiers = modifiers | Preview3D::ALT;

    if(!TwEventKeyboardGLUT(k,x,y))
	{
		preview_3D->key_down(k,modifiers, x, y);
	}

    glutPostRedisplay();
}


// Main
int main(int argc, char *argv[])
{

  // Initialize GLUT
  glutInit(&argc, argv);
  glutInitDisplayString( "rgba depth double samples>=8 ");
  glutInitWindowSize(1280, 720);
  // Center window
  glutInitWindowPosition(INIT_WINDOW_POS_X,INIT_WINDOW_POS_Y);
  glutCreateWindow("Skinning Viewer");
  glutCreateMenu(NULL);
  // Set GLUT callbacks
  glutDisplayFunc(Display);
  glutReshapeFunc(Reshape);
  atexit(Terminate);  // Called after glutMainLoop ends

#ifdef _WIN32
  glewInit();
#endif
  preview_3D = new Preview3D(glutGet(GLUT_ELAPSED_TIME));
  // Hook up AntTweakBar to GLUT events
  // - Directly redirect GLUT mouse button events to AntTweakBar
  glutMouseFunc((GLUTmousebuttonfun)mouse);
  // - Directly redirect GLUT mouse motion events to AntTweakBar
  glutMotionFunc(mouse_move);
  // - Directly redirect GLUT mouse "passive" motion events to AntTweakBar (same as MouseMotion)
  glutPassiveMotionFunc(mouse_move);
  // - Directly redirect GLUT key events to AntTweakBar
  //glutKeyboardFunc((GLUTkeyboardfun)TwEventKeyboardGLUT);
  // pass keyboard events to preview3d
  glutKeyboardFunc(keyboard);
  // - Directly redirect GLUT special key events to AntTweakBar
  glutSpecialFunc((GLUTspecialfun)TwEventSpecialGLUT);
  // - Send 'glutGetModifers' function pointer to AntTweakBar;
  //   required because the GLUT key event functions do not report key modifiers states.
  TwGLUTModifiersFunc(glutGetModifiers);


  // Load the given mesh
  //if(argc < 0 ||     argv[1] == NULL ||     !preview_3D->load_mesh_from_file(argv[1]))
  //{}
  
  if (argc > 1)
  {
	  // notice that argv[0] is the path_name of the viewer, not the argument!!
	  //setDeformerPicking(argv[1]);
	  //DeformerPicking::GetReference().start_with_command(std::string(argv[1]));
  }
  else
  {
	  // really? not useful, since main is called later than construction function
	  //setDeformerPicking("\0");
  }


  // Call the GLUT main loop
  glutMainLoop();
  return 0;
}



