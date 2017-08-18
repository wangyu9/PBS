#include "main.h"
#include "Skinning.h"
#include <cstdio>
#include <string>
using namespace std;
#include <igl/dirname.h>
#include <igl/get_seconds_hires.h>
using namespace igl;
#include <iomanip>

#ifdef __APPLE__
#   include <GLUT/glut.h>
#   include <OpenGL/glu.h>
#else
#ifdef _WIN32
//#define FREEGLUT_STATIC
#   include <GL/freeglut.h>
#else
#   include <GL/glut.h>
#endif
#endif
#include "CTGALoader.h"

#ifdef _WIN32
	// I use left-hand trackball... :)
	#define OUR_RIGHT_BUTTON GLUT_LEFT_BUTTON
#else
	#define OUR_RIGHT_BUTTON GLUT_RIGHT_BUTTON
#endif

//#define SCREEN_CAPTURE
//#define DISPLAY_CROWD

int g_width = -1;
int g_height = -1;

// Remember which mouse button is down
int down_glutButton;

#ifdef SCREEN_CAPTURE
int g_imgCnt = 0;
double g_timerStart = 0.0;

void saveDoubleToFile(const string &fname, double number)
{
	FILE *f = fopen(fname.c_str(), "wt");
	fprintf(f, "%f", number);
	fclose(f);
}
#endif

void Display(void)
{
#ifndef DISPLAY_CROWD
  skinning->display();
#else
  skinning->displayCrowd(10, 10, 1.0f, 1.0f);
#endif
  // Present frame buffer
  glutSwapBuffers();
  // Recall Display at next frame
  glutPostRedisplay();

#ifdef SCREEN_CAPTURE
  double elapsedFPS = 1.0 / (get_seconds_hires() - g_timerStart);

  stringstream padnum; padnum << setw(4) << setfill('0') << g_imgCnt;
  string imgFname = "out/image" + padnum.str() + ".tga";  
  string fpsFname = "out/fps" + padnum.str() + ".txt";  
	printf("Saved image %i\n", g_imgCnt);
  CTGALoader tgaLoader;
  tgaLoader.SaveTGAScreenShot(imgFname.c_str(), g_width, g_height);
  saveDoubleToFile(fpsFname, elapsedFPS);
  g_imgCnt++;
  g_timerStart = get_seconds_hires();
#endif
}

void Reshape(int width, int height)
{
  skinning->resize(width,height);
  g_width = width;
  g_height = height;
}

// Function called at exit
void Terminate(void)
{ 
  delete skinning;
}

void key(unsigned char key,int mouse_x, int mouse_y)
{
  if(!skinning->key_down(key,mouse_x,g_height-mouse_y,false,false,false))
  {
  }
}

void mouse(int glutButton, int glutState, int mouse_x, int mouse_y)
{
  down_glutButton = glutButton;

  if(glutState==1)
  {
    // Should differentiate between types of clicks
    if(glutButton == OUR_RIGHT_BUTTON)
    {
      skinning->right_mouse_up(mouse_x,g_height-mouse_y,false,false,false);
    }else
    {
      skinning->mouse_up(mouse_x,g_height-mouse_y,false,false,false);
    }
  }else if(glutState==0)
  {
    if(glutButton == OUR_RIGHT_BUTTON)
    {
      skinning->right_mouse_down(mouse_x,g_height-mouse_y,false,false,false);
    }else
    {
      skinning->mouse_down(mouse_x,g_height-mouse_y,false,false,false);
    }
  }
}

void mouse_drag(int mouse_x, int mouse_y)
{
  if(down_glutButton == OUR_RIGHT_BUTTON)
  {
    if(!skinning->right_mouse_drag(mouse_x,g_height-mouse_y,false,false,false))
    {
    }
  }
  else
  {
    if(!skinning->mouse_drag(mouse_x,g_height-mouse_y,false,false,false))
    {
    }
  }
}

void mouse_move(int mouse_x, int mouse_y)
{
  if(!skinning->mouse_move(mouse_x,g_height-mouse_y,false,false,false))
  {
  }
}


int main(int argc, char *argv[])
{
  // Initialize GLUT
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
  glutInitWindowSize(960, 540);
  //glutInitWindowSize(1088, 612);
  //glutInitWindowSize(glutGet(GLUT_SCREEN_WIDTH),glutGet(GLUT_SCREEN_HEIGHT));
  // Center window
  glutInitWindowPosition(
      glutGet(GLUT_SCREEN_WIDTH)/2-glutGet(GLUT_INIT_WINDOW_WIDTH)/2,
      glutGet(GLUT_SCREEN_HEIGHT)/2-glutGet(GLUT_INIT_WINDOW_HEIGHT)/2);
  glutCreateWindow("SkinningGLUT");
  glutCreateMenu(NULL);
  // Set GLUT callbacks
  glutDisplayFunc(Display);
  glutReshapeFunc(Reshape);
#ifdef __APPLE__
  atexit(Terminate);  // this was causing funny issues in the destructor (weird string bugs)
#else
  glutWMCloseFunc(Terminate);
#endif
  
#ifdef _WIN32
  GLenum err = glewInit();
  if (GLEW_OK != err)
  {
      /* Problem: glewInit failed, something is seriously wrong. */
      fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
      return 1;
  }
#endif

  skinning = new Skinning();
  if(argc > 1)
  {
    if(string(argv[1]) == "-h" || string(argv[1]) == "--help")
    {
      printf("Usage:\n  ./skinning\nor\n  ./skinning [path to obj,dmat dir]\n");
      return 1;
    }
    skinning->load(string(argv[1]));
    skinning->load_shader_pair_from_files(LBS,"GLSL/lbs.frag");
  }

  // Hook up AntTweakBar to GLUT events
  glutMouseFunc(mouse);
  glutMotionFunc(mouse_drag);
  glutPassiveMotionFunc(mouse_move);
  glutKeyboardFunc(key);

  // Call the GLUT main loop
  glutMainLoop();
  return 0;
}
