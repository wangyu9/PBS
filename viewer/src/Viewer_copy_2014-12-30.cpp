

#include "Viewer.h"
#include "ViewerTrackball.h"

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

#include <cmath>
#include <cstdio>
#include <string>
#include <sstream>
#include <iomanip>

#include <algorithm>
using namespace std;

#include "FileDialog.h"

// Undef Visual Studio macros...
#undef max
#undef min

#include <limits>
#include <cassert>

#ifndef PREVIEW3D_NO_SHADERS
#include "directionalperpixel.h"
#include "directionalperpixelcolor.h"
#include "isolines.h"
#endif

#ifndef _NOMATLAB_
#include "matlabIO.h"
#endif

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#define PBS_SHADER_VERT_PATH "C:\\WorkSpace\\Visual Studio 2013\\PBS\\viewer\\src\\GLSL\\PBSshader.vert"
#define PBS_SHADER_FRAG_PATH "C:\\WorkSpace\\Visual Studio 2013\\PBS\\viewer\\src\\GLSL\\PBSshader.frag"

#ifdef USING_IGL_HEADER_ONLY_MODE
#define IGL_HEADER_ONLY 
#endif


#include "igl/readOBJ.h"
#include "igl/readOFF.h"
#include "igl/readMESH.h"//wangyu
#include "igl/readDMAT.h"//wangyu
#include "igl/per_face_normals.h"
#include "igl/per_vertex_normals.h"
#include "igl/per_corner_normals.h"

#include <draw_primitives.h>//wangyu
#include <draw_mesh_with_pbs_shader.h>///wangyu



//#include "igl/vf.h"
#include <vf.h>//wangyu this is removed from igl
#include "igl/adjacency_list.h"
#include "igl/writeOBJ.h"
#include "igl/writeOFF.h"
#include "igl/writeMESH.h"//wangyu
#include "igl/writeDMAT.h"//wangyu
#include "igl/timer.h"
#include "igl/png/render_to_png.h"//wangyu


#include "texture_png.h"//wangyu
#include "stb_image_write.h"//added by wangyu, file got from tiantian
#include <time.h>//wangyu

#include "PluginManager.h"

#include "../plugins/DeformSkinning.h"//wangyu

//
//#define SHADER_INDEX_OF_WEIGHTS 4
//#define SHADER_INDEX_OF_WEIGHT_INDICES 9


// Max line size for reading files
#define REBAR_NAME "Viewer"
#define MAX_LINE 1000
#define SQRT_2_OVER_2 0.707106781f
#define NUM_CANONICAL_VIEW_QUATERNIONS 24
#define pi 3.1415926535897932384626433832795

float Preview3D::ZERO[] = {0.0f,0.0f,0.0f,0.0f};
float Preview3D::DEFAULT_QUATERNION[] = { 0.0f, 0.0f, 0.0f, 1.0f };

float Preview3D::DEFAULT_LIGHT_DIRECTION[] = { -0.50f, -0.40f, -0.75f};

float Preview3D::CANONICAL_VIEW_QUATERNIONS[][4] = 
{
  {             0,             0,             0,             1},
  {             0,             0, SQRT_2_OVER_2, SQRT_2_OVER_2},
  {             0,             0,             1,             0},
  {             0,             0, SQRT_2_OVER_2,-SQRT_2_OVER_2},
  
  {             0,            -1,             0,             0},
  {-SQRT_2_OVER_2, SQRT_2_OVER_2,             0,             0},
  {            -1,             0,             0,             0},
  {-SQRT_2_OVER_2,-SQRT_2_OVER_2,             0,             0},
  
  {          -0.5,          -0.5,          -0.5,           0.5},
  {             0,-SQRT_2_OVER_2,             0, SQRT_2_OVER_2},
  {           0.5,          -0.5,           0.5,           0.5},
  { SQRT_2_OVER_2,             0, SQRT_2_OVER_2,             0},
  
  { SQRT_2_OVER_2,             0,-SQRT_2_OVER_2,             0},
  {           0.5,           0.5,          -0.5,           0.5},
  {             0, SQRT_2_OVER_2,             0, SQRT_2_OVER_2},
  {          -0.5,           0.5,           0.5,           0.5},
  
  {             0, SQRT_2_OVER_2, SQRT_2_OVER_2,             0},
  {          -0.5,           0.5,           0.5,          -0.5},
  {-SQRT_2_OVER_2,             0,             0,-SQRT_2_OVER_2},
  {          -0.5,          -0.5,          -0.5,          -0.5},
  
  {-SQRT_2_OVER_2,             0,             0, SQRT_2_OVER_2},
  {          -0.5,          -0.5,           0.5,           0.5},
  {             0,-SQRT_2_OVER_2, SQRT_2_OVER_2,             0},
  {           0.5,          -0.5,           0.5,          -0.5}
};



float Preview3D::GOLD_AMBIENT[4] = { 51.0/255.0,43.0/255.0,33.3/255.0,1.0f };
float Preview3D::GOLD_DIFFUSE[4] = { 255.0/255.0,228.0/255.0,58.0/255.0,1.0f };
float Preview3D::GOLD_SPECULAR[4] = { 255.0/255.0,235.0/255.0,80.0/255.0,1.0f };
float Preview3D::SILVER_AMBIENT[4] = { 0.2f, 0.2f, 0.2f, 1.0f };
float Preview3D::SILVER_DIFFUSE[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
float Preview3D::SILVER_SPECULAR[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
float Preview3D::DARK_BLUE[4] = { 0.3f, 0.3f, 0.5f, 1.0f };
float Preview3D::BLACK[4] = {0.0f,0.0f,0.0f,1.0f};
float Preview3D::WHITE[4] = {1.0f,1.0f,1.0f,1.0f};

ColorPreset Preview3D::color_presets[NUM_COLOR_PRESETS] = { 
  ColorPreset(Preview3D::GOLD_AMBIENT,  Preview3D::GOLD_DIFFUSE,  Preview3D::GOLD_SPECULAR,  Preview3D::DARK_BLUE),
  ColorPreset(Preview3D::SILVER_AMBIENT,Preview3D::SILVER_DIFFUSE,Preview3D::SILVER_SPECULAR,Preview3D::DARK_BLUE),
  ColorPreset(Preview3D::SILVER_AMBIENT,Preview3D::SILVER_DIFFUSE,Preview3D::SILVER_SPECULAR,Preview3D::WHITE),
  ColorPreset(Preview3D::GOLD_AMBIENT,  Preview3D::GOLD_DIFFUSE,  Preview3D::GOLD_SPECULAR,  Preview3D::WHITE),
};

Preview3D::Preview3D(int start_time, bool deleteonexit)
{      
  // Initialize fields
  serializer = new igl::XMLSerializer("IGLViewer");
   
  m_pause = false;//added by wangyu
  use_glCallList = false;//added by wangyu

  m_full_screen = false;
  m_recording_screen = false;//added by wangyu
  m_record_one_frame = false;//added by wangyu
  bFlipYCoord = false;//added by wangyu
  strcpy(current_working_folder,"C:\\WorkSpace");//added by wangyu
  frame_index = 0;//added by wangyu

  mesh_alpha = 1.;//added by wangyu

  number_of_vertices = 0;
  number_of_faces = 0;
  number_of_tets = 0;

  delete_on_exit = deleteonexit;
  corner_threshold = 20;
  // Causes gldeletelists to return error on first call
  faces_display_list = -1;
  frustum_shift_x = 0.125;
  //    frustum_shift_x = 0.;
  width_percentage = 1.;
  height_percentage = 1.;
  view_angle = 45.0;
  dnear = 1.0;
  dfar = 100.0;
  g_Zoom = 1.0f;
  zoom = 1.0f;
  g_Translation << 0,0,0;
  Preview3D::CopyArray4(DEFAULT_QUATERNION,g_Rotation);
  trackball_snap_to_canonical = 0.0;
  enable_rotation = true;
  g_AutoRotate = 0;
  g_KeyBoardRotate = false;//wangyu
  g_RotateSpeed = 1.0;//wangyu
  key_board_x_rotation = 0.0;
  key_board_y_rotation = 0.0;
  auto_rotate_restart = false;
  g_RotateTime = 0;
  Preview3D::CopyArray4(DEFAULT_QUATERNION,g_RotateStart);
  Preview3D::CopyArray4(GOLD_AMBIENT,g_MatAmbient);
  Preview3D::CopyArray4(GOLD_DIFFUSE,g_MatDiffuse);
  Preview3D::CopyArray4(GOLD_SPECULAR,g_MatSpecular);
  g_MatShininess = 35.0f;
  Preview3D::CopyArray4(DARK_BLUE,background_color);
  Preview3D::CopyArray4(BLACK,line_color);
  use_lighting = true;
  draw_grid = false;
  g_LightMultiplier = 1.0f;
  Preview3D::CopyArray3(DEFAULT_LIGHT_DIRECTION,g_LightDirection);
  normals_type = PER_VERTEX;//this is not used, wangyu
  per_vertex_normal_type = igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_AREA;
  normals_changed = false;
  is_compiled = false;
  show_faces = true;
  show_lines = true;
  invert_normals = false;
  show_overlay = true;
  show_overlay_depth = true;
  show_vertid = false;
  show_faceid = false;
  enable_autoRefresh = false;
  useOthographic = false;
  show_trackball = false;
  numIsoLevels = 50;
  show_isolines = false;
  show_texture = false;
  filename = std::string("");
  texture_id = 0;
  linewidth = 1;
  
#ifndef PREVIEW3D_NO_SHADERS
  meshDrawingType = MESH_DRAWING_DEFAULT;
  // Load shaders
  load_shader();
  //s_directionalPerPixelProgram = loadShaderProgramStr(directionalperpixel_vert, directionalperpixel_frag);
  //s_directionalPerPixelColorProgram = loadShaderProgramStr(directionalperpixelcolor_vert, directionalperpixelcolor_frag);
  //s_pbsShaderProgram = loadShaderProgram(PBS_SHADER_VERT_PATH, PBS_SHADER_FRAG_PATH);
  shader_mode = OFF;
  shader_id = 0;

  //printShaderInfoLog(s_pbsShaderProgram.p);
  //printProgramInfoLog(s_pbsShaderProgram.p); 
#endif
  
  // Display options
  colorBarType = COLORBAR_HIGHLIGHT_NEG;


  // initialize scroll position to 0
  scroll_position = 0.0f;
  down = false;
  bar = new igl::ReTwBar;
  barWidth = 200;
  barHeight = 685;
  maxBarHeight = 1200;
  float axis[] = { 0.7f, 0.7f, 0.0f }; // initial model rotation
  float angle = 0.8f;

  timer = new igl::Timer();
  
  // Initialize AntTweakBar
  if(!TwInit(TW_OPENGL, NULL) )
  {
    // A fatal error occured    
    fprintf(stderr, "AntTweakBar initialization failed: %s\n", TwGetLastError());
  }
  
  // Create a tweak bar
  bar->TwNewBar("IGLViewer");
  TwDefine(" IGLViewer help='This is a simple 3D mesh viewer.' "); // Message added to the help bar->
  stringstream barSize;
  barSize << " IGLViewer size='" << barWidth << " " << barHeight << "'";  
  TwDefine(barSize.str().c_str()); // change default tweak bar size
  TwDefine(" IGLViewer color='76 76 127' "); // change default tweak bar color
  TwDefine(" IGLViewer refresh=0.5"); // change refresh rate
  
  //TwCopyStdStringToClientFunc(CopySTdStringToClient);

  // added by wagnyu
  bar->TwAddVarRW("Pause",TW_TYPE_BOOLCPP,&m_pause," label='Turn on/off pause.' key=p ");

  // ---------------------- GENERAL STATS ----------------------
  
  bar->TwAddVarRO("NumberOfVertices",TW_TYPE_UINT32, &number_of_vertices,
             " label='Number of vertices' help='Displays number of vertices in mesh.'"
             " group='Mesh statistics'");
  bar->TwAddVarRO("NumberOfFaces",TW_TYPE_UINT32, &number_of_faces,
             " label='Number of faces' help='Displays number of faces in mesh.'"
             " group='Mesh statistics'");
  bar->TwAddVarRO("NumberOfTets",TW_TYPE_UINT32, &number_of_tets,
	  " label='Number of tets' help='Displays number of tets in mesh.'"
	  " group='Mesh statistics'");
  bar->TwAddVarRO("fps",TW_TYPE_DOUBLE, &fps,
             " label='Frames per second' help='Displays current number of frames"
             " drawn per second.'"
             " group='Mesh statistics'");

  // ---------------------- RECORDING ------------------------
  //Added by wangyu
  bar->TwAddVarCB("Full Screen", TW_TYPE_BOOLCPP, SetFullScreenCB, GetFullScreenCB, this, " group='Recording' key=f");
  bar->TwAddVarRW("Recording Screen",TW_TYPE_BOOLCPP,&m_recording_screen,
			" label='Turn on/off recording of screen.' group='Recording' key=r");
  bar->TwAddButton("Record One Frame", record_frame_CB, this,
	  " group='Recording'"
	  " label='Record One Frame.' ");

  // ---------------------- LOADING ----------------------
  
  bar->TwAddButton("Load Mesh", open_dialog_mesh, this,
              " group='Load & Save'"
              " label='Open mesh' key=o help='Load a mesh.'");
  bar->TwAddButton("Save Mesh", save_dialog_mesh, this,
				" group='Load & Save'"
				" label='Save mesh'"
				);// added by wangyu
  bar->TwAddButton("Load Texture", open_dialog_texture, this,
              " group='Load & Save'"
              " label='Open texture' key=O help='Load a texture.'");
  bar->TwAddButton("Load Scene", LoadSceneCB, this, 
        "group='Load & Save'"
        " label='Load Scene' help='Save a scene.'");
  bar->TwAddButton("Save Scene", SaveSceneCB, this,
        "group='Load & Save'"
        " label='Save Scene' help='Load a scene.'");
  bar->TwAddButton("Load Camera", LoadCameraCB, this, 
	  "group='Load & Save'"
	  " label='Load Camera' help='Save a Camera.'");
  bar->TwAddButton("Save Camera", SaveCameraCB, this,
	  "group='Load & Save'"
	  " label='Save Camera' help='Load a Camera.'");
  bar->TwAddButton("Compile Mesh", compile_dialog_mesh, this,
	  " group='Load & Save'"
	  " label='Compile mesh' key=o help='Compile a mesh.'");//added by wangyu
  bar->TwAddButton("Load Pose Mesh", open_dialog_pose_mesh, this,
	  " group='Load & Save'"
	  " label='Open pose mesh'");
  bar->TwAddVarRW( "Flip Y Coord", TW_TYPE_BOOLCPP, &bFlipYCoord,
	  " group='Load & Save'"
	  " label='Flip Y Coord'");//added by wangyu
  bar->TwAddButton("Save Tets", save_dialog_tets, this,
	  " group='Load & Save'"
	  " label='Save tets'");// added by wangyu
  bar->TwAddButton("Send Status to Matlab", SendStatusToMatlabCB, this, " group='Load & Save'");//added by wangyu
  // ---------------------- Overlays ----------------------
  
  bar->TwAddVarRW( "Wireframe", TW_TYPE_BOOLCPP, &show_lines,
             " group='Overlays'"
             " label='Wireframe' key=l help='Toggle wire frame of mesh'");
  bar->TwAddVarRW( "Fill", TW_TYPE_BOOLCPP, &show_faces,
             " group='Overlays'"
             " label='Fill' key=t help='Display filled polygons of mesh'");
  bar->TwAddVarRW( "ShowVertexId", TW_TYPE_BOOLCPP, &show_vertid,
             " group='Overlays'"
             " label='Show Vertex Labels' key=';' help='Toggle vertex indices'");
  bar->TwAddVarRW( "ShowFaceId", TW_TYPE_BOOLCPP, &show_faceid,
             " group='Overlays'"
             " label='Show Faces Labels' key='CTRL+;' help='Toggle face"
             " indices'");

  bar->TwAddVarRW( "Show Isolines", TW_TYPE_BOOLCPP, &show_isolines,
             " group='Overlays'"
             " label='Show Isolines' help='Toggle display of isolines of scalar property'");
  bar->TwAddVarCB("Isolines #",TW_TYPE_INT32, set_numIsoLevelsCB, get_numIsoLevelsCB, this,
             " group='Overlays'"
             " label='Isolines #' help='Number of Isolines of the scalar field to display'");
  
  // ---------------------- SCENE ----------------------
  
  bar->TwAddButton("ViewXYPlane", view_xy_planeCB, this,
              " group='Scene'"
              " label='View XY Plane' key=z help='View the XY plane.'");
  bar->TwAddButton("ViewXZPlane", view_xz_planeCB, this,
              " group='Scene'"
              " label='View XZ Plane' key=y help='View the XZ plane.'");
  bar->TwAddButton("ViewYZPlane", view_yz_planeCB, this,
              " group='Scene'"
              " label='View YZ Plane' key=x help='View the XY plane.'");
  
#ifndef PREVIEW3D_NO_SHADERS

  // added by wangyu:
  TwEnumVal meshDrawingEV[NUM_MESH_DRAWING_TYPE] = {
	  {MESH_DRAWING_DEFAULT,"DEFAULT"},
	  {MESH_DRAWING_PBS,"PBS"}
  };
  TwType meshDrawingT = TwDefineEnum("Mesh Drawing Type",meshDrawingEV, NUM_MESH_DRAWING_TYPE);
  bar->TwAddVarCB("Mesh Drawing Type", meshDrawingT, SetMeshDrawingTypeCB, GetMeshDrawingTypeCB, this, " group='Scene'");

  TwEnumVal ShaderModeEV[NUM_SHADER_MODE] = {
    {OFF,"OFF"},
    {DIRECTIONAL_PER_PIXEL, "Per pixel lighting"}
  };
  TwType ShaderModeTW = igl::ReTwDefineEnum("ShaderMode", ShaderModeEV, NUM_SHADER_MODE);
  bar->TwAddVarCB(
             
             "Shader",
             ShaderModeTW,
             set_shader_modeCB,
             get_shader_modeCB,
             this,
             " group='Scene' help='Select shader in use' keyIncr='>' keyDecr='<'");
  bar->TwAddVarRW("Update Shader Attribs Every Frame", TW_TYPE_BOOLCPP, &update_shader_attribs_every_frame,
			" group='Scene'");
  bar->TwAddButton("Reload Shader", ReLoadShader, this, " group='Scene'");
#endif
  
  // Add 'g_Zoom' to 'bar': this is a modifable (RW) variable of type TW_TYPE_FLOAT. Its key shortcuts are [z] and [Z].
  bar->TwAddVarRW( "Zoom", TW_TYPE_FLOAT, &g_Zoom,
             " min=0.05 max=50 step=0.1 keyIncr=+ keyDecr=- help='Scale the object (1=original size).' group='Scene'");
  bar->TwAddVarRW( "ObjTranslation", TW_TYPE_DIR3F, &g_Translation, 
             " group='Scene'"
             " label='Object translation' help='Change the object translation.' ");
  // Add 'g_Rotation' to 'bar': this is a variable of type TW_TYPE_QUAT4F which defines the object's orientation
  bar->TwAddVarRW( "ObjRotation", TW_TYPE_QUAT4F, &g_Rotation, 
             " group='Scene'"
             " label='Object rotation' open help='Change the object orientation.' ");

  bar->TwAddVarRW("Rotate Speed", TW_TYPE_FLOAT, &g_RotateSpeed, " ");
  // Add callback to toggle auto-rotate mode (callback functions are defined above).
  bar->TwAddVarCB( "AutoRotate", TW_TYPE_BOOLCPP, SetAutoRotateCB, GetAutoRotateCB,this,
             " label='Auto-rotate' help='Toggle auto-rotate mode.' key=m");
  bar->TwAddVarRW(" KeyRotate", TW_TYPE_BOOLCPP, &g_KeyBoardRotate, 
			" label='Key-rotate'");
  bar->TwAddButton("SnapView", snap_to_canonical_quaternionCB, this,
              " group='Scene'"
              " label='Snap to canonical view' key=Z "
              " help='Snaps view to nearest canonical view.'");
  bar->TwAddVarRW("TrackballSnapView", TW_TYPE_DOUBLE,
             &trackball_snap_to_canonical,
             " group='Scene'"
             " label='Trackball snap' keyIncr=D keyDecr=S step=0.05 min=0.0 max=1.0"
             " help='Snaps view to nearest canonical view while using trackball. "
             " 0.0 means no snapping, 1 mean complete snapping'");
  bar->TwAddVarRW("Lighting",TW_TYPE_BOOLCPP, &use_lighting,
             " group='Scene'"
             " label='Lighting' key=L help='Use lighting when displaying model'");
  // Add 'g_LightMultiplier' to 'bar': this is a variable of type TW_TYPE_FLOAT. Its key shortcuts are [+] and [-].
  bar->TwAddVarRW( "Multiplier", TW_TYPE_FLOAT, &g_LightMultiplier, 
             " group='Scene'"
             " label='Light booster' min=0.1 max=4 step=0.02 help='Increase/decrease the light power.' ");
  // Add 'g_LightDirection' to 'bar': this is a variable of type TW_TYPE_DIR3F which defines the light direction
  bar->TwAddVarRW( "LightDir", TW_TYPE_DIR3F, &g_LightDirection, 
             " group='Scene'"
             " label='Light direction' open help='Change the light direction.' ");
  bar->TwAddVarCB( "LineWidth", TW_TYPE_INT32, SetLineWidthCB, GetLineWidthCB,this,
             " group='Scene'");
   
  bar->TwAddVarCB( "Show Texture", TW_TYPE_BOOLCPP, SetShowTextureCB, GetShowTextureCB, this,
             " label='Show Texture'");    
  

  // ---------------------- DRAW OPTIONS ----------------------

  bar->TwAddVarRW("Mesh Alpha", TW_TYPE_FLOAT, &mesh_alpha, " group='Draw options' max=1.0 min=0.0 step=0.1");

  // added by wangyu:
  TwEnumVal colorBarTypeEV[NUM_OF_COLORBAR_TYPE] = {
	  { COLORBAR_IGL_DEFAULT, "DEFAULT" },
	  { COLORBAR_ZERO_ONE, "0-1" },
	  { COLORBAR_HIGHLIGHT_NEG, "0-1 (hightlight neg)" },
	  { COLORBAR_ZERO_ONE_GREY_RED, "0-1 grey-red" }
  };
  TwType colorBarT = TwDefineEnum("Color Bar Type", colorBarTypeEV, NUM_OF_COLORBAR_TYPE);
  bar->TwAddVarCB("Color Bar Type", colorBarT, SetColorBarTypeCB, GetColorBarTypeCB, this, " group='Draw options'");


  bar->TwAddButton("Send Color to MATALB", SendColorToMatlabCB, this, " group='Draw options'");

  bar->TwAddVarRW( "Auto Refresh", TW_TYPE_BOOLCPP, &enable_autoRefresh,  
             " group='Draw options'"
             " label='Auto Refresh' help='Enables continuously redrawing of screen'");
  bar->TwAddVarRW("Trackball",TW_TYPE_BOOLCPP, &show_trackball,
             " group='Scene'"
             " label='Show trackball' key='B' help='Show the trackball in the scene.'");
  bar->TwAddVarCB( "Toggle Orthographic/Perspective", TW_TYPE_BOOLCPP, set_toggle_orthoCB, get_toggle_orthoCB, this, 
             " group='Viewing Options'"
             " label='Orthographic view' "
             " help='Toggles orthographic / perspective view. Default: perspective.'");
  bar->TwAddButton("Align Camera Center", alignCameraCenterCB, this,
              " group='Viewing Options'"
              " label='Align Camera' key=A help='Set the center of the camera to the mesh center.'");
  
  bar->TwAddVarCB("Corner Threshold",TW_TYPE_DOUBLE, set_corner_thresholdCB, get_corner_thresholdCB, this,
             " group='Draw options'"
             " label='Corner Threshold' help='Angle difference (cosine) above which there is a sharp feature'");
  TwEnumVal NormalsTypeEV[NUM_COLOR_PRESET_IDS] = {
    {PER_FACE,  "PER_FACE"  },
    {PER_VERTEX,"PER_VERTEX"},
    {PER_CORNER,"PER_CORNER"}
  };
  TwType NormalsTypeTW = 
  igl::ReTwDefineEnum("NormalsType", NormalsTypeEV, NUM_NORMALS_TYPE);
  bar->TwAddVarRW( "NormalsType", NormalsTypeTW, &normals_type,
             " group='Draw options'"
             " label='Normals Type' key=T help='Toggle per face shading or per"
             " vertex shading or per corner shading.' ");

#define NUM_PER_VERTEX_NORMAL_TYPE 3
  TwEnumVal PerVertexNormalsTypeEV[NUM_PER_VERTEX_NORMAL_TYPE] = {
	  {igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_UNIFORM,  "UNIFORM_WEIGHTING"},
	  {igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_AREA,"AREA_WEIGHTING"},
	  {igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_ANGLE,"ANGLE_WEIGHTING"}
  };
  TwType PerVertexNormalsTypeTW = 
	  igl::ReTwDefineEnum("PerVertexNormalsType", PerVertexNormalsTypeEV, NUM_PER_VERTEX_NORMAL_TYPE);
  bar->TwAddVarRW( "PerVertexNormalsType", PerVertexNormalsTypeTW, &per_vertex_normal_type,
	  " group='Draw options'");

  bar->TwAddButton( "InvertNormals", invert_normalsCB, this,  
              " group='Draw options'"
              " label='Invert normals' key=i help='Invert normal directions for inside out meshes.' ");
  bar->TwAddVarRW( "Draw Grid", TW_TYPE_BOOLCPP, &draw_grid,  
             " group='Draw options'"
             " label='Draw Grid' key=g help='Draw a grid below the mesh.' ");
  
  bar->TwAddVarRW( "ShowOverlay", TW_TYPE_BOOLCPP, &show_overlay,  
             " group='Draw options'"
             " label='Show overlay' key=o help='Show the overlay layers.' ");
  bar->TwAddVarRW( "ShowOverlayDepth", TW_TYPE_BOOLCPP, &show_overlay_depth,  
             " group='Draw options'"
             " label='Show overlay depth test' help='Enable the depth test for overlay layer.' ");
  bar->TwAddVarRW( "Background color", TW_TYPE_COLOR3F,
             &background_color,
             " help='Select a background color' colormode=hls");
  TwEnumVal ColorPresetEV[NUM_COLOR_PRESET_IDS] = {
    {GOLD_ON_DARK_BLUE, "Gold on dark blue"},
    {SILVER_ON_DARK_BLUE, "Silver on dark blue"},
    {SILVER_ON_WHITE, "Silver on white"},
    {GOLD_ON_WHITE, "Gold on white"},
    {CUSTOM,"CUSTOM"},
  };
  TwType ColorPresetTW = 
  igl::ReTwDefineEnum("ColorPreset", ColorPresetEV, NUM_COLOR_PRESET_IDS);
  bar->TwAddVarCB(
              
             "ColorPreset",
             ColorPresetTW, 
             set_color_presetCB, 
             get_color_presetCB, 
             this,
             " group='Scene' help='Select from background and foreground color pair"
             " presets' label='Color Preset' key=c"); 
  bar->TwAddVarRW( "LineColor", TW_TYPE_COLOR3F,
             &line_color,
             " label='Line color' help='Select a outline color' ");
  // Add 'g_MatAmbient' to 'bar': this is a variable of type TW_TYPE_COLOR4F (4 floats color)
  // and is inserted into a group named 'Material'.
  bar->TwAddVarRW( "Ambient", TW_TYPE_COLOR4F, &g_MatAmbient, " group='Material' ");
  // Add 'g_MatDiffuse' to 'bar': this is a variable of type TW_TYPE_COLOR4F (4 floats color)
  // and is inserted into group 'Material'.
  bar->TwAddVarRW("Diffuse", TW_TYPE_COLOR4F, &g_MatDiffuse, " group='Material' ");
  bar->TwAddVarRW("Specular", TW_TYPE_COLOR4F, &g_MatSpecular, " group='Material' ");
  bar->TwAddVarRW("Shininess",TW_TYPE_FLOAT,&g_MatShininess," group='Material'"
             " min=0 max=128");
  
  bar->TwAddButton("Hide All TW Windows", HideAllTWWindowsCB, this, " group='Global' key=h");

  g_RotateTime = start_time;
  
  // Init rotation
  SetQuaternionFromAxisAngle(axis, angle, g_Rotation);
  SetQuaternionFromAxisAngle(axis, angle, g_RotateStart);

  vertices = new PointMatrixType (0,3);
  faces = new FaceMatrixType (0,3);
  tets = new TetMatrixType(0,3);
  face_colors = new PointMatrixType (0,3);
  vertex_colors = new PointMatrixType (0,3);
  vertex_normals = new PointMatrixType (0,3);
  face_normals = new PointMatrixType (0,3);
  
  face_property = new VectorX (0,1);
  vertex_property = new VectorX (0,1);
  
  

  corner_normals = new PointMatrixType (0,3);
  
  fNormIndices = new FaceMatrixType (0,3);
  
  texCoords = new UVMatrixType (0,2);
  
  fTexIndices = new FaceMatrixType (0,3);
  
  for (unsigned int i = 0; i<PluginManager().plugin_list_.size(); ++i)
    PluginManager().plugin_list_[i]->init(this);

  if (false)// added by wangyu
  {
	  set_color_preset(ColorPresetID::GOLD_ON_WHITE);
  }
}

Preview3D::~Preview3D()
{
  if (vertices)
  {
    delete vertices;
    vertices = 0;
  }
  if (faces)
  {
    delete faces;
    faces = 0;
  }
  if(face_normals)
  {
    delete face_normals;
    face_normals = 0;
  }
  if(vertex_normals)
  {
    delete vertex_normals;
    vertex_normals = 0;
  }
  if(fNormIndices)
  {
    delete fNormIndices;
    fNormIndices = 0;
  }
  if(vertex_colors)
  {
    delete vertex_colors;
    vertex_colors = 0;
  }
  if(face_colors)
  {
    delete face_colors;
    face_colors = 0;
  }
  if(vertex_property)
  {
    delete vertex_property;
    vertex_property = 0;
  }
  if(face_property)
  {
    delete face_property;
    face_property = 0;
  }

  delete serializer;
}








bool Preview3D::load_mesh_from_file(const char* mesh_file_name)
{
  std::string mesh_file_name_string = std::string(mesh_file_name);
  filename = mesh_file_name;
  clear_mesh();
  size_t last_dot = mesh_file_name_string.rfind('.');
  if(last_dot == std::string::npos)
  {
    // No file type determined
    printf("Error: No file extension found in %s\n",mesh_file_name);
    return false;
  }
  std::string extension = mesh_file_name_string.substr(last_dot+1);
  if(extension == "off" || extension =="OFF")
  {
    if(!igl::readOFF(mesh_file_name_string, *vertices, *faces))
    {
      return false;
    }
  }
  else if(extension == "mesh" || extension == "MESH")
  {//added by wangyu
	  if(!igl::readMESH(mesh_file_name_string, *vertices, *tets, *faces))
	  {
		  return false;
	  }
  }
  else if(extension == "obj" || extension =="OBJ")
  {
    if(!(igl::readOBJ(mesh_file_name_string, *vertices, *faces, *corner_normals, *fNormIndices, *texCoords, *fTexIndices)))
    {
      vector< vector< IndexType > > faces_poly;
      if(igl::readOBJPoly(mesh_file_name_string, *vertices, faces_poly, *corner_normals, *fNormIndices, *texCoords, *fTexIndices))
      {
        // Triangulate all faces
        vector<vector<int> > tri;
        vector< Vector3 > tri_normals;
        
        for (unsigned i=0; i<faces_poly.size(); ++i)
        {
          vector<IndexType> f = faces_poly[i];
          int iter = f.size()-2;
          
          Vector3 e1 = vertices->row(f[0+2]) - vertices->row(f[0]);
          Vector3 e2 = vertices->row(f[0+1]) - vertices->row(f[0]);
          Vector3 n = e2.cross(e1);
          n.normalize();
          
          for (int j=0; j<iter; ++j)
          {
            vector<int> t(3);
            t[0] = f[0];
            t[1] = f[1];
            t[2] = f[2];
            f.erase(f.begin()+1);
            tri.push_back(t);
            tri_normals.push_back(n);
          }
          assert(f.size() == 2);
          
        }

        faces->resize(tri.size(),3);
        face_normals->resize(tri.size(),3);
        for (unsigned i=0; i < tri.size();++i)
        {
          (*faces)(i,0) = tri[i][0];
          (*faces)(i,1) = tri[i][1];
          (*faces)(i,2) = tri[i][2];
          face_normals->row(i) = tri_normals[i];
        }
        
        // Add the polygonal edges
//        lines.clear();
        for (unsigned i=0; i<faces_poly.size(); ++i)
        {
          vector<IndexType> f = faces_poly[i];
          for (unsigned j=0; j<f.size(); ++j)
          {
            vector<double> t(9);
            t[0] = (*vertices)(f[j  ],0);
            t[1] = (*vertices)(f[j  ],1);
            t[2] = (*vertices)(f[j  ],2);
            t[3] = (*vertices)(f[(j+1)%f.size()],0);
            t[4] = (*vertices)(f[(j+1)%f.size()],1);
            t[5] = (*vertices)(f[(j+1)%f.size()],2);
            t[6] = 1; t[7] = 0; t[8] = 0;
            lines.push_back(t);
          }
        }
      }
      else
        return false;
    }
  }
  else if (extension == "mp" || extension =="MP")
  {
    delete_on_exit = true;
    MatlabIO mio;
    mio.deserialize(mesh_file_name);
    if (mio.error)
    {
      return false;
    } else
    {
      vertices->resize(mio.V.size(),3);
      for (unsigned int i = 0; i <mio.V.size(); ++i)
        vertices->row(i) << mio.V[i][0], mio.V[i][1], mio.V[i][2];
      
      if (mio.F.size()==0)
        faces->resize(mio.F.size(),3);
      else
        faces->resize(mio.F.size(),mio.F[0].size());
      
      for (unsigned int i = 0; i <mio.F.size(); ++i)
        for(unsigned int j=0;j<mio.F[0].size();++j)
          (*faces)(i,j) = mio.F[i][j];
      
      vertex_colors->resize(mio.VC.size(),3);
      for (unsigned int i = 0; i <mio.VC.size(); ++i)
        vertex_colors->row(i) << mio.VC[i][0], mio.VC[i][1], mio.VC[i][2];
      
      vertex_property->resize(mio.VP.size(),1);
      for (unsigned int i = 0; i <mio.VP.size(); ++i)
        (*vertex_property)(i,0) = mio.VP[i][0];
      
      texCoords->resize(mio.TC.size(),3);
      for (unsigned int i = 0; i <mio.TC.size(); ++i)
        texCoords->row(i) << mio.TC[i][0], mio.TC[i][1];
      
      std::vector<vector<double> > tf = mio.TF;
      if (tf.size())
      {
        fUseTexture.clear();
        fUseTexture.resize(faces->rows(),true);
        for (int fi = 0; fi < faces->rows(); ++fi)
          for(int vit = 0; vit < faces->cols(); ++vit)
          {
            fUseTexture[fi] = fUseTexture[fi] && (tf[(*faces)(fi,vit)][0] >0)   ;
          }
      }
      
      face_colors->resize(mio.FC.size(),3);
      for (unsigned int i = 0; i <mio.FC.size(); ++i)
        face_colors->row(i) << mio.FC[i][0], mio.FC[i][1], mio.FC[i][2];
      
      face_property->resize(mio.FP.size(),1);
      for (unsigned int i = 0; i <mio.FP.size(); ++i)
        (*face_property)(i,0) = mio.FP[i][0];
      
      
      vertex_normals->resize(mio.VN.size(),3);
      for (unsigned int i = 0; i <mio.VN.size(); ++i)
        vertex_normals->row(i) << mio.VN[i][0], mio.VN[i][1], mio.VN[i][2];
      
      face_normals->resize(mio.FN.size(),3);
      for (unsigned int i = 0; i <mio.FN.size(); ++i)
        face_normals->row(i) << mio.FN[i][0], mio.FN[i][1], mio.FN[i][2];
      
      lines = mio.L;
      points = mio.P;
      textp = mio.TEXTP;
      texts = mio.TEXTS;
      
      if (vertex_property->rows() !=0)
      {
        show_isolines = true;
        
      }
      
      
      if (delete_on_exit)
        remove(mesh_file_name);
    }
  }
  
  else
  {
    // unrecognized file type
    printf("Error: %s is not a recognized file type.\n",extension.c_str());
    return false;
  }
  
  //added by wangyu
  rest_vertices = *vertices;

  number_of_vertices = vertices->rows();
  number_of_faces = faces->rows();
  number_of_tets = tets->rows();
  
  show_faces = true;
  normals_type = PER_VERTEX;//wangyu PER_FACE
  
  is_compiled = false;
  get_scale_and_shift_to_fit_mesh(vertices,zoom,g_Translation);
  radius = 1/zoom;

  if (face_normals->rows() == 0 || face_normals->cols() != 3)
    recompute_face_normals();
  
  if (vertex_normals->rows() == 0  || vertex_normals->cols() != 3)
    recompute_vertex_normals();
  
  std::vector<std::vector<IndexType> > vfi;
  //igl::vf(*vertices, *faces, vertex_to_faces, vfi);
  vf(*vertices, *faces, vertex_to_faces, vfi);
  igl::adjacency_list(*faces, vertex_to_vertices);
  if (corner_normals->rows() == 0  || corner_normals->cols() != 3 || fNormIndices->size() != faces->size())
    recompute_corner_normals();
  
  if (face_colors->rows() == 0  || face_colors->cols() != 3)
    compute_face_colors(face_property, face_colors, colorBarType);
  if (vertex_colors->rows() == 0 || vertex_colors->cols() != 3)
    compute_vertex_colors(vertex_property, vertex_colors, colorBarType);
  
  if(vertex_property->rows())
    compute_isolevels();
  
#ifndef PREVIEW3D_NO_SHADERS
  if (face_colors->rows() > 0)
  {
    shader_mode = DIRECTIONAL_PER_PIXEL;
    shader_id = 0;
  }
#endif  
  
  if(fTexIndices->size() != faces->size())
  {
    fTexIndices->resize(faces->rows(),faces->cols());
    for (int fi = 0; fi<faces->rows(); ++fi)
      fTexIndices->row(fi) = faces->row(fi);
    has_wedge_texture = false;
  }
  else
    has_wedge_texture = (*faces != *fTexIndices);
  
  if(texCoords->rows()&&bFlipYCoord)//wangyu
    flipCoord(texCoords,1,texCoords);
  resetTexCoords(!texCoords->rows(), !fUseTexture.size());

  // compute mesh dimension properties
  compute_bounding_box(vertices, aabb_min, aabb_max);
  compute_centroid(vertices, aabb_center);
  diameter = (aabb_min - aabb_max).norm();
  avg_edge_length = 0;
  for(int f=0;f<faces->rows();f++)
  {
    for(int b=0, e=2;b<3;b++,e=b)
    {
      Eigen::Matrix<ScalarType,1,3> begin = vertices->row(faces->row(f)[b]);
      Eigen::Matrix<ScalarType,1,3> end = vertices->row(faces->row(f)[e]);
      avg_edge_length += (begin - end).norm();
    }
  }
  avg_edge_length /= faces->rows()*3;
  
  for (unsigned int i = 0; i<PluginManager().plugin_list_.size(); ++i)
    PluginManager().plugin_list_[i]->init(this);

  return true;
}

bool Preview3D::save_mesh_to_file(const char* mesh_file_name)
{
  std::string mesh_file_name_string(mesh_file_name);
  size_t last_dot = mesh_file_name_string.rfind('.');
  if(last_dot == std::string::npos)
  {
    // No file type determined
    printf("Error: No file extension found in %s\n",mesh_file_name);
    return false;
  }
  std::string extension = mesh_file_name_string.substr(last_dot+1);
  if(extension == "off" || extension =="OFF")
  {
    return igl::writeOFF(mesh_file_name_string,*vertices,*faces);
  }else if(extension == "obj" || extension =="OBJ")
  {
    return igl::writeOBJ(mesh_file_name_string, *vertices, *faces, *corner_normals, *fNormIndices, *texCoords, *fTexIndices);
  }else if(extension == "mesh" || extension =="MESH")//added by wangyu
  {
	  return igl::writeMESH(mesh_file_name_string, *vertices, *tets, *faces);
 
  }else
  {
    // unrecognized file type
    printf("Error: %s is not a recognized file type.\n",extension.c_str());
    return false;
  }
  return true;
}

void Preview3D::recompute_all_normals()
{
  recompute_face_normals();
  recompute_vertex_normals();
  recompute_corner_normals();
}

void Preview3D::recompute_face_normals()
{
  igl::per_face_normals(*vertices, *faces, *face_normals);
}

void Preview3D::recompute_vertex_normals()
{
  igl::per_vertex_normals(*vertices, *faces, per_vertex_normal_type, *face_normals, *vertex_normals);
  invert_normals = test_for_inverted_normals(vertices,vertex_normals);
}

void Preview3D::recompute_corner_normals()
{
  igl::per_corner_normals(*vertices, *faces, *face_normals, vertex_to_faces, corner_threshold, *corner_normals);
  fNormIndices->resize(faces->rows(), faces->cols());
  int index  = 0;
  for (int i = 0; i<faces->rows(); ++i)
    for (int j = 0; j<faces->cols(); ++j)
      (*fNormIndices)(i,j) =index++;
}

void Preview3D::clear_mesh()
{
  *vertices = PointMatrixType (0,3);
  *faces = FaceMatrixType (0,3);
  *tets = TetMatrixType(0,4);//wangyu
  *face_colors = PointMatrixType (0,3);
  *vertex_colors = PointMatrixType (0,3);
  *vertex_normals = PointMatrixType (0,3);
  *face_normals = PointMatrixType (0,3);
  
  *face_property = VectorX (0,1);
  *vertex_property = VectorX (0,1);
  
  *corner_normals = PointMatrixType (0,3);
  
  *fNormIndices = FaceMatrixType (0,3);
  
  *texCoords = UVMatrixType (0,2);
  
  *fTexIndices = FaceMatrixType (0,3);
  
  fUseTexture.clear();
  
  //disabled by wangyu //for (unsigned int i = 0; i<PluginManager().plugin_list_.size(); ++i)
  //  PluginManager().plugin_list_[i]->init(this);
  
  lines.clear();
  points.clear();
  textp.clear();
}

void Preview3D::resetTexCoords(bool reset_uv, bool reset_valid)
{
  if(reset_uv)
  {
    // use vertex and faces as texture coordinates and corner texture indices
    if(!copyXY(vertices,texCoords))
    {
      exit(-1);
    }
    // flip Y coordinate since we usually think of images with reversed Y
    // direction
	if(bFlipYCoord)
		flipCoord(texCoords,1,texCoords);
    normalize(texCoords,texCoords);
    fprintf(stderr,
            "WARNING: texture coordinates not given."
            " Using vertex (X,max(Y)-Y-min(Y)) positions as texture coordinates.\n");
    
    if(!normalized(texCoords))
    {
      normalize(texCoords,texCoords);
      fprintf(stderr,
              "WARNING: texture coordinates are not between 0 and 1."
              " Normalizing them so that they are.\n");
    }
  }
  if(reset_valid)
  {
    fUseTexture.clear();
    fUseTexture.resize(faces->rows(),true);
  }
  is_compiled = false;
}








Eigen::Vector3f Preview3D::screen_to_world(float x, float y, float z)
{
  GLfloat winX, winY;
  Eigen::Vector3d point;

  winX = (float)x;
  winY = (float)m_viewport[3] - (float)y;

  gluUnProject(winX, winY, z, m_modelview_matrix, m_projection_matrix, m_viewport, (GLdouble*)&point[0], (GLdouble*)&point[1], (GLdouble*)&point[2]);

  return Eigen::Vector3f(point[0],point[1],point[2]);
}

void Preview3D::compute_isolevels()
{
  if(vertex_property->rows() == vertices->rows() 
	  && vertex_property->rows()>0 )//wangyu
  {
    glDeleteLists(isolines_display_list, 1);
    
    double min_val = vertex_property->minCoeff();
    double max_val = vertex_property->maxCoeff();
    
    numIsoLevels = std::max(1,std::min(numIsoLevels,5000));
    
    isoLevels.clear();
    isoLevels.resize(numIsoLevels);
    isoSegments.clear();
    isoSegments.resize(numIsoLevels,std::vector< std::pair< isoPoint, isoPoint > > (0));
    
    std::vector< std::vector< double > > isocolors;
    isocolors.resize(numIsoLevels);
    
    for(int i = 0; i < numIsoLevels-1; ++i)
    {
      double val = (1.0 *i) /(numIsoLevels -1);
      isoLevels[i] = min_val + (max_val - min_val) * val;
      isocolors[i].resize(3);
      isocolors[i][0] = 1.;
      isocolors[i][1] = 0.;
      isocolors[i][2] = 0.;
    }
    
    isoLevels[numIsoLevels-1] = max_val;
    isocolors[numIsoLevels-1].resize(3);
    isocolors[numIsoLevels-1][0] = 1.;
    isocolors[numIsoLevels-1][1] = 0.;
    isocolors[numIsoLevels-1][2] = 0.;
    
    for (int i = 0; i < faces->rows(); ++i)
    {
      double t1, t2, p0, p1, p2;
      size_t v0,v1,v2;
      
      for (int j = 0; j<faces->cols(); j++)
      {
        v0 = (*faces)(i,j);
        p0 = (*vertex_property)(v0,0);
        
        v1 = (*faces)(i,(j+1)%(faces->cols()));
        v2 = (*faces)(i,(j-1+faces->cols())%(faces->cols()));
        
        p1 = (*vertex_property)(v1,0);
        p2 = (*vertex_property)(v2,0);
        
        isoPoint iso1, iso2;
        for (int l = 0; l<numIsoLevels; ++l)
        {
          double level = isoLevels[l];
          if ( ((p0 >= level && p1<=level) || (p1 >= level && p0<=level)) && (p0 != p1) )
          {
            t1 = (p1-level) / (p1 - p0);
            vertexPair v = std::make_pair(v0, v1);
            iso1 = std::make_pair(v, t1);
            
            if ( ((p0 >= level && p2<=level) || (p2 >= level && p0<=level)) && (p0 != p2) )
            {
              t2 = (p2-level) / (p2 - p0);
              vertexPair v = std::make_pair(v0, v2);
              iso2 = std::make_pair(v, t2);
              isoSegments[l].push_back(std::make_pair(iso1, iso2));
            }
          }
        }
      }
    }
    
    glDeleteLists(isolines_display_list, 1);
    
    // generate new display list
    isolines_display_list = glGenLists(1);
    glNewList(isolines_display_list, GL_COMPILE);
    float linewidth;
    glGetFloatv(GL_LINE_WIDTH,&linewidth);
    glLineWidth(2);            
    for (int l = 0; l<numIsoLevels; ++l)
    {
      glColor3d(isocolors[l][0], isocolors[l][1], isocolors[l][2]);
      for (unsigned int i = 0; i < isoSegments[l].size(); ++i)
      {
        isoPoint iso1 = isoSegments[l][i].first;
        isoPoint iso2 = isoSegments[l][i].second;
        
        double p1[3], p2[3], t;
        vertexPair vp;
        
        t = iso1.second;
        vp = iso1.first;
        p1[0] = t*(*vertices)(vp.first,0) + (1 - t)*(*vertices)(vp.second,0);
        p1[1] = t*(*vertices)(vp.first,1) + (1 - t)*(*vertices)(vp.second,1);
        p1[2] = t*(*vertices)(vp.first,2) + (1 - t)*(*vertices)(vp.second,2);
        
        t = iso2.second;
        vp = iso2.first;
        p2[0] = t*(*vertices)(vp.first,0) + (1 - t)*(*vertices)(vp.second,0);
        p2[1] = t*(*vertices)(vp.first,1) + (1 - t)*(*vertices)(vp.second,1);
        p2[2] = t*(*vertices)(vp.first,2) + (1 - t)*(*vertices)(vp.second,2);
        
        glBegin(GL_LINES);
        glVertex3dv(p1);
        glVertex3dv(p2);
        glEnd();
      }
    }
    glLineWidth(linewidth);            
    glEndList();
  }
}

bool Preview3D::key_down(unsigned char key, int modifiers, int mouse_x, int mouse_y)
{
  for (unsigned int i = 0; i <PluginManager().plugin_list_.size(); ++i)
    if (PluginManager().plugin_list_[i]->keyDownEvent(key, modifiers, mouse_x, mouse_y))
      return true;
	//wangyu changed this
	//for (unsigned int i = 0; i <PluginManager().plugin_list_.size(); ++i)
	//	PluginManager().plugin_list_[i]->keyDownEvent(key, modifiers, mouse_x, mouse_y);


  
  if (key == '+')//wangyu (key == 's')
  {
    mouse_scroll(mouse_x, mouse_y, 1);
  }
  if (key == '-')//wangyu (key == 'a')
  {
    mouse_scroll(mouse_x, mouse_y, -1);
  }
  if (key == 'w')
  {
	  key_board_x_rotation += 0.1;
  }
  if (key == 's')
  {
	  key_board_x_rotation -= 0.1;
  }
  if (key == 'a')
  {
	  key_board_y_rotation -= 0.1;
  }
  if (key == 'd')
  {
	  key_board_y_rotation += 0.1;
  }
  
  return false;
}

bool Preview3D::key_up(unsigned char /*key*/, int /*modifiers*/, int /*mouse_x*/, int /*mouse_y*/)
{
  return false;
}

#define MOUSE_KEY_EVENT_ALL_RESPONSE false

bool Preview3D::mouse_down(int mouse_x,
                           int mouse_y,
                           int button,
                           int modifiers)
{
  // First pass to AntTweakBar
  if (TwEventMouseButtonGLUT(button, 0, mouse_x, mouse_y))
    return true;

  down = true;
  
#ifdef MAYA_STYLE_CAMERA
  if (modifiers & ALT) {
      down_x = mouse_x;
      down_y = mouse_y;
      mouse_mode =
          button == GLUT_LEFT_BUTTON   ? ROTATION :
          button == GLUT_MIDDLE_BUTTON ? TRANSLATE : ZOOM;
  } else {
	  bool any_true = false;
      for (unsigned int i = 0; i <PluginManager().plugin_list_.size(); ++i)
		  any_true |= PluginManager().plugin_list_[i]->mouseDownEvent(mouse_x, mouse_y, button, modifiers);
      if(any_true) return true; 
	  // wangyu  if (PluginManager().plugin_list_[i]->mouseDownEvent(mouse_x, mouse_y, button, modifiers))
       //      return true;
  }
#else
  switch(button) 
  {
    case GLUT_LEFT_BUTTON :
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
        if(enable_rotation)
        {
          //init track ball
          down_x = mouse_x;
          down_y = mouse_y;
          double coord[3];
          Eigen::RowVector3d center;
          compute_centroid(vertices, center);
          gluProject(center[0], center[1], center[2],m_modelview_matrix, m_projection_matrix,m_viewport,&coord[0],&coord[1],&coord[2]);
          down_z = coord[2];
          down_rotation[0] = g_Rotation[0];
          down_rotation[1] = g_Rotation[1];
          down_rotation[2] = g_Rotation[2];
          down_rotation[3] = g_Rotation[3];
          mouse_mode = ROTATION;
        }
      }
      
      break;
    }    
    
  case GLUT_RIGHT_BUTTON :
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
          down_x = mouse_x;
          down_y = mouse_y;
          double coord[3];
          Eigen::RowVector3d center;
          compute_centroid(vertices, center);
          gluProject(center[0], center[1], center[2],m_modelview_matrix, m_projection_matrix,m_viewport,&coord[0],&coord[1],&coord[2]);
          down_z = coord[2];
          down_translation[0] = g_Translation[0];
          down_translation[1] = g_Translation[1];
          down_translation[2] = g_Translation[2];
          mouse_mode = TRANSLATE;
        }
      break;
    }

  case 3: // MouseWheel up
  {
    mouse_scroll(mouse_x, mouse_y, -1);
  }
  break;

  case 4: // MouseWheel down
  {
    mouse_scroll(mouse_x, mouse_y, 1);
  }
  break;

  }
#endif
  return true;
}

bool Preview3D::mouse_up(int mouse_x, 
                         int mouse_y,
                         int button,
                         int modifiers)
{
  if (TwEventMouseButtonGLUT(button, 1, mouse_x, mouse_y))
    return true;

  down = false;

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
  
  mouse_mode = NOTHING;
  
  return true;
}

bool Preview3D::mouse_move(int mouse_x, int mouse_y)
{
  if(TwEventMouseMotionGLUT(mouse_x,mouse_y))
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
  
  if(down)
  {
    glPushMatrix();
    glScaled(g_Zoom, g_Zoom, g_Zoom);
    
    double model_view_matrix[16];
    double projection_matrix[16];
    int    view_port[4];
    glGetDoublev(GL_MODELVIEW_MATRIX,  model_view_matrix);
    glGetDoublev(GL_PROJECTION_MATRIX, projection_matrix);
    glGetIntegerv(GL_VIEWPORT, view_port);
    double origin_x, origin_y, origin_z;
    gluProject(
               0.,0.,0.,
               model_view_matrix, projection_matrix,
               view_port, &origin_x, &origin_y, &origin_z);
    
    float center_x=0., center_y=0., half_width=0., half_height=0.;
    switch(mouse_mode)
    {
      case ROTATION :
      {
        if(enable_rotation == false)
          return false;

        center_x = origin_x;
        center_y = origin_y;
        
        half_width =  ((float)(view_port[2]))/4.f;
        half_height = ((float)(view_port[3]))/4.f;
        
#ifdef MAYA_STYLE_CAMERA
        float quat_hrz[4], quat_vrt[4];
        float axis_y[3] = {0, 1, 0};
        float axis_x[3] = {1, 0, 0};
        float phi_hrz = 0.5 * 3.14 * (mouse_x - down_x) / half_width;
        float phi_vrt = 0.5 * 3.14 * (mouse_y - down_y) / half_height;
        axis_to_quat(axis_y, phi_hrz, quat_hrz);
        axis_to_quat(axis_x, phi_vrt, quat_vrt);
        add_quats(g_Rotation, quat_hrz, g_Rotation);
        add_quats(g_Rotation, quat_vrt, g_Rotation);
        down_x = mouse_x;
        down_y = mouse_y;
#else
        // Trackball centered at object's centroid
        float new_quaternion[4];
        trackball(new_quaternion,
                  (center_x-down_x)/half_width,
                  (down_y-center_y)/half_height,
                  (center_x-mouse_x)/half_width,
                  (mouse_y-center_y)/half_height);

        // I think we need to do this because we have z pointing out of the
        // screen rather than into the screen
        new_quaternion[2] = -new_quaternion[2];
        add_quats(down_rotation,new_quaternion,g_Rotation);
#endif
        
        Preview3D::snap_to_canonical_quaternion(g_Rotation,trackball_snap_to_canonical);
        
        break;
      }
        
      case TRANSLATE:
      {
        //translation
        GLfloat winX, winY;               // Holds Our X and Y Coordinates
        winX = (float)mouse_x;                  // Holds The Mouse X Coordinate
        winY = (float)mouse_y;
        winY = (float)m_viewport[3] - winY;
        double pos1[3];
        gluUnProject(winX, winY, down_z, m_modelview_matrix,m_projection_matrix,m_viewport, &(pos1[0]),&(pos1[1]),&(pos1[2]));
        
        
        winX = (float)down_x;                  // Holds The Mouse X Coordinate
        winY = (float)down_y;
        winY = (float)m_viewport[3] - winY;
        double pos0[3];
        gluUnProject(winX, winY, down_z, m_modelview_matrix,m_projection_matrix,m_viewport, &(pos0[0]),&(pos0[1]),&(pos0[2]));
        
#ifdef MAYA_STYLE_CAMERA
        g_Translation[0] += 5 * (pos1[0] - pos0[0]);
        g_Translation[1] += 5 * (pos1[1] - pos0[1]);
        g_Translation[2] += 5 * (pos1[2] - pos0[2]);
        down_x = mouse_x;
        down_y = mouse_y;
#else
        g_Translation[0] = down_translation[0] + (pos1[0] - pos0[0]);
        g_Translation[1] = down_translation[1] + (pos1[1] - pos0[1]);
        g_Translation[2] = down_translation[2] + (pos1[2] - pos0[2]);
#endif
        
        
        break;
      }
      case ZOOM:
      {
        float delta = 0.001f * (mouse_x - down_x + mouse_y - down_y);
        g_Zoom *= 1 + delta;
        down_x = mouse_x;
        down_y = mouse_y;
        break;
      }
        
      default:
        break;
    }
    glPopMatrix();
  }
  return true;
}

void Preview3D::drawSelectedFace(int selected_face)
{
  if(selected_face > 0)
  {
    if(use_lighting)
      glEnable(GL_LIGHTING);
    else
      glDisable(GL_LIGHTING);
    
    glEnable(GL_BLEND);
    glEnable(GL_POLYGON_OFFSET_FILL); // Avoid Stitching!
    glDisable(GL_DEPTH_TEST);
    glPolygonOffset(1.0, 1.0);
    //draw mesh faces
    {
      glBegin(GL_TRIANGLES);
      
      glColor3d(0., 0., 0.);
      
      // flat shading
      if(normals_type == PER_FACE)
      {
        const RowVector3 &normal = face_normals->row(selected_face);
        glNormal3f(normal[0], normal[1], normal[2]);
      }
      // loop over vertices in this face
      for(int vit = 0; vit < faces->cols(); ++vit)
      {
        glColor3d(0., 0., 0.);
        // not flat shading goes here
        if(normals_type == PER_VERTEX)
        {
          const RowVector3 &normal = vertex_normals->row((*faces)(selected_face,vit));
          glNormal3f(normal[0], normal[1], normal[2]);
        }
        else
          if (normals_type == PER_CORNER)
          {
            const RowVector3 &normal = corner_normals->row((*fNormIndices)(selected_face,vit));
            glNormal3f(normal[0], normal[1], normal[2]);
          }
        // assumes every point in vertices is length == 3
        const ScalarType *vertex_data = vertices->data() + 3* (*faces)(selected_face,vit);
        glVertex3f(vertex_data[0], vertex_data[1], vertex_data[2]);
        
      }            
      glEnd();
    }
    glDisable(GL_POLYGON_OFFSET_FILL);
    glEnable(GL_DEPTH_TEST);
    
    
#ifndef PREVIEW3D_NO_SHADERS
    glUseProgram(shader_id);
#endif
  }

}






bool Preview3D::mouse_scroll(int mouse_x, int mouse_y, float delta_y)
{
  scroll_position += delta_y;
  bool in_bar = TwMouseMotion(mouse_x,mouse_y);
  if(!TwMouseWheel(scroll_position) && !in_bar)
  {
    for (unsigned int i = 0; i <PluginManager().plugin_list_.size(); ++i)
      if (PluginManager().plugin_list_[i]->mouseScrollEvent(mouse_x, mouse_y, delta_y))
        return true;
    double factor = 0.05f;
    g_Zoom = (g_Zoom + delta_y*factor > 0.1f ? g_Zoom + delta_y*factor : 0.1f);
    return true;
  }
  return true;
}

void Preview3D::draw(int current_time)
{
	if (m_pause)
	{
		return;
	}
	

  // Clear frame buffer
  glClearColor(
               background_color[0],
               background_color[1],
               background_color[2],
               background_color[3]);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  for (unsigned int i = 0; i <PluginManager().plugin_list_.size(); ++i)
    PluginManager().plugin_list_[i]->preDraw(current_time);
  
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);
  glEnable(GL_NORMALIZE);

  // Set light
  if(use_lighting)
  {
    // will be used to set light paramters
    float v[4]; 
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

	glLightModelfv(GL_LIGHT_MODEL_AMBIENT,glLightModel_ambient);
	//glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,1);//added by wangyu

    v[0] = v[1] = v[2] = g_LightMultiplier*0.4f; v[3] = 1.0f;
    glLightfv(GL_LIGHT0, GL_AMBIENT, v);
    v[0] = v[1] = v[2] = g_LightMultiplier*0.8f; v[3] = 1.0f;
    glLightfv(GL_LIGHT0, GL_DIFFUSE, v);
    v[0] = -g_LightDirection[0]; v[1] = -g_LightDirection[1]; v[2] = -g_LightDirection[2]; v[3] = 0.0f;
    glLightfv(GL_LIGHT0, GL_POSITION, v);
  }else
  {
    glDisable(GL_LIGHTING);
  }
  
#ifndef PREVIEW3D_NO_SHADERS
  glUseProgram(shader_id);
#endif  
 
  push_scene(current_time);
  
  // draw grid
  if (draw_grid && (aabb_min.size() == 3))
  {
    glDisable(GL_LIGHTING);
    drawgrid(aabb_min,aabb_max,aabb_center);
    if(use_lighting)
      glEnable(GL_LIGHTING);
  }
  
  // Set material
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, g_MatAmbient);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, g_MatDiffuse);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, g_MatSpecular);
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS,g_MatShininess);
  
  if ( (
#ifndef PREVIEW3D_NO_SHADERS
        shader_mode == OFF &&
#endif        
        (vertex_colors->rows() > 0 || 
         face_colors->rows() > 0 )))
  {
    // Match perpixel color shader which only colors diffuse
    glColorMaterial(GL_FRONT_AND_BACK,GL_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
  }
   
  if (width > 1e-3 && height>1e-3 )
  {
    // need to recompile if flat_shading has changed
    is_compiled = is_compiled &&
    (use_lighting == use_lighting_at_compile) &&
    (normals_type == normals_type_at_compile) &&
    (show_vertid == show_vertid_at_compile) && 
    (show_faceid == show_faceid_at_compile);
    
    if(!is_compiled)
    {
      use_lighting_at_compile   = use_lighting;
      normals_type_at_compile = normals_type;
      show_vertid_at_compile    = show_vertid;
      show_faceid_at_compile    = show_faceid;
      
      Preview3D::compile_mesh();
      Preview3D::compile_overlay();
      is_compiled = true;
    }
    
	//added by wangyu
	if (normals_changed)
	{
		recompute_all_normals();
		normals_changed = false;
	}
	
    Preview3D::draw_mesh();
    if(show_trackball)
      Preview3D::DrawSphereIcon();
  }

  glDisable(GL_COLOR_MATERIAL);
  
  double t = timer->getElapsedTimeInSec();
  fps = 1.0/t;
  timer->start();
  
  pop_scene();
#ifndef PREVIEW3D_NO_SHADERS
  glUseProgram(0);
#endif
  
  for (unsigned int i = 0; i <PluginManager().plugin_list_.size(); ++i)
    PluginManager().plugin_list_[i]->postDraw(current_time);
  
  TwDraw();

  // redraw
  if(enable_autoRefresh)
    glutPostRedisplay();

  //added by wangyu recording
  if (m_recording_screen)
  {
	  char anim_filename[256];
	  sprintf_s(anim_filename, 256, "frame%04d.png", frame_index++);
	  grab_screen(anim_filename);
  }
	
  if (m_record_one_frame)
  {
	  time_t t;
	  time(&t);
	  char anim_filename[256];
	  sprintf_s(anim_filename, 256, "time%d.png", t);
	  grab_screen(anim_filename);

	  m_record_one_frame = false;
  }
}

void Preview3D::drawgrid(const RowVector3 &aabb_min,
                         const RowVector3 &aabb_max,
                         const RowVector3 &aabb_center)
{
  RowVector3 min = aabb_min;
  RowVector3 max = aabb_max;
  
  RowVector3 o;
  o[0] = aabb_center[0];
  o[1] = aabb_min[1] - (max[1]-min[1])/10;
  o[2] = aabb_center[2];
  
  
  double cells = 50;
  double cellSize = (max[1] - min[1])/10;
  
  glColor3f(0.5,0.5,0.5);
  
  // draw axis
  glLineWidth(3.0);
  glBegin(GL_LINES);
  glVertex3f(o[0] - cellSize*cells/2,    o[1] , aabb_center[2]);
  glVertex3f(o[0] + cellSize*cells/2,    o[1] , aabb_center[2]);
  glVertex3f(aabb_center[0], o[1], o[2] - cellSize*cells/2 );
  glVertex3f(aabb_center[0], o[1], o[2] + cellSize*cells/2 );
  glEnd();
  
  
  glLineWidth(1.0);
  // draw grid
  for(int i=0; i<cells; ++i)
  {
    for(int j=0; j<cells; ++j)
    {
      glBegin(GL_LINE_LOOP);
      glVertex3f(o[0] + cellSize*i     -cellSize/2*cells    , o[1], o[2] + cellSize*j     -cellSize/2*cells);
      glVertex3f(o[0] + cellSize*(i+1) -cellSize/2*cells    , o[1], o[2] + cellSize*j     -cellSize/2*cells);
      glVertex3f(o[0] + cellSize*(i+1) -cellSize/2*cells    , o[1], o[2] + cellSize*(j+1) -cellSize/2*cells);
      glVertex3f(o[0] + cellSize*i     -cellSize/2*cells    , o[1], o[2] + cellSize*(j+1) -cellSize/2*cells);
      glEnd();
    }
  }
}

bool Preview3D::saveScene()
{
  char fname[FILE_DIALOG_MAX_BUFFER];
  fname[0] = 0;
  get_save_file_path(fname);
  
  if(fname[0] == 0)
    return false;

  // open new XMLDocument
  tinyxml2::XMLDocument* doc = new tinyxml2::XMLDocument();

  // serialize previously added objects
  serializer->SaveToXMLDoc(doc);

  // serialize AntTweakBar
  igl::save_ReAntTweakBar(bar,doc);

  // serialize objects of all plugins
  for (unsigned int i = 0; i<PluginManager().plugin_list_.size(); ++i)
    PluginManager().plugin_list_[i]->Serialize(doc,NULL);

  // Save doc to xml file
  tinyxml2::XMLError error = doc->SaveFile(fname);
  if(error != tinyxml2::XML_NO_ERROR)
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
  if(error != tinyxml2::XML_NO_ERROR)
  {
    doc->PrintError();
    return false;
  }

  // deserialize previously added objects
  serializer->LoadFromXMLDoc(doc);

  // serialize AntTweakBar
  igl::load_ReAntTweakBar(bar,doc);

  // deserialize objects of all plugins
  for (unsigned int i = 0; i<PluginManager().plugin_list_.size(); ++i)
    PluginManager().plugin_list_[i]->Deserialize(doc,NULL);

  delete doc;
  
  return true;
}

bool Preview3D::saveCamera()
{
  bool ret = false;
  char fname[FILE_DIALOG_MAX_BUFFER];
  fname[0] = 0;
  get_save_file_path(fname);
  
  if(fname[0] == 0)
    return false;
  FILE *f = fopen(fname,"w");
  if(NULL==f)
  {
    printf("IOError: %s could not be opened for reading...",fname);
    return false;                                              
  }
  
  ret= write_camera_to_file(f);
  fclose(f);
  return ret;
}

bool Preview3D::write_camera_to_file(FILE *f)
{
  fprintf(f,"%.10g %.10g %.10g %.10g %.10g %.10g %.10g %.10g %.10g",
          g_Translation[0],
          g_Translation[1],
          g_Translation[2],
          g_Rotation[0],
          g_Rotation[1],
          g_Rotation[2],
          g_Rotation[3],
          g_Zoom,
          zoom);
  return true;
}

void Preview3D::alignCameraCenter()
{
  get_scale_and_shift_to_fit_mesh(vertices,zoom,g_Translation);
}

void Preview3D::set_linewidth(int value)
{
  linewidth = value;
  is_compiled = false;
}

bool Preview3D::loadCamera()
{
  bool ret = false; 

  char fname[FILE_DIALOG_MAX_BUFFER];
  get_open_file_path(fname);

  if(fname[0] == 0)
    return false;
  FILE *f = fopen(fname,"r");
  if(NULL==f)
  {
    printf("IOError: %s could not be opened for reading...",fname);
    return false;                                              
  }
  ret = read_camera_from_file(f);
  fclose(f);
  
  return ret;
}

bool Preview3D::read_camera_from_file(FILE *f)
{
  int count = fscanf(f,"%g %g %g %g %g %g %g %g %g",
                     &(g_Translation[0]),
                     &(g_Translation[1]),
                     &(g_Translation[2]),
                     &(g_Rotation[0]),
                     &(g_Rotation[1]),
                     &(g_Rotation[2]),
                     &(g_Rotation[3]),
                     &(g_Zoom),
                     &(zoom));
  return count == 9;
  
  return true;
}

void Preview3D::push_scene(int current_time)
{

  float mat[4*4]; // rotation matrix
  // Rotate scene
  glPushMatrix();
  // not sure why this was here
  //glTranslatef(0.5f, -0.3f, 0.0f);
  if( g_AutoRotate ) 
  {
    if(auto_rotate_restart)
    {
      g_RotateStart[0] = g_Rotation[0];
      g_RotateStart[1] = g_Rotation[1];
      g_RotateStart[2] = g_Rotation[2];
      g_RotateStart[3] = g_Rotation[3];
      g_RotateTime = current_time;
      auto_rotate_restart = false;
    }
    float axis[3] = { 0, 1, 0 };
    float angle = (float)(current_time-g_RotateTime)/1000.0f;
    float quat[4];
	Preview3D::SetQuaternionFromAxisAngle(axis, angle*g_RotateSpeed, quat);
    Preview3D::MultiplyQuaternions(g_RotateStart, quat, g_Rotation);
  }
  else if(g_KeyBoardRotate)
  {
	  float identity_rotate[4] = {0,0,0,1}; 
	  //added by wangyu key board rotate
	  float axis_x[3] = { 1, 0, 0 };
	  float quat_x[4];
	  Preview3D::SetQuaternionFromAxisAngle(axis_x, key_board_x_rotation, quat_x);
	  Preview3D::MultiplyQuaternions(identity_rotate, quat_x, g_Rotation);
	  float axis_y[3] = { 0, 1, 0 };
	  float quat_y[4];
	  Preview3D::SetQuaternionFromAxisAngle(axis_y, key_board_y_rotation*g_RotateSpeed, quat_y);
	  Preview3D::MultiplyQuaternions(g_Rotation, quat_y, g_Rotation);
  }
  Preview3D::ConvertQuaternionToMatrix(g_Rotation, mat);
  glMultMatrixf(mat);
  glScaled(g_Zoom, g_Zoom, g_Zoom);
  glScaled(zoom, zoom, zoom);
  glTranslatef(g_Translation[0],g_Translation[1],g_Translation[2]);

  glGetIntegerv(GL_VIEWPORT,  m_viewport);
  glGetDoublev(GL_PROJECTION_MATRIX,  m_projection_matrix);
  glGetDoublev(GL_MODELVIEW_MATRIX,  m_modelview_matrix);
  
}

void Preview3D::DrawSphereIcon()
{  
  glPushAttrib (GL_TRANSFORM_BIT |GL_ENABLE_BIT | GL_LINE_BIT | GL_CURRENT_BIT | GL_LIGHTING_BIT);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix ();
  
  //Point3f center = tb->center + tb->track.InverseMatrix()*Point3f(0, 0, 0);
  glTranslatef(-g_Translation[0],-g_Translation[1],-g_Translation[2]);
  glScalef (radius / g_Zoom, radius / g_Zoom, radius / g_Zoom);
  
  float amb[4] = { .3f, .3f, .3f, 1.0f };
  float col[4] = { .5f, .5f, .8f, 1.0f };
  glEnable (GL_LINE_SMOOTH);
  glLineWidth (2);
  glEnable (GL_LIGHTING);
  glEnable (GL_LIGHT0);
  glEnable (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glColor3fv(WHITE);
  glMaterialfv (GL_FRONT_AND_BACK, GL_EMISSION, amb);
  glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, col);

  DrawCircle ();
  glRotatef (90, 1, 0, 0);
  DrawCircle ();
  glRotatef (90, 0, 1, 0);
  DrawCircle ();

  glPopMatrix ();
  glPopAttrib ();
}

void Preview3D::DrawCircle ()
{
  int nside = 100;
  const double pi2 = 3.14159265 * 2.0;
  glBegin (GL_LINE_LOOP);
  for (double i = 0; i < nside; i++) {
    glNormal3d (cos (i * pi2 / nside), sin (i * pi2 / nside), 0.0);
    glVertex3d (cos (i * pi2 / nside), sin (i * pi2 / nside), 0.0);
  }
  glEnd ();
  DrawPlaneHandle ();
}
void Preview3D::DrawPlaneHandle ()
{
  float r = 1.0;
  float dr = r / 10.0f;
  
  glBegin (GL_LINE_STRIP);
  glVertex3f (+r + dr, +r, 0.0);
  glVertex3f (+r, +r + dr, 0.0);
  glVertex3f (+r - dr, +r, 0.0);
  glVertex3f (+r, +r - dr, 0.0);
  glVertex3f (+r + dr, +r, 0.0);
  glEnd ();
  glBegin (GL_LINE_STRIP);
  glVertex3f (-r + dr, -r, 0.0);
  glVertex3f (-r, -r + dr, 0.0);
  glVertex3f (-r - dr, -r, 0.0);
  glVertex3f (-r, -r - dr, 0.0);
  glVertex3f (-r + dr, -r, 0.0);
  glEnd ();
}
void Preview3D::pop_scene()
{
  glPopMatrix();
}

void Preview3D::resize(int w, int h)
{
  width = width_percentage*w;
  height = height_percentage*h;
  m_viewport[2] = width;
  m_viewport[3] = height;
  
  eye[0] = 0;
  eye[1] = 0;
  eye[2] = 5;
  
  center[0] = 0;
  center[1] = 0;
  center[2] = 0;
  
  up[0] = 0;
  up[1] = 1;
  up[2] = 0;
  
  if(useOthographic)
  {
    // Set OpenGL viewport and camera
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    // Set the correct perspective.
    double length = sqrt((eye[0] - center[0]) * (eye[0] - center[0]) + (eye[1] - center[1]) * (eye[1] - center[1]) + (eye[2] - center[2]) * (eye[2] - center[2]) );
    double h = tan(view_angle/360.0 * pi) * (length);
    glOrtho(-h*width/height-frustum_shift_x*length/dnear, h*width/height-frustum_shift_x*length/dnear, -h, h, dnear, dfar);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(eye[0], eye[1], eye[2], center[0], center[1], center[2], up[0], up[1], up[2]);
    
  }
  else
  {
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    //gluPerspective(40, (double)width/height, 1, 100);
    // from: http://nehe.gamedev.net/data/articles/article.asp?article=11
    // shift everything a little to the right
    double fH = tan( view_angle / 360.0 * pi ) * dnear;
    double fW = fH * (double)width/(double)height;
    
    glFrustum( -fW-frustum_shift_x, fW-frustum_shift_x, -fH, fH,dnear, dfar);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(eye[0], eye[1], eye[2], center[0], center[1], center[2], up[0], up[1], up[2]);
  }

  // Send the new window size to AntTweakBar
  barHeight = height-50;
  if(barHeight > maxBarHeight)
  barHeight = maxBarHeight;
  stringstream barSize;
  barSize << " IGLViewer size='" << barWidth << " " << barHeight << "'";
  TwDefine(barSize.str().c_str());
  TwWindowSize(width, height);
}


void Preview3D::SetAutoRotate(bool value)
{
  g_AutoRotate = value; // copy value to g_AutoRotate
  if( g_AutoRotate!=0 ) 
  {
    // init rotation
    //g_RotateTime = glutGet(GLUT_ELAPSED_TIME);
    auto_rotate_restart = true;
   
  enable_autoRefresh = true;

    // make Rotation variable read-only
    TwDefine(" IGLViewer/ObjRotation readonly ");
  }
  else
  {
  enable_autoRefresh = false;
    // make Rotation variable read-write
    TwDefine(" IGLViewer/ObjRotation readwrite ");
  }
}

bool Preview3D::GetAutoRotate()
{
  return g_AutoRotate; // copy g_AutoRotate to value
}

std::string Preview3D::GetTexFilename()
{
  return texFilename;
}

void Preview3D::SetTexFilename(const std::string fname)
{
  texFilename = fname;
  texture_id = texture_from_png(texFilename);//wangyu texture_id = texture_from_tga(texFilename);
  if(texture_id)
  {
    show_texture = true;
    is_compiled = false;
  }
}


bool Preview3D::GetShowTexture()
{
  return show_texture;
}

void Preview3D::SetShowTexture(const bool value)
{
  if(show_texture != value)
  {
    show_texture = value;
    is_compiled = false;
  }
}

void Preview3D::view_xy_plane()
{
  g_Rotation[0] = 0.0f;
  g_Rotation[1] = 0.0f;
  g_Rotation[2] = 0.0f;
  g_Rotation[3] = 1.0f;
  auto_rotate_restart = g_AutoRotate;
}

void Preview3D::view_xz_plane()
{
  g_Rotation[0] = -sqrt(2.0f)/2.0f;
  g_Rotation[1] = 0.0f;
  g_Rotation[2] = 0.0f;
  g_Rotation[3] = sqrt(2.0f)/2.0f;
  auto_rotate_restart = g_AutoRotate;
}

void Preview3D::view_yz_plane()
{
  g_Rotation[0] = -0.5f;
  g_Rotation[1] = -0.5f;
  g_Rotation[2] = -0.5f;
  g_Rotation[3] = 0.5f;
  auto_rotate_restart = g_AutoRotate;
}

void Preview3D::set_color_preset(const ColorPresetID id)
{
  int nid = id;
  if(nid == NUM_COLOR_PRESETS) return;
  if(nid >= NUM_COLOR_PRESETS) nid = 0;
  Preview3D::CopyArray4(color_presets[nid].back, background_color);
  Preview3D::CopyArray4(color_presets[nid].ambient, g_MatAmbient);
  Preview3D::CopyArray4(color_presets[nid].diffuse, g_MatDiffuse);
  Preview3D::CopyArray4(color_presets[nid].specular, g_MatSpecular);
}

ColorPresetID Preview3D::get_color_preset()
{
  ColorPreset c(g_MatAmbient,g_MatDiffuse,g_MatSpecular,background_color);
  // loop over presets and determine if currently using a preset
  int j = 0;
  for(; j<NUM_COLOR_PRESETS; j++)
  {
    if(c == color_presets[j])
    {
      break;
    }
  }
  return (ColorPresetID)j;
}
#ifndef PREVIEW3D_NO_SHADERS
void Preview3D::set_shader_mode(const ShaderMode id)
{
  int nid = id;
  if(nid >= NUM_SHADER_MODE)
    nid = 0;
  shader_mode = id;
  if(texture_id)
    shader_id = 0;
  else
  {
    switch(shader_mode){
      case DIRECTIONAL_PER_PIXEL:
      {
        if(use_lighting)
        {
          if (vertex_colors->rows() > 0)
          {
            shader_id = s_directionalPerPixelColorProgram.p;
          }else
          {
            shader_id = s_directionalPerPixelProgram.p;
          }
        }else
        {
          shader_id = 0;
        }
        break;
      }
      default:
      {
        shader_id = 0;
        break;
      }
    }
  }
}

Preview3D::ShaderMode Preview3D::get_shader_mode()
{
  return shader_mode;
}
#endif


bool Preview3D::snap_to_canonical_quaternion(
                                             float q[4],
                                             const double threshold)
{
  // 0.290019
  // 0.300000
  // 0.400000
  double q_mag = (q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
  if(q_mag == 0)
  {
    printf("ERROR: snap_to_canonical_quaternion passed (0,0,0,0)\n");
    return false;
  }
  
  const double MAX_DISTANCE = 0.5;
  double min_distance = 2*MAX_DISTANCE;
  int min_index = -1;
  int min_sign = 0;
  // loop over canonical view quaternions
  for(int sign = -1;sign<=1;sign+=2)
  {
    for(int i = 0; i<NUM_CANONICAL_VIEW_QUATERNIONS; i++)
    {
      float distance = 0.0;
      // loop over coordinates
      for(int j = 0;j<4;j++)
      {
        distance += 
        (q[j]-sign*CANONICAL_VIEW_QUATERNIONS[i][j])*
        (q[j]-sign*CANONICAL_VIEW_QUATERNIONS[i][j]);
      }
      if(min_distance > distance)
      {
        min_distance = distance;
        min_index = i;
        min_sign = sign;
      }
    }
  }
  
  if(MAX_DISTANCE < min_distance)
  {
    printf("FOUND NEW MAX MIN_DISTANCE: %g\n",min_distance);
  }
  
  if(min_index<0)
  {
    printf("q: %g %g %g %g\n",q[0],q[1],q[2],q[3]);
  }
  assert(min_index >= 0);
  
  //printf("min/max: %g <=? %g\n",(min_distance/MAX_DISTANCE),threshold);
  if( min_distance/MAX_DISTANCE <= threshold)
  {
    // loop over coordinates
    for(int j = 0;j<4;j++)
    {
      q[j] = min_sign*CANONICAL_VIEW_QUATERNIONS[min_index][j];
    }
    return true;
  }
  return false;
}

void Preview3D::set_toggle_ortho(bool value)
{
  if(useOthographic != value){
    useOthographic = value;
    if(useOthographic)
    {
      // Set OpenGL viewport and camera
      glViewport(0, 0, width, height);
      glMatrixMode(GL_PROJECTION);
      glLoadIdentity();
      
      // Set the correct perspective
      // Set the average depth of the scene to the distance between the camera viewpoint and the center of the scene (where the camera looks at.
      double length = sqrt((eye[0] - center[0]) * (eye[0] - center[0]) + (eye[1] - center[1]) * (eye[1] - center[1]) + (eye[2] - center[2]) * (eye[2] - center[2]) );
      //this is basically the same height as the one specified in the perspective case, except now it is scaled by this average depth
      //instead of dnear
      double h = tan(view_angle/360.0 * pi) * (length);
      //we also need to scale the frustum_shift_x to take into account this change
      glOrtho(-h*width/height-frustum_shift_x*length/dnear, h*width/height-frustum_shift_x*length/dnear, -h, h, dnear, dfar);
      
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();
      gluLookAt(eye[0], eye[1], eye[2], center[0], center[1], center[2], up[0], up[1], up[2]);
      
    }
    else
    {
      glViewport(0, 0, width, height);
      glMatrixMode(GL_PROJECTION);
      glLoadIdentity();
      
      double fH = tan( view_angle / 360.0 * pi ) * dnear;
      double fW = fH * (double)width/(double)height;
      
      glFrustum( -fW-frustum_shift_x, fW-frustum_shift_x, -fH, fH,dnear, dfar);
      
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();
      gluLookAt(eye[0], eye[1], eye[2], center[0], center[1], center[2], up[0], up[1], up[2]);
    }
  }
  
}
bool Preview3D::get_toggle_ortho()
{
  return useOthographic;
}
void Preview3D::set_corner_threshold(double value)
{
  corner_threshold = value;
  igl::per_corner_normals(*vertices, *faces, *face_normals, vertex_to_faces, corner_threshold, *corner_normals);
  is_compiled = false;
}
double Preview3D::get_corner_threshold()
{
  return corner_threshold;
}
void Preview3D::set_numIsoLevels(int value)
{
  numIsoLevels = value;
  compute_isolevels();
  is_compiled = false;
}
int Preview3D::get_numIsoLevels()
{
  return numIsoLevels;
}

inline void write_PBS_mesh_attributes(GLuint program, const GLchar* name, const Eigen::MatrixXd & H, const Eigen::MatrixXd & W)
{

}


void Preview3D::draw_mesh()
{
	switch(meshDrawingType)
	{
	case MESH_DRAWING_DEFAULT:
		{
			draw_mesh_default();
			break;
		}
	case MESH_DRAWING_PBS:
		{
			draw_mesh_pbs();
			break;
		}
	}
}













static bool weights_setup = false;


static Weights_shader_info weights_shader_info[NUM_WEIGHTS_SLOTS_IN_SHADER];

static int loc_of_weights_in_shader[NUM_WEIGHTS_SLOTS_IN_SHADER];
static int loc_of_weight_indices_in_shader[NUM_WEIGHTS_SLOTS_IN_SHADER];

static bool buffers_setup = false;
// vertex, color, normal, texture, index
static GLuint m_vbo, m_cbo, m_nbo, m_tbo, m_ibo;


static Weight_gradient weight_gradient[NUM_WEIGHT_GRADIENTS_SLOTS_IN_SHADER];

void Preview3D::draw_mesh_pbs()
{
	GLuint shader_id_for_face_and_line = s_pbsShaderProgram.p;
#ifndef PREVIEW3D_NO_SHADERS
	int shader_id;
	glGetIntegerv(GL_CURRENT_PROGRAM,&shader_id);
	glUseProgram(shader_id_for_face_and_line);//added by wangyu

	//draw_triangle_mesh_or_line_with_shader(
	//	true, 
	//	DeformSkinning::GetReference().W,
	//	DeformSkinning::GetReference().WI,
	//	DeformSkinning::GetReference().sorted_alphaFactors,
	//	DeformSkinning::GetReference().sorted_betaFactors,
	//	shader_id_for_face_and_line,
	//	vertices,
	//	faces,
	//	vertex_normals,
	//	s_pbsShaderProgram,
	//	weight_gradient,
	//	m_projection_matrix,
	//	m_modelview_matrix,
	//	g_MatAmbient,
	//	g_MatDiffuse,
	//	g_LightMultiplier,
	//	g_LightDirection,
	//	linewidth);
	draw_triangle_mesh_or_line_with_shader
	(
		true, 
		DeformSkinning::GetReference().HandlesToViewer(),
		DeformSkinning::GetReference().W,
		DeformSkinning::GetReference().WI,
		DeformSkinning::GetReference().sorted_alphaFactors,
		DeformSkinning::GetReference().sorted_betaFactors,
		shader_id_for_face_and_line,
		vertices,
		faces,
		vertex_normals,
		s_pbsShaderProgram,
		weight_gradient,
		weights_shader_info,
		m_projection_matrix,
		m_modelview_matrix,
		g_MatAmbient,
		g_MatDiffuse,
		g_LightMultiplier,
		g_LightDirection,
		linewidth,
		m_vbo,
		m_nbo,
		m_ibo,
		loc_of_weights_in_shader,
		loc_of_weight_indices_in_shader,
		buffers_setup,
		weights_setup
	);
	glUseProgram(0);
#endif
}

#define PREVIEW3D_NO_SHADERS
void Preview3D::draw_mesh_default()
{
	bool transparent = true;

  if(use_lighting)
    glEnable(GL_LIGHTING);
  else
    glDisable(GL_LIGHTING);
  
  //GLuint shader_id_for_face_and_line = shader_id;//s_pbsShaderProgram.p;//shader_id;//= 0;//addedby wangyu
  //GLuint SHADER_INDEX_OF_WEIGHTS = 4;
  //GLuint SHADER_INDEX_OF_WEIGHT_INDICES = 9;


  if(show_faces)
  {
#ifndef PREVIEW3D_NO_SHADERS
    int shader_id;
    glGetIntegerv(GL_CURRENT_PROGRAM,&shader_id);
	glUseProgram(0);//glUseProgram(shader_id_for_face_and_line);//added by wangyu
#endif

    glEnable(GL_POLYGON_OFFSET_FILL); // Avoid Stitching!
    glPolygonOffset(1.0, 1.0);
    if(texture_id)
      glBindTexture(GL_TEXTURE_2D, texture_id);
    if(use_glCallList)//wangyu 
    {
		glCallList(faces_display_list);
    }
	else
	{
		draw_triangle_mesh_or_line(true);
		//draw_triangle_mesh_or_line_with_shader(true, 
		//		DeformSkinning::GetReference().W,
		//		DeformSkinning::GetReference().WI,
		//		DeformSkinning::GetReference().sorted_alphaFactors,
		//		DeformSkinning::GetReference().sorted_betaFactors,
		//		shader_id_for_face_and_line);
	}
    glDisable(GL_POLYGON_OFFSET_FILL);
#ifndef PREVIEW3D_NO_SHADERS
	// pop current shader, added by wangyu
	glUseProgram(shader_id);
#endif
  }
  
  if(show_lines)
  {
#ifndef PREVIEW3D_NO_SHADERS
    // push current shader
#endif
    if(show_faces)
    {
#ifndef PREVIEW3D_NO_SHADERS
      // turn off shader
		int shader_id;//added by wagnyu
		glGetIntegerv(GL_CURRENT_PROGRAM, &shader_id);//added by wangyu
		glUseProgram(0);//glUseProgram(shader_id_for_face_and_line);//wangyu //glUseProgram(0);
#endif
      glDisable(GL_LIGHTING);
      glColor3fv(line_color);
    }
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	if(use_glCallList)//wangyu 
	{
		glCallList(lines_display_list);//wangyu
	}
	else
	{
		draw_triangle_mesh_or_line(false);
		//draw_triangle_mesh_or_line_with_shader(false,
		//	DeformSkinning::GetReference().W,
		//	DeformSkinning::GetReference().WI,
		//	shader_id_for_face_and_line);
	}
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
#ifndef PREVIEW3D_NO_SHADERS
    // pop current shader
    glUseProgram(shader_id);
#endif
  }
  
  if(show_isolines && vertex_property->rows())
  {
#ifndef PREVIEW3D_NO_SHADERS
    // push current shader
#endif
    if(show_faces)
    {
#ifndef PREVIEW3D_NO_SHADERS
		int shader_id;//added by wagnyu
		glGetIntegerv(GL_CURRENT_PROGRAM, &shader_id);//added by wagnyu
      // turn off shader
      glUseProgram(0);
#endif
      glDisable(GL_LIGHTING);
      glColor3fv(line_color);
    }
    
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	if(use_glCallList)//wangyu 
	{
		glCallList(isolines_display_list);//wangyu
	}
	else
	{
		draw_isolines();
	}
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
#ifndef PREVIEW3D_NO_SHADERS
    // pop current shader
    glUseProgram(shader_id);
    
#endif
  }
  
  if(show_overlay)
  {
#ifndef PREVIEW3D_NO_SHADERS
    glUseProgram(0);
#endif
    
    if (!show_overlay_depth)
      glDisable(GL_DEPTH_TEST);
    
    glDisable(GL_LIGHTING);
    
    
    glCallList(overlay_display_list);
    
    glEnable(GL_DEPTH_TEST);
    if(use_lighting)
    {
      glEnable(GL_LIGHTING);
    }else
    {
      glDisable(GL_LIGHTING);
    }
    
#ifndef PREVIEW3D_NO_SHADERS
    glUseProgram(shader_id);
#endif
    
  }

  if(false)//show_normals
  {
#ifndef PREVIEW3D_NO_SHADERS
	  glUseProgram(0);
#endif

	  draw_normals();

#ifndef PREVIEW3D_NO_SHADERS
	  glUseProgram(shader_id);
#endif
  }

}
#undef PREVIEW3D_NO_SHADERS

void draw_text(double x, double y, double z, string str, bool bsmall) 
{  
  const char *c = str.c_str();
  glRasterPos3f(x, y, z);
  for (; *c != '\0'; c++) {
    glutBitmapCharacter(bsmall ? GLUT_BITMAP_TIMES_ROMAN_10 : GLUT_BITMAP_TIMES_ROMAN_24, *c);
  }
}


void Preview3D::compile_mesh()
{
  if(invert_normals)
  {
    *vertex_normals = -(*vertex_normals);
    *face_normals = -(*face_normals);
    *corner_normals = -(*corner_normals);
    invert_normals = false;
  }

  if(faces->cols() == 3)
  {
    Preview3D::compile_triangle_mesh();
  } else if(faces->cols() == 4)
  {
    Preview3D::compile_quad_mesh();
  } else
  {
    Preview3D::compile_polygon_mesh();
  }
}

void Preview3D::compile_triangle_mesh_aux(bool color)
{
  glLineWidth(linewidth);
  
  
  glBegin(GL_TRIANGLES);
  // loop over all faces
  for(int face_index = 0; face_index<faces->rows();face_index++)
  {
    if(show_texture && texture_id && fUseTexture[face_index])
      continue;
    
    if (color && face_colors->rows() > 0)
    {
      const ScalarType* color_data = face_colors->data() + 3 * face_index;
      glColor3f(color_data[0], color_data[1], color_data[2]);
    }
    // flat shading
    if(normals_type == PER_FACE)
    {
      
      const RowVector3 &normal = face_normals->row(face_index);
      glNormal3f(normal[0], normal[1], normal[2]);
    }
    // loop over vertices in this face
    for(int vit = 0; vit < faces->cols(); ++vit)
    {
      if (color && vertex_colors->rows() > 0)
      {
        const ScalarType* color_data = vertex_colors->data() + 3 * ( (*faces)(face_index,vit) );
        glColor3f(color_data[0], color_data[1], color_data[2]);
      }
      // not flat shading goes here
      if(normals_type == PER_VERTEX)
      {
        const RowVector3 &normal = vertex_normals->row((*faces)(face_index,vit));
        glNormal3f(normal[0], normal[1], normal[2]);
      }
      else
        if (normals_type == PER_CORNER)
        {
          const RowVector3 &normal = corner_normals->row((*fNormIndices)(face_index,vit));
          glNormal3f(normal[0], normal[1], normal[2]);
        }
      // assumes every point in vertices is length == 3
      const ScalarType *vertex_data = vertices->data() + 3* (*faces)(face_index,vit);
      glVertex3f(vertex_data[0], vertex_data[1], vertex_data[2]);
      
    }
  }
  glEnd();
  
  if(!show_texture || !texture_id)
    return;
  
  glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
  glEnable(GL_TEXTURE_2D);
  
  glBegin(GL_TRIANGLES);
  // loop over all faces
  for(int face_index = 0; face_index<faces->rows();face_index++)
  {
    if(!fUseTexture[face_index])
      continue;
    
    if (color && face_colors->rows() > 0)
    {
      const ScalarType* color_data = face_colors->data() + 3 * face_index;
      glColor3f(color_data[0], color_data[1], color_data[2]);
    }
    // flat shading
    if(normals_type == PER_FACE)
    {
      
      const RowVector3 &normal = face_normals->row(face_index);
      glNormal3f(normal[0], normal[1], normal[2]);
    }
    int corner_index = 0;
    // loop over vertices in this face
    for(int vit = 0; vit < faces->cols(); ++vit, ++corner_index)
    {
      if (color && vertex_colors->rows() > 0)
      {
        const ScalarType* color_data = vertex_colors->data() + 3 * ( (*faces)(face_index,vit) );
        glColor3f(color_data[0], color_data[1], color_data[2]);
      }
      // not flat shading goes here
      if(normals_type == PER_VERTEX)
      {
        const RowVector3 &normal = vertex_normals->row((*faces)(face_index,vit));
        glNormal3f(normal[0], normal[1], normal[2]);
      }
      else
        if (normals_type == PER_CORNER)
        {
          const RowVector3 &normal = corner_normals->row((*fNormIndices)(face_index,vit));
          glNormal3f(normal[0], normal[1], normal[2]);
        }
      
      int ti = (*fTexIndices)(face_index,vit);
      double u = (*texCoords)(ti,0);
      double v = (*texCoords)(ti,1);
      glTexCoord2d(u,v);
      // assumes every point in vertices is length == 3
      const ScalarType *vertex_data = vertices->data() + 3* (*faces)(face_index,vit);
      glVertex3f(vertex_data[0], vertex_data[1], vertex_data[2]);
      
    }
  }
  glEnd();
  
  glDisable(GL_TEXTURE_2D);    
  
  glLineWidth(1);
  
}

void Preview3D::compile_triangle_mesh()
{
  // Delete old display list
  glDeleteLists(faces_display_list, 1);
  glDeleteLists(lines_display_list, 1);
  
  // generate new display list
  faces_display_list = glGenLists(1);
  
  glNewList(faces_display_list, GL_COMPILE);
  compile_triangle_mesh_aux(true);
  glEndList();
  
  lines_display_list = glGenLists(1);
  glNewList(lines_display_list, GL_COMPILE);
  compile_triangle_mesh_aux(false);
  glEndList();
  
}

void Preview3D::compile_overlay()
{
  // Delete old display list
  glDeleteLists(overlay_display_list, 1);
  
  // generate new display list
  overlay_display_list = glGenLists(1);
  
  glNewList(overlay_display_list, GL_COMPILE);
  
  // Lines
  glBegin(GL_LINES);
  for(unsigned int i=0; i<lines.size(); ++i)
  {
    vector<double> l = lines[i];
    glColor3d(l[6],l[7],l[8]);
    glVertex3d(l[0],l[1],l[2]);
    glVertex3d(l[3],l[4],l[5]);
  }
  glEnd();
  
  // Points
  for(unsigned int i=0; i<points.size(); ++i)
  {
    vector<double> p = points[i];
    glPointSize(p[3]);
    glColor3d(p[4],p[5],p[6]);
    glBegin(GL_POINTS);
    glVertex3d(p[0],p[1],p[2]);
    glEnd();
  }
  
  // Text
  for(unsigned int i=0; i<textp.size(); ++i)
  {
    vector<double> p = textp[i];
    glColor3f(0,0,0);
    draw_text(p[0],p[1],p[2],texts[i],false);
  }
  
  if(show_vertid)
  {
    glColor3f(0, 0, 0);
    for(int i=0; i<vertices->rows(); ++i)
    {
      draw_text((*vertices)(i,0),(*vertices)(i,1),(*vertices)(i,2),convertInt(i+1),false);
    }
    
  }
  
  if(show_faceid)
  {
    glColor3f(0, 0, 0);
    for(int i=0; i<faces->rows(); ++i)
    {
      RowVector3 p = RowVector3::Zero();
      
      for(int j=0;j<faces->cols();++j)
        p += vertices->row((*faces)(i,j));
      
      p /= faces->cols();
      
      draw_text(p[0],p[1],p[2],convertInt(i+1),false);
    }
    
  }
  
  glEndList();
  
}

void Preview3D::compile_quad_mesh()
{
  // Delete old display list
  glDeleteLists(faces_display_list, 1);
  // generate new display list
  faces_display_list = glGenLists(1);
  
  glNewList(faces_display_list, GL_COMPILE);
  compile_quad_mesh_aux();
  glEndList();
  
  lines_display_list = glGenLists(1);
  glNewList(lines_display_list, GL_COMPILE);
  compile_quad_mesh_aux();
  glEndList();
}

void Preview3D::compile_quad_mesh_aux()
{
  glLineWidth(linewidth);
  
  glBegin(GL_QUADS);
  // loop over all faces
  int face_index = 0;
  for(face_index = 0; face_index<faces->rows();face_index++)
  {
    // flat shading
    if(normals_type == PER_FACE)
    {
      const RowVector3 &normal = face_normals->row(face_index);
      glNormal3f(normal[0], normal[1], normal[2]);
    }
    {
      // loop over vertices in this face
      for(int vit = 0; vit < faces->cols(); ++vit)
      {
        // not flat shading goes here
        if(normals_type == PER_VERTEX)
        {
          const RowVector3 &normal = (invert_normals ? -1 : 1) * vertex_normals->row((*faces)(face_index,vit));
          glNormal3f(normal[0], normal[1], normal[2]);
        }
        else
          if (normals_type == PER_CORNER)
          {
            const RowVector3 &normal = corner_normals->row((*fNormIndices)(face_index,vit));
            glNormal3f(normal[0], normal[1], normal[2]);
          }
        // assumes every point in vertices is length == 3
        const ScalarType *vertex_data = vertices->data() + 3* (*faces)(face_index,vit);
        glVertex3f(vertex_data[0], vertex_data[1], vertex_data[2]);
        
      }
    }
  }
  glEnd();
  glLineWidth(1);
  
}

void Preview3D::compile_polygon_mesh()
{
  // Delete old display list
  glDeleteLists(faces_display_list, 1);
  // generate new display list
  faces_display_list = glGenLists(1);
  
  glNewList(faces_display_list, GL_COMPILE);
  compile_polygon_mesh_aux();
  glEndList();
  
  lines_display_list = glGenLists(1);
  glNewList(lines_display_list, GL_COMPILE);
  compile_polygon_mesh_aux();
  glEndList();
}

void Preview3D::compile_polygon_mesh_aux()
{
  glLineWidth(linewidth);
  
  // loop over all faces
  int face_index = 0;
  for(face_index = 0; face_index<faces->rows();face_index++)
  {
    // flat shading
    if(normals_type == PER_FACE)
    {
      const RowVector3 &normal = face_normals->row(face_index);
      glNormal3f(normal[0], normal[1], normal[2]);
    }
    {
      glBegin(GL_POLYGON);
      // loop over vertices in this face
      for(int vit = 0; vit < faces->cols(); ++vit)
      {
        // not flat shading goes here
        if(normals_type == PER_VERTEX)
        {
          const RowVector3 &normal = vertex_normals->row((*faces)(face_index,vit));
          glNormal3f(normal[0], normal[1], normal[2]);
        }
        else
          if (normals_type == PER_CORNER)
          {
            const RowVector3 &normal = corner_normals->row((*fNormIndices)(face_index,vit));
            glNormal3f(normal[0], normal[1], normal[2]);
          }
        
        // assumes every point in vertices is length == 3
        const ScalarType *vertex_data = vertices->data() + 3* (*faces)(face_index,vit);
        glVertex3f(vertex_data[0], vertex_data[1], vertex_data[2]);
        
      }
      glEnd();
    }
  }
  glLineWidth(1);
}

void Preview3D::compute_vertex_colors(
                                      const VectorX *vertex_property, 
                                      PointMatrixType *vertex_colors,
									  const ColorBarType colorBarType)
{
	per_attrib_colors(vertex_property,vertex_colors, colorBarType);
}
void Preview3D::compute_face_colors(
                                    const VectorX *face_property, 
                                    PointMatrixType * face_colors,
									const ColorBarType colorBarType)
{
	per_attrib_colors(face_property,face_colors,colorBarType);
}


void Preview3D::compute_bounding_box(
                                                  const PointMatrixType *vertices, 
                                                  RowVector3 & min_point,
                                                  RowVector3 & max_point)
{
  min_point = vertices->colwise().minCoeff();
  max_point = vertices->colwise().maxCoeff();
}

void Preview3D::compute_centroid(
                                                  const PointMatrixType *vertices,
                                                  RowVector3 & centroid)
{
  centroid = vertices->colwise().sum()/vertices->rows();
}


void Preview3D::get_scale_and_shift_to_fit_mesh( 
                                                const PointMatrixType *vertices, 
                                                float& zoom,
                                                Eigen::Vector3f& shift)
{
  //Compute mesh centroid
  RowVector3 centroid;
  RowVector3 min_point;
  RowVector3 max_point;
  compute_bounding_box(vertices, min_point, max_point);
  compute_centroid(vertices, centroid);

  shift[0] = -centroid[0];
  shift[1] = -centroid[1];
  shift[2] = -centroid[2];
  double x_scale = fabs(max_point[0] - min_point[0]);
  double y_scale = fabs(max_point[1] - min_point[1]);
  double z_scale = fabs(max_point[2] - min_point[2]);
  zoom = 2.0/ std::max(z_scale,std::max(x_scale,y_scale));
}

bool Preview3D::test_for_inverted_normals(
                                          const PointMatrixType *vertices, 
                                          const PointMatrixType *vertex_normals)
{
  RowVector3 centroid;
  RowVector3 min_point;
  RowVector3 max_point;
  compute_bounding_box(vertices, min_point, max_point);
  compute_centroid(vertices, centroid);
  
  double average_dot_product = 0.0;
  // loop over vertices
  for( int vi = 0; vi < vertices->rows(); ++vi)
  {  
  // take dot product of unit displacement vector and normal
    RowVector3 unit_displacement = vertices->row(vi) - centroid;
    unit_displacement.normalize();

    double dot_product = vertex_normals->row(vi).dot(unit_displacement);
    average_dot_product += dot_product;
  }
  return average_dot_product < 0;
}













// Routine to set a quaternion from a rotation axis and angle
// ( input axis = float[3] angle = float  output: quat = float[4] )
void Preview3D::SetQuaternionFromAxisAngle(const float *axis, float angle, float *quat)
{
  float sina2, norm;
  sina2 = (float)sin(0.5f * angle);
  norm = (float)sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
  quat[0] = sina2 * axis[0] / norm;
  quat[1] = sina2 * axis[1] / norm;
  quat[2] = sina2 * axis[2] / norm;
  quat[3] = (float)cos(0.5f * angle);
  
}

// Routine to convert a quaternion to a 4x4 matrix
// ( input: quat = float[4]  output: mat = float[4*4] )
void Preview3D::ConvertQuaternionToMatrix(const float *quat, float *mat)
{
  float yy2 = 2.0f * quat[1] * quat[1];
  float xy2 = 2.0f * quat[0] * quat[1];
  float xz2 = 2.0f * quat[0] * quat[2];
  float yz2 = 2.0f * quat[1] * quat[2];
  float zz2 = 2.0f * quat[2] * quat[2];
  float wz2 = 2.0f * quat[3] * quat[2];
  float wy2 = 2.0f * quat[3] * quat[1];
  float wx2 = 2.0f * quat[3] * quat[0];
  float xx2 = 2.0f * quat[0] * quat[0];
  mat[0*4+0] = - yy2 - zz2 + 1.0f;
  mat[0*4+1] = xy2 + wz2;
  mat[0*4+2] = xz2 - wy2;
  mat[0*4+3] = 0;
  mat[1*4+0] = xy2 - wz2;
  mat[1*4+1] = - xx2 - zz2 + 1.0f;
  mat[1*4+2] = yz2 + wx2;
  mat[1*4+3] = 0;
  mat[2*4+0] = xz2 + wy2;
  mat[2*4+1] = yz2 - wx2;
  mat[2*4+2] = - xx2 - yy2 + 1.0f;
  mat[2*4+3] = 0;
  mat[3*4+0] = mat[3*4+1] = mat[3*4+2] = 0;
  mat[3*4+3] = 1;
}

// Routine to multiply 2 quaternions (ie, compose rotations)
// ( input q1 = float[4] q2 = float[4]  output: qout = float[4] )
void Preview3D::MultiplyQuaternions(const float *q1, const float *q2, float *qout)
{
  float qr[4];
  qr[0] = q1[3]*q2[0] + q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1];
  qr[1] = q1[3]*q2[1] + q1[1]*q2[3] + q1[2]*q2[0] - q1[0]*q2[2];
  qr[2] = q1[3]*q2[2] + q1[2]*q2[3] + q1[0]*q2[1] - q1[1]*q2[0];
  qr[3]  = q1[3]*q2[3] - (q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2]);
  qout[0] = qr[0]; qout[1] = qr[1]; qout[2] = qr[2]; qout[3] = qr[3];
}


//  Callback function called when the 'AutoRotate' variable value of the tweak bar has changed
void TW_CALL Preview3D::SetAutoRotateCB(const void *value, void *clientData)
{
  static_cast<Preview3D *>(clientData)->SetAutoRotate(*static_cast<const bool *>(value));
}
void TW_CALL Preview3D::GetAutoRotateCB(void *value, void *clientData)
{
  *static_cast<bool *>(value) = static_cast<Preview3D*>(clientData)->GetAutoRotate();
}

void TW_CALL Preview3D::SetShowTextureCB(const void *value, void *clientData)
{
  static_cast<Preview3D *>(clientData)->SetShowTexture(*static_cast<const bool *>(value));
}
void TW_CALL Preview3D::GetShowTextureCB(void *value, void *clientData)
{
  *static_cast<bool *>(value) = static_cast<Preview3D*>(clientData)->GetShowTexture();
}

void TW_CALL Preview3D::set_color_presetCB(const void *value, void *clientData)
{
  static_cast<Preview3D *>(clientData)->set_color_preset(*static_cast<const ColorPresetID *>(value));
}
void TW_CALL Preview3D::get_color_presetCB(void *value, void *clientData)
{
  *static_cast<ColorPresetID *>(value) = static_cast<Preview3D*>(clientData)->get_color_preset();
}

#ifndef PREVIEW3D_NO_SHADERS
void TW_CALL Preview3D::set_shader_modeCB(const void *value, void *clientData)
{
  static_cast<Preview3D *>(clientData)->set_shader_mode(*static_cast<const ShaderMode *>(value));
}
void TW_CALL Preview3D::get_shader_modeCB(void *value, void *clientData)
{
  *static_cast<Preview3D::ShaderMode *>(value) = static_cast<Preview3D*>(clientData)->get_shader_mode();
}
#endif

void TW_CALL Preview3D::view_xy_planeCB(void *clientData)
{
  static_cast<Preview3D *>(clientData)->view_xy_plane();
}
void TW_CALL Preview3D::view_xz_planeCB(void *clientData)
{
  static_cast<Preview3D *>(clientData)->view_xz_plane();
}
void TW_CALL Preview3D::view_yz_planeCB(void *clientData)
{
  static_cast<Preview3D *>(clientData)->view_yz_plane();
}
void TW_CALL Preview3D::snap_to_canonical_quaternionCB(void *clientData)
{
  Preview3D::snap_to_canonical_quaternion(static_cast<Preview3D *>(clientData)->g_Rotation);
}
void TW_CALL Preview3D::invert_normalsCB(void *clientData)
{
  Preview3D *p3d = static_cast<Preview3D *>(clientData);
  p3d->invert_normals = !p3d->invert_normals ;
  p3d->is_compiled = false;
}
void TW_CALL Preview3D::get_toggle_orthoCB(void *value, void *clientData)
{
  *static_cast<bool *>(value) = static_cast<Preview3D *>(clientData)->get_toggle_ortho();
}
void TW_CALL Preview3D::set_toggle_orthoCB(const void *value, void *clientData)
{
  static_cast<Preview3D *>(clientData)->set_toggle_ortho(*static_cast<const bool *>(value));
}
void TW_CALL Preview3D::alignCameraCenterCB(void *clientData)
{
  static_cast<Preview3D *>(clientData)->alignCameraCenter();
}
void TW_CALL Preview3D::get_corner_thresholdCB(void *value, void *clientData)
{
  *static_cast<double *>(value) = static_cast<Preview3D *>(clientData)->get_corner_threshold();
}
void TW_CALL Preview3D::set_corner_thresholdCB(const void *value, void *clientData)
{
  static_cast<Preview3D *>(clientData)->set_corner_threshold(*static_cast<const double *>(value));
}
void TW_CALL Preview3D::get_numIsoLevelsCB(void *value, void *clientData)
{
  *static_cast<int *>(value) = static_cast<Preview3D *>(clientData)->get_numIsoLevels();
}
void TW_CALL Preview3D::set_numIsoLevelsCB(const void *value, void *clientData)
{
  static_cast<Preview3D *>(clientData)->set_numIsoLevels(*static_cast<const int *>(value));
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

void TW_CALL Preview3D::SaveCameraCB(void *clientData)
{
	static_cast<Preview3D *>(clientData)->saveCamera();
}

void TW_CALL Preview3D::LoadCameraCB(void *clientData)
{
	static_cast<Preview3D *>(clientData)->loadCamera();
}

void TW_CALL Preview3D::SetLineWidthCB(const void *value, void *clientData)
{
  static_cast<Preview3D *>(clientData)->set_linewidth(*static_cast<const int *>(value));
}

void TW_CALL Preview3D::GetLineWidthCB(void *value, void *clientData)
{
  *static_cast<int *>(value) = static_cast<Preview3D *>(clientData)->linewidth;
}

std::string Preview3D::convertInt(int number)
{
  std::stringstream ss;//create a stringstream
  ss << number;//add number to the stream
  return ss.str();//return a string with the contents of the stream
}

std::string Preview3D::convertDouble(double number)
{
  std::stringstream ss;//create a stringstream
  ss << std::setprecision(4) << number;//add number to the stream
  return ss.str();//return a string with the contents of the stream
}

void TW_CALL Preview3D::open_dialog_texture(void *clientData)
{
  char fname[2048];
  
  fname[0] = 0;

  get_open_file_path(fname);
  
  if(fname[0] == 0)
    return;
  
  static_cast<Preview3D *>(clientData)->SetTexFilename(fname);
  
}

void TW_CALL Preview3D::open_dialog_mesh(void *clientData)
{
  char fname[2048];
  
  fname[0] = 0;
  get_open_file_path(fname);
  
  if(fname[0] == 0)
    return;
  
  bool bLoad = static_cast<Preview3D *>(clientData)->load_mesh_from_file(fname);
  if(bLoad)//wangyu
  {
	  static_cast<Preview3D *>(clientData)->copy_working_folder(fname);
  }
}

/*******These are added by wangyu***/ 
void TW_CALL Preview3D::save_dialog_mesh(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_save_file_path(fname);

	if(fname[0] == 0)
		return;
	static_cast<Preview3D *>(clientData)->save_mesh_to_file(fname);
}

void TW_CALL Preview3D::save_dialog_tets(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_save_file_path(fname);

	if(fname[0] == 0)
		return;
	static_cast<Preview3D *>(clientData)->save_tets_to_file(fname);
}

void Preview3D::save_tets_to_file(const char* mesh_file_name)
{
	igl::writeDMAT(mesh_file_name,*tets);
}

/*********************************/

/****************Added by wangyu*********************/
void Preview3D::copy_working_folder(const char *working_folder)
{
	strcpy(current_working_folder,working_folder);
}

void TW_CALL Preview3D::SendStatusToMatlabCB(void *clientData)
{
	static_cast<Preview3D *>(clientData)->send_status_to_matlab();
}

void TW_CALL Preview3D::open_dialog_pose_mesh(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_open_file_path(fname);

	if(fname[0] == 0)
		return;

	static_cast<Preview3D *>(clientData)->load_pose_mesh_from_file(fname);

}

bool Preview3D::load_pose_mesh_from_file(const char* mesh_file_name)
{
	std::string mesh_file_name_string = std::string(mesh_file_name);
	filename = mesh_file_name;
	size_t last_dot = mesh_file_name_string.rfind('.');
	if(last_dot == std::string::npos)
	{
		// No file type determined
		printf("Error: No file extension found in %s\n",mesh_file_name);
		return false;
	}
	std::string extension = mesh_file_name_string.substr(last_dot+1);
	if(extension == "off" || extension =="OFF")
	{
		if(!igl::readOFF(mesh_file_name_string, *vertices, *faces))
		{
			return false;
		}
	}else if(extension == "obj" || extension =="OBJ")
	{
		if(!(igl::readOBJ(mesh_file_name_string, *vertices, *faces, *corner_normals, *fNormIndices, *texCoords, *fTexIndices)))
		{
			vector< vector< IndexType > > faces_poly;
			if(igl::readOBJPoly(mesh_file_name_string, *vertices, faces_poly, *corner_normals, *fNormIndices, *texCoords, *fTexIndices))
			{
				// Triangulate all faces
				vector<vector<int> > tri;
				vector< Vector3 > tri_normals;

				for (unsigned i=0; i<faces_poly.size(); ++i)
				{
					vector<IndexType> f = faces_poly[i];
					int iter = f.size()-2;

					Vector3 e1 = vertices->row(f[0+2]) - vertices->row(f[0]);
					Vector3 e2 = vertices->row(f[0+1]) - vertices->row(f[0]);
					Vector3 n = e2.cross(e1);
					n.normalize();

					for (int j=0; j<iter; ++j)
					{
						vector<int> t(3);
						t[0] = f[0];
						t[1] = f[1];
						t[2] = f[2];
						f.erase(f.begin()+1);
						tri.push_back(t);
						tri_normals.push_back(n);
					}
					assert(f.size() == 2);

				}

				faces->resize(tri.size(),3);
				face_normals->resize(tri.size(),3);
				for (unsigned i=0; i < tri.size();++i)
				{
					(*faces)(i,0) = tri[i][0];
					(*faces)(i,1) = tri[i][1];
					(*faces)(i,2) = tri[i][2];
					face_normals->row(i) = tri_normals[i];
				}

				// Add the polygonal edges
				//        lines.clear();
				for (unsigned i=0; i<faces_poly.size(); ++i)
				{
					vector<IndexType> f = faces_poly[i];
					for (unsigned j=0; j<f.size(); ++j)
					{
						vector<double> t(9);
						t[0] = (*vertices)(f[j  ],0);
						t[1] = (*vertices)(f[j  ],1);
						t[2] = (*vertices)(f[j  ],2);
						t[3] = (*vertices)(f[(j+1)%f.size()],0);
						t[4] = (*vertices)(f[(j+1)%f.size()],1);
						t[5] = (*vertices)(f[(j+1)%f.size()],2);
						t[6] = 1; t[7] = 0; t[8] = 0;
						lines.push_back(t);
					}
				}
			}
			else
				return false;
		}
	}

	else
	{
		// unrecognized file type
		printf("Error: %s is not a recognized file type.\n",extension.c_str());
		return false;
	}

//	number_of_vertices = vertices->rows();
//	number_of_faces = faces->rows();
//
//	show_faces = true;
//	normals_type = PER_VERTEX;//wangyu PER_FACE
//
	is_compiled = false;
//	get_scale_and_shift_to_fit_mesh(vertices,zoom,g_Translation);
//	radius = 1/zoom;
//
//	if (face_normals->rows() == 0 || face_normals->cols() != 3)
//		recompute_face_normals();
//
//	if (vertex_normals->rows() == 0  || vertex_normals->cols() != 3)
//		recompute_vertex_normals();
//
//	std::vector<std::vector<IndexType> > vfi;
//	igl::vf(*vertices, *faces, vertex_to_faces, vfi);
//	igl::adjacency_list(*faces, vertex_to_vertices);
//	if (corner_normals->rows() == 0  || corner_normals->cols() != 3 || fNormIndices->size() != faces->size())
//		recompute_corner_normals();
//
//	if (face_colors->rows() == 0  || face_colors->cols() != 3)
//		compute_face_colors(face_property, face_colors);
//	if (vertex_colors->rows() == 0 || vertex_colors->cols() != 3)
//		compute_vertex_colors(vertex_property, vertex_colors);
//
//	if(vertex_property->rows())
//		compute_isolevels();
//
//#ifndef PREVIEW3D_NO_SHADERS
//	if (face_colors->rows() > 0)
//	{
//		shader_mode = OFF;
//		shader_id = 0;
//	}
//#endif  
//
//	if(fTexIndices->size() != faces->size())
//	{
//		fTexIndices->resize(faces->rows(),faces->cols());
//		for (int fi = 0; fi<faces->rows(); ++fi)
//			fTexIndices->row(fi) = faces->row(fi);
//		has_wedge_texture = false;
//	}
//	else
//		has_wedge_texture = (*faces != *fTexIndices);
//
//	if(texCoords->rows()&&bFlipYCoord)//wangyu
//		flipCoord(texCoords,1,texCoords);
//	resetTexCoords(!texCoords->rows(), !fUseTexture.size());
//
//	// compute mesh dimension properties
//	compute_bounding_box(vertices, aabb_min, aabb_max);
//	compute_centroid(vertices, aabb_center);
//	diameter = (aabb_min - aabb_max).norm();
//	avg_edge_length = 0;
//	for(int f=0;f<faces->rows();f++)
//	{
//		for(int b=0, e=2;b<3;b++,e=b)
//		{
//			Eigen::Matrix<ScalarType,1,3> begin = vertices->row(faces->row(f)[b]);
//			Eigen::Matrix<ScalarType,1,3> end = vertices->row(faces->row(f)[e]);
//			avg_edge_length += (begin - end).norm();
//		}
//	}
//	avg_edge_length /= faces->rows()*3;
//
//	for (unsigned int i = 0; i<PluginManager().plugin_list_.size(); ++i)
//		PluginManager().plugin_list_[i]->init(this);

	return true;
}

void TW_CALL Preview3D::compile_dialog_mesh(void *clientData)
{
	static_cast<Preview3D *>(clientData)->compile_mesh();
}

void Preview3D::update_vertices_in_GL()
{

	// Delete old display list
	//glDeleteLists(faces_display_list, 1);
	//glDeleteLists(lines_display_list, 1);

	// generate new display list
	//faces_display_list = glGenLists(1);

	//glNewList(faces_display_list, GL_COMPILE);
	compile_mesh_vertices();
	//glEndList();

	//lines_display_list = glGenLists(1);
	glNewList(lines_display_list, GL_COMPILE);
	compile_mesh_vertices();
	glEndList();
	//bool color = false;

	//glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	//glEnable(GL_TEXTURE_2D);

	//glBegin(GL_TRIANGLES);
	//// loop over all faces
	//for(int face_index = 0; face_index<faces->rows();face_index++)
	//{
	//	if(!fUseTexture[face_index])
	//		continue;

	//	if (color && face_colors->rows() > 0)
	//	{
	//		const ScalarType* color_data = face_colors->data() + 3 * face_index;
	//		glColor3f(color_data[0], color_data[1], color_data[2]);
	//	}
	//	// flat shading
	//	if(normals_type == PER_FACE)
	//	{

	//		const RowVector3 &normal = face_normals->row(face_index);
	//		glNormal3f(normal[0], normal[1], normal[2]);
	//	}
	//	int corner_index = 0;
	//	// loop over vertices in this face
	//	for(int vit = 0; vit < faces->cols(); ++vit, ++corner_index)
	//	{
	//		if (color && vertex_colors->rows() > 0)
	//		{
	//			const ScalarType* color_data = vertex_colors->data() + 3 * ( (*faces)(face_index,vit) );
	//			glColor3f(color_data[0], color_data[1], color_data[2]);
	//		}
	//		// not flat shading goes here
	//		if(normals_type == PER_VERTEX)
	//		{
	//			const RowVector3 &normal = vertex_normals->row((*faces)(face_index,vit));
	//			glNormal3f(normal[0], normal[1], normal[2]);
	//		}
	//		else
	//			if (normals_type == PER_CORNER)
	//			{
	//				const RowVector3 &normal = corner_normals->row((*fNormIndices)(face_index,vit));
	//				glNormal3f(normal[0], normal[1], normal[2]);
	//			}

	//			int ti = (*fTexIndices)(face_index,vit);
	//			double u = (*texCoords)(ti,0);
	//			double v = (*texCoords)(ti,1);
	//			glTexCoord2d(u,v);
	//			// assumes every point in vertices is length == 3
	//			const ScalarType *vertex_data = vertices->data() + 3* (*faces)(face_index,vit);
	//			glVertex3f(vertex_data[0], vertex_data[1], vertex_data[2]);

	//	}
	//}
	//glEnd();

	//glDisable(GL_TEXTURE_2D);  
}

void Preview3D::compile_mesh_vertices()
{
	bool color = false;

	glLineWidth(linewidth);


	glBegin(GL_TRIANGLES);
	// loop over all faces
	for(int face_index = 0; face_index<faces->rows();face_index++)
	{
		if(show_texture && texture_id && fUseTexture[face_index])
			continue;

		//if (color && face_colors->rows() > 0)
		//{
		//	const ScalarType* color_data = face_colors->data() + 3 * face_index;
		//	glColor3f(color_data[0], color_data[1], color_data[2]);
		//}
		//// flat shading
		//if(normals_type == PER_FACE)
		//{

		//	const RowVector3 &normal = face_normals->row(face_index);
		//	glNormal3f(normal[0], normal[1], normal[2]);
		//}
		// loop over vertices in this face
		for(int vit = 0; vit < faces->cols(); ++vit)
		{
			//if (color && vertex_colors->rows() > 0)
			//{
			//	const ScalarType* color_data = vertex_colors->data() + 3 * ( (*faces)(face_index,vit) );
			//	glColor3f(color_data[0], color_data[1], color_data[2]);
			//}
			//// not flat shading goes here
			//if(normals_type == PER_VERTEX)
			//{
			//	const RowVector3 &normal = vertex_normals->row((*faces)(face_index,vit));
			//	glNormal3f(normal[0], normal[1], normal[2]);
			//}
			//else
			//	if (normals_type == PER_CORNER)
			//	{
			//		const RowVector3 &normal = corner_normals->row((*fNormIndices)(face_index,vit));
			//		glNormal3f(normal[0], normal[1], normal[2]);
			//	}
			//	// assumes every point in vertices is length == 3
				const ScalarType *vertex_data = vertices->data() + 3* (*faces)(face_index,vit);
				glVertex3f(vertex_data[0], vertex_data[1], vertex_data[2]);

		}
	}
	glEnd();

	if(!show_texture || !texture_id)
		return;

	glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glEnable(GL_TEXTURE_2D);

	glBegin(GL_TRIANGLES);
	// loop over all faces
	//for(int face_index = 0; face_index<faces->rows();face_index++)
	//{
	//	if(!fUseTexture[face_index])
	//		continue;

	//	if (color && face_colors->rows() > 0)
	//	{
	//		const ScalarType* color_data = face_colors->data() + 3 * face_index;
	//		glColor3f(color_data[0], color_data[1], color_data[2]);
	//	}
	//	// flat shading
	//	if(normals_type == PER_FACE)
	//	{

	//		const RowVector3 &normal = face_normals->row(face_index);
	//		glNormal3f(normal[0], normal[1], normal[2]);
	//	}
	//	int corner_index = 0;
	//	// loop over vertices in this face
	//	for(int vit = 0; vit < faces->cols(); ++vit, ++corner_index)
	//	{
	//		if (color && vertex_colors->rows() > 0)
	//		{
	//			const ScalarType* color_data = vertex_colors->data() + 3 * ( (*faces)(face_index,vit) );
	//			glColor3f(color_data[0], color_data[1], color_data[2]);
	//		}
	//		// not flat shading goes here
	//		if(normals_type == PER_VERTEX)
	//		{
	//			const RowVector3 &normal = vertex_normals->row((*faces)(face_index,vit));
	//			glNormal3f(normal[0], normal[1], normal[2]);
	//		}
	//		else
	//			if (normals_type == PER_CORNER)
	//			{
	//				const RowVector3 &normal = corner_normals->row((*fNormIndices)(face_index,vit));
	//				glNormal3f(normal[0], normal[1], normal[2]);
	//			}

	//			int ti = (*fTexIndices)(face_index,vit);
	//			double u = (*texCoords)(ti,0);
	//			double v = (*texCoords)(ti,1);
	//			glTexCoord2d(u,v);
	//			// assumes every point in vertices is length == 3
	//			const ScalarType *vertex_data = vertices->data() + 3* (*faces)(face_index,vit);
	//			glVertex3f(vertex_data[0], vertex_data[1], vertex_data[2]);

	//	}
	//}
	glEnd();

	glDisable(GL_TEXTURE_2D);    

	glLineWidth(1);

}

void Preview3D::draw_triangle_mesh_or_line(bool is_face)
{

	bool color = is_face;

	glLineWidth(linewidth);

	bool transparent = true;
	float alpha = mesh_alpha;
	if (alpha == 1.)
		transparent = false;

	glEnable(GL_BLEND);
	//glEnable(GL_DEPTH);  
	//glClearColor(0.0, 0.0, 0.0, 0.0);
	if (transparent)
	{	
		//glDisable(GL_CULL_FACE);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);	
		
		//glBlendFunc(GL_ONE, GL_ONE);
	}
	// Push Current Color
	float currentColor[4];
	glGetFloatv(GL_CURRENT_COLOR, currentColor);

	glBegin(GL_TRIANGLES);
	// loop over all faces
	for(int face_index = 0; face_index<faces->rows();face_index++)
	{
		if(show_texture && texture_id && fUseTexture[face_index])
			continue;

		if (color)
		{
			if (face_colors->rows() > 0)
			{
				const ScalarType* color_data = face_colors->data() + 3 * face_index;
				glColor4f(color_data[0], color_data[1], color_data[2], alpha);
			}
		}
		if (transparent)
		{
			glColor4f(currentColor[0], currentColor[1], currentColor[2], alpha);
		}
		// flat shading
		if(normals_type == PER_FACE)
		{

			const RowVector3 &normal = face_normals->row(face_index);
			glNormal3f(normal[0], normal[1], normal[2]);
		}
		// loop over vertices in this face
		for(int vit = 0; vit < faces->cols(); ++vit)
		{
			if (color)
			{
				if (vertex_colors->rows() > 0)
				{
					const ScalarType* color_data = vertex_colors->data() + 3 * ((*faces)(face_index, vit));
					glColor4f(color_data[0], color_data[1], color_data[2], alpha);
				}
			}
			if (transparent)
			{
				glColor4f(currentColor[0], currentColor[1], currentColor[2], alpha);
			}
			// not flat shading goes here
			if(normals_type == PER_VERTEX)
			{
				const RowVector3 &normal = vertex_normals->row((*faces)(face_index,vit));
				glNormal3f(normal[0], normal[1], normal[2]);
			}
			else
				if (normals_type == PER_CORNER)
				{
					const RowVector3 &normal = corner_normals->row((*fNormIndices)(face_index,vit));
					glNormal3f(normal[0], normal[1], normal[2]);
				}
				// assumes every point in vertices is length == 3
				const ScalarType *vertex_data = vertices->data() + 3* (*faces)(face_index,vit);
				glVertex3f(vertex_data[0], vertex_data[1], vertex_data[2]);

		}
	}
	
	glEnd();

	glDisable(GL_BLEND);
	// Popcurrent color
	glColor4f(currentColor[0], currentColor[1], currentColor[2], currentColor[3]);//wangyu

	if(!show_texture || !texture_id)
		return;

	glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glEnable(GL_TEXTURE_2D);

	glBegin(GL_TRIANGLES);
	// loop over all faces
	for(int face_index = 0; face_index<faces->rows();face_index++)
	{
		if(!fUseTexture[face_index])
			continue;

		if (color && face_colors->rows() > 0)
		{
			const ScalarType* color_data = face_colors->data() + 3 * face_index;
			glColor4f(color_data[0], color_data[1], color_data[2], alpha);
		}
		// flat shading
		if(normals_type == PER_FACE)
		{

			const RowVector3 &normal = face_normals->row(face_index);
			glNormal3f(normal[0], normal[1], normal[2]);
		}
		int corner_index = 0;
		// loop over vertices in this face
		for(int vit = 0; vit < faces->cols(); ++vit, ++corner_index)
		{
			if (color && vertex_colors->rows() > 0)
			{
				const ScalarType* color_data = vertex_colors->data() + 3 * ( (*faces)(face_index,vit) );
				glColor4f(color_data[0], color_data[1], color_data[2], alpha);
			}
			// not flat shading goes here
			if(normals_type == PER_VERTEX)
			{
				const RowVector3 &normal = vertex_normals->row((*faces)(face_index,vit));
				glNormal3f(normal[0], normal[1], normal[2]);
			}
			else
				if (normals_type == PER_CORNER)
				{
					const RowVector3 &normal = corner_normals->row((*fNormIndices)(face_index,vit));
					glNormal3f(normal[0], normal[1], normal[2]);
				}

				int ti = (*fTexIndices)(face_index,vit);
				double u = (*texCoords)(ti,0);
				double v = (*texCoords)(ti,1);
				glTexCoord2d(u,v);
				// assumes every point in vertices is length == 3
				const ScalarType *vertex_data = vertices->data() + 3* (*faces)(face_index,vit);
				glVertex3f(vertex_data[0], vertex_data[1], vertex_data[2]);

		}
	}
	glEnd();

	glDisable(GL_TEXTURE_2D);    

	glLineWidth(1);
}

void Preview3D::draw_normals()
{
	glLineWidth(0.5); 
	glColor3f(0.0, 0.0, 1.0);
	glBegin(GL_LINES);

		for (int i=0; i<vertices->rows(); i++)
		{
			Vector3 from( (*vertices)(i,0), (*vertices)(i,1), (*vertices)(i,2) );
			Vector3 to( (*vertex_normals)(i,0), (*vertex_normals)(i,1), (*vertex_normals)(i,2) );
			to = to*0.02 + from;
			//paintArrow(from,to,2);
			glVertex3f(from(0),from(1),from(2));
			glVertex3f(to(0),to(1),to(2));
		}

	glEnd();
}

void Preview3D::draw_isolines()
{
	if(vertex_property->rows() == vertices->rows())
	{
		//glDeleteLists(isolines_display_list, 1);

		double min_val = vertex_property->minCoeff();
		double max_val = vertex_property->maxCoeff();

		numIsoLevels = std::max(1,std::min(numIsoLevels,5000));

		isoLevels.clear();
		isoLevels.resize(numIsoLevels);
		isoSegments.clear();
		isoSegments.resize(numIsoLevels,std::vector< std::pair< isoPoint, isoPoint > > (0));

		std::vector< std::vector< double > > isocolors;
		isocolors.resize(numIsoLevels);

		for(int i = 0; i < numIsoLevels-1; ++i)
		{
			double val = (1.0 *i) /(numIsoLevels -1);
			isoLevels[i] = min_val + (max_val - min_val) * val;
			isocolors[i].resize(3);
			isocolors[i][0] = 1.;
			isocolors[i][1] = 0.;
			isocolors[i][2] = 0.;
		}

		isoLevels[numIsoLevels-1] = max_val;
		isocolors[numIsoLevels-1].resize(3);
		isocolors[numIsoLevels-1][0] = 1.;
		isocolors[numIsoLevels-1][1] = 0.;
		isocolors[numIsoLevels-1][2] = 0.;

		for (int i = 0; i < faces->rows(); ++i)
		{
			double t1, t2, p0, p1, p2;
			size_t v0,v1,v2;

			for (int j = 0; j<faces->cols(); j++)
			{
				v0 = (*faces)(i,j);
				p0 = (*vertex_property)(v0,0);

				v1 = (*faces)(i,(j+1)%(faces->cols()));
				v2 = (*faces)(i,(j-1+faces->cols())%(faces->cols()));

				p1 = (*vertex_property)(v1,0);
				p2 = (*vertex_property)(v2,0);

				isoPoint iso1, iso2;
				for (int l = 0; l<numIsoLevels; ++l)
				{
					double level = isoLevels[l];
					if ( ((p0 >= level && p1<=level) || (p1 >= level && p0<=level)) && (p0 != p1) )
					{
						t1 = (p1-level) / (p1 - p0);
						vertexPair v = std::make_pair(v0, v1);
						iso1 = std::make_pair(v, t1);

						if ( ((p0 >= level && p2<=level) || (p2 >= level && p0<=level)) && (p0 != p2) )
						{
							t2 = (p2-level) / (p2 - p0);
							vertexPair v = std::make_pair(v0, v2);
							iso2 = std::make_pair(v, t2);
							isoSegments[l].push_back(std::make_pair(iso1, iso2));
						}
					}
				}
			}
		}

		//glDeleteLists(isolines_display_list, 1);

		// generate new display list
		//isolines_display_list = glGenLists(1);
		//glNewList(isolines_display_list, GL_COMPILE);
		float linewidth;
		glGetFloatv(GL_LINE_WIDTH,&linewidth);
		glLineWidth(2);            
		for (int l = 0; l<numIsoLevels; ++l)
		{
			glColor3d(isocolors[l][0], isocolors[l][1], isocolors[l][2]);
			for (unsigned int i = 0; i < isoSegments[l].size(); ++i)
			{
				isoPoint iso1 = isoSegments[l][i].first;
				isoPoint iso2 = isoSegments[l][i].second;

				double p1[3], p2[3], t;
				vertexPair vp;

				t = iso1.second;
				vp = iso1.first;
				p1[0] = t*(*vertices)(vp.first,0) + (1 - t)*(*vertices)(vp.second,0);
				p1[1] = t*(*vertices)(vp.first,1) + (1 - t)*(*vertices)(vp.second,1);
				p1[2] = t*(*vertices)(vp.first,2) + (1 - t)*(*vertices)(vp.second,2);

				t = iso2.second;
				vp = iso2.first;
				p2[0] = t*(*vertices)(vp.first,0) + (1 - t)*(*vertices)(vp.second,0);
				p2[1] = t*(*vertices)(vp.first,1) + (1 - t)*(*vertices)(vp.second,1);
				p2[2] = t*(*vertices)(vp.first,2) + (1 - t)*(*vertices)(vp.second,2);

				glBegin(GL_LINES);
				glVertex3dv(p1);
				glVertex3dv(p2);
				glEnd();
			}
		}
		glLineWidth(linewidth);            
		//glEndList();
	}
}

void Preview3D::update_colors()
{
	compute_face_colors(face_property, face_colors, colorBarType);
	compute_vertex_colors(vertex_property, vertex_colors, colorBarType);
}

void Preview3D::grab_screen(char* filename)
{
#if 0
	unsigned char* bitmapData = new unsigned char[3 * width * height];

	for (int i=0; i < height; i++) 
	{
		glReadPixels(0, i, width, 1, GL_RGB, GL_UNSIGNED_BYTE, 
			bitmapData + (width * 3 * ((height - 1) - i)));
	}

	stbi_write_png(filename, width, height, 3, bitmapData, width * 3);

	delete [] bitmapData;
#else
	igl::render_to_png(filename,width,height,true,false);
#endif
}

void Preview3D::grab_screen(int index)
{
	char anim_filename[256];
	sprintf_s(anim_filename, 256, "recording/frame%04d.png", index);
	grab_screen(anim_filename);
}

void Preview3D::grab_screen()
{
	grab_screen(frame_index);
}

void TW_CALL Preview3D::record_frame_CB(void *clientData)
{
	//time_t t;
	//time(&t);
	//char anim_filename[256];
	//sprintf_s(anim_filename, 256, "time%d.png", t);
	//static_cast<Preview3D *>(clientData)->grab_screen(anim_filename);
	static_cast<Preview3D *>(clientData)->m_record_one_frame = true;
	return;
}

void Preview3D::restart()
{
	for (unsigned int i = 0; i<PluginManager().plugin_list_.size(); ++i)
		PluginManager().plugin_list_[i]->init(this);
}

void Preview3D::draw_colors_on_mesh(Eigen::VectorXd& new_color)
{
	*vertex_property = new_color;
	compute_isolevels();
	// do not need this: compute_vertex_colors(vertex_property,vertex_colors);
	update_colors();
}

void Preview3D::update_normal(NormalUpdateMethod method)
{
	DeformSkinning::GetReference().update_skinning = true;
	switch(method)
	{
	case NORMAL_UPDATE_NAIVE_RECOMPUTE:
		{
			normals_changed = true;
		}
		break;
	case NORMAL_UPDATE_PBS:
		{

		}
		break;
	case NORMAL_UPDATE_ALL:
		{
			normals_changed = true;
		}
		break;
	}
}

void Preview3D::ReLoadShader(void *clientData)
{
	static_cast<Preview3D *>(clientData)->reload_shader();
}

void Preview3D::reload_shader()
{
	deinitShaderProgram(s_directionalPerPixelProgram);
	deinitShaderProgram(s_directionalPerPixelColorProgram);
	deinitShaderProgram(s_pbsShaderProgram);

	// Load shaders
	//s_directionalPerPixelProgram = loadShaderProgramStr(directionalperpixel_vert, directionalperpixel_frag);
	//s_directionalPerPixelColorProgram = loadShaderProgramStr(directionalperpixelcolor_vert, directionalperpixelcolor_frag);
	//s_pbsShaderProgram = loadShaderProgram(PBS_SHADER_VERT_PATH, PBS_SHADER_FRAG_PATH);
	load_shader();
}

void Preview3D::load_shader()
{
	printf("Loading shader.\n");

	GLint n;
	glGetIntegerv(GL_MAX_VERTEX_ATTRIBS, &n);
	printf("GL_MAX_VERTEX_ATTRIBS: %d\n",n);
	glGetIntegerv(GL_MAX_VERTEX_UNIFORM_COMPONENTS, &n);
	printf("GL_MAX_VERTEX_UNIFORM_COMPONENTS: %d\n",n);
	printf("GL_VERSION: %s\n", glGetString(GL_VERSION));
	printf("GL_SHADING_LANGUAGE_VERSION: %s\n", 
		glGetString(GL_SHADING_LANGUAGE_VERSION));
	printf("GL_RENDERER: %s\n",glGetString(GL_RENDERER));


	std::vector<struct GLSL_Attrib> glsl_attribs;
	glsl_attribs.clear();
	//for (int i=0; i<NUM_WEIGHTS_SLOTS_IN_SHADER; i++)
	//{
	//	// This should corresponds to the name in the shader file.
	//	char name_weights[256] = "weights0";
	//	name_weights[7] += i;
	//	char name_weight_indices[256] = "weight_indices0";
	//	name_weight_indices[14] += i;


	//	struct GLSL_Attrib new_attrib;

	//	loc_of_weights_in_shader[i] = 2 + i;// Location is assigned here.
	//	new_attrib.id = loc_of_weights_in_shader[i];
	//	new_attrib.name = std::string(name_weights);
	//	glsl_attribs.push_back(new_attrib);

	//	loc_of_weight_indices_in_shader[i] = 2+ NUM_WEIGHTS_SLOTS_IN_SHADER + i;// Location is assigned here.
	//	new_attrib.id = loc_of_weight_indices_in_shader[i];
	//	new_attrib.name = std::string(name_weight_indices);
	//	glsl_attribs.push_back(new_attrib);
	//}

	//for (int i=0; i<NUM_WEIGHTS_SLOTS_IN_SHADER; i++)
	//{
	//	loc_of_weights_in_shader[i] = 0;
	//	loc_of_weight_indices_in_shader[i] = 0;
	//}

	// Load shaders
	s_directionalPerPixelProgram = loadShaderProgramStr(directionalperpixel_vert, directionalperpixel_frag);
	s_directionalPerPixelColorProgram = loadShaderProgramStr(directionalperpixelcolor_vert, directionalperpixelcolor_frag);
	s_pbsShaderProgram = loadShaderProgramWithAttribs(PBS_SHADER_VERT_PATH, PBS_SHADER_FRAG_PATH, glsl_attribs);
	//s_pbsShaderProgram = loadShaderProgram(PBS_SHADER_VERT_PATH, PBS_SHADER_FRAG_PATH);

	printShaderInfoLog(s_pbsShaderProgram.p);
	printProgramInfoLog(s_pbsShaderProgram.p); 
}

#include <print_matlab.h>
bool Preview3D::send_status_to_matlab()
{
	const Eigen::MatrixXd vertices_ = vertices->cast<double>();
	const Eigen::MatrixXi faces_ = faces->cast<int>();
	const Eigen::MatrixXi tets_ = tets->cast<int>();
	print_matlab("vertices", vertices_);
	print_matlab("faces", faces_);
	print_matlab("tets", tets_);

	return true;
}

void Preview3D::send_color_to_matlab()
{
	const Eigen::MatrixXd vertex_property_ = vertex_property->cast<double>();
	const Eigen::MatrixXd face_property_ = face_property->cast<double>();
	print_matlab("vertex_property", vertex_property_);
	print_matlab("face_property", face_property_);
	//print_matlab("tet_property",)
}

bool Preview3D::set_vertex_colors(const Eigen::VectorXi& new_color_indices, const Eigen::MatrixXd& new_vertex_colors)
{
	if (vertices->rows()==0)
	{
		printf("Mesh is not set yet!\n");
		return false;
	}

	if (vertex_colors->rows() != vertices->rows())
	{ 
		vertex_colors->resize(vertices->rows(), 3);
		vertex_colors->col(0).setConstant(g_MatDiffuse[0]);
		vertex_colors->col(1).setConstant(g_MatDiffuse[1]);
		vertex_colors->col(2).setConstant(g_MatDiffuse[2]);
	}


	if (new_vertex_colors.cols() != 3)
	{
		printf("Error: the new_vertex_colors matrix does not have correct columns.\n");
		return false;
	}

	if (new_color_indices.rows()!=new_vertex_colors.rows())
	{
		
		if (new_vertex_colors.rows()==1)
		{
			printf("Warning: the new_color_indices size does not match new_vertex_colors size, use the first color for all indices.\n");
		}
		else
		{
			printf("Error: the new_color_indices size does not match new_vertex_colors size.\n");
			return false;
		}
	}

	for (int i = 0; i < new_color_indices.rows(); i++)
	{
		if (new_color_indices(i)>=vertex_colors->rows())
		{
			printf("Error: the new_color_indices(%d) exceeds the rows of vertex_colors.\n");
			is_compiled = false;
			return false;
		}
		else
		{
			if (i >= new_vertex_colors.rows())
			{
				vertex_colors->row(new_color_indices(i)) = new_vertex_colors.row(0);
			}
			else
			{
				vertex_colors->row(new_color_indices(i)) = new_vertex_colors.row(i);
			}
			
		}
		
	}

	is_compiled = false;
	return true;
}

void Preview3D::SetFullScreen(bool fs)
{
	m_full_screen = fs;
	if (m_full_screen)
	{
		glutFullScreen();
	}
	else
	{
		glutReshapeWindow(1280, 720);
		glutPositionWindow(0, 0);
	}
}
bool Preview3D::GetFullScreen() const
{
	return m_full_screen;
}

void Preview3D::hide_all_tw_windows()
{
	if (old_positions.size()==0)
	{
		int pos[2];
		twGetWindowPos(bar, pos);
		old_positions.push_back(std::make_pair(pos[0], pos[1]));
		twSetWindowPos(bar, pos[0], 1000);

		for (unsigned int i = 0; i < PluginManager().plugin_list_.size(); ++i)
		{
			twGetWindowPos(PluginManager().plugin_list_[i]->bar, pos);
			old_positions.push_back(std::make_pair(pos[0], pos[1]));
			twSetWindowPos(PluginManager().plugin_list_[i]->bar, pos[0], 1000);
		}
	}
	else
	{
		assert(PluginManager().plugin_list_.size() + 1 == old_positions.size());

		twSetWindowPos(bar, old_positions[0].first, old_positions[0].second);

		for (unsigned int i = 0; i < PluginManager().plugin_list_.size(); ++i)
		{
			twSetWindowPos(PluginManager().plugin_list_[i]->bar, old_positions[i+1].first, old_positions[i+1].second);
		}
		old_positions.clear();
	}	
}