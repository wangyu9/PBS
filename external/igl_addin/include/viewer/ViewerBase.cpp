#include "ViewerBase.h"

#include <utils/ViewerTrackball.h>

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

// Undef Visual Studio macros...
#undef max
#undef min

#include <limits>
#include <cassert>



#ifndef _NOMATLAB_
#include "matlabIO.h"
#endif

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif


#ifdef USING_IGL_HEADER_ONLY_MODE
#define IGL_HEADER_ONLY 
#endif



#include "igl/readDMAT.h"//wangyu


#include <draw_primitives.h>//wangyu
#include <draw_mesh_with_pbs_shader.h>///wangyu


#include "igl/writeDMAT.h"//wangyu
#include "igl/timer.h"
#include "igl/png/render_to_png.h"//wangyu


#include "utils/texture_png.h"//wangyu
#include "utils/stb_image_write.h"//added by wangyu, file got from tiantian




#include "../plugins/DeformSkinning.h"//wangyu

#include "math_helper.h"
//
//#define SHADER_INDEX_OF_WEIGHTS 4
//#define SHADER_INDEX_OF_WEIGHT_INDICES 9


// Max line size for reading files
#define REBAR_NAME "Viewer"
#define MAX_LINE 1000

#define pi 3.1415926535897932384626433832795




ColorPreset ViewerBase::color_presets[NUM_COLOR_PRESETS] = {
	ColorPreset(GOLD_AMBIENT,  GOLD_DIFFUSE,  GOLD_SPECULAR,  DARK_BLUE),
	ColorPreset(SILVER_AMBIENT,SILVER_DIFFUSE,SILVER_SPECULAR,DARK_BLUE),
	ColorPreset(SILVER_AMBIENT,SILVER_DIFFUSE,SILVER_SPECULAR,WHITE),
	ColorPreset(GOLD_AMBIENT,  GOLD_DIFFUSE,  GOLD_SPECULAR,  WHITE),
};



ViewerBase::ViewerBase(int start_time)
{
	// Initialize fields
	serializer = new igl::XMLSerializer("IGLViewer");

	//mesh_display_list.push_back(MeshDisplayUI());
	//mesh_display_list[0] = MeshDisplayUI();
	pmesh_display.push_back(&mesh_display0);
	pmesh_display.push_back(&mesh_display1);

	strcpy(current_working_folder, "C:\\WorkSpace");//added by wangyu

	enable_autoRefresh = true;//false

							  // Camera
	camera_index = 0;
	num_active_camera = 1;

	current_camera_index = 0;

	// Window
	screenResolutionType = SCREEN_RESOLUTION_720P;//m_full_screen = false;
	m_window_x = INIT_WINDOW_POS_X;
	m_window_y = INIT_WINDOW_POS_Y;
	winPosType = WIN_POS_DEFAULT;



	width_percentage = 1.;
	height_percentage = 1.;



	material.set(GOLD_AMBIENT, GOLD_DIFFUSE, GOLD_SPECULAR, 35.0f);


	down = false;
	bar = new igl::ReTwBar;
	barWidth = 200;
	barHeight = 685;
	maxBarHeight = 1200;

	timer = new igl::Timer();

	// Initialize AntTweakBar
	if (!TwInit(TW_OPENGL, NULL))
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



	// ---------------------- GENERAL STATS ----------------------

	bar->TwAddVarRO("fps", TW_TYPE_DOUBLE, &fps,
		" label='Frames per second' help='Displays current number of frames"
		" drawn per second.'"
		" group='Mesh statistics'");

	// ---------------------- RECORDING ------------------------

	//Added by wangyu

	// added by wangyu:
	TwEnumVal screenResolutionEV[NUM_SCREEN_RESOLUTION_TYPE] = {
		{ SCREEN_RESOLUTION_720P, "720P" },
		{ SCREEN_RESOLUTION_1080p, "1080p" },
		{ SCREEN_RESOLUTION_FULL, "FULL" }
	};
	TwType screenResolutionT = TwDefineEnum("Screen Resolution", screenResolutionEV, NUM_SCREEN_RESOLUTION_TYPE);
	bar->TwAddVarCB("Screen Resolution", screenResolutionT, SetFullScreenCB, GetFullScreenCB, this, " group='Recording' key=f");


	//bar->TwAddVarCB("Full Screen", TW_TYPE_BOOLCPP, SetFullScreenCB, GetFullScreenCB, this, " group='Recording' key=f");


	// ---------------------- Camera ----------------------
	bar->TwAddVarRW("Camera Index", TW_TYPE_INT32, &camera_index, "group='Camera'");
	bar->TwAddButton("Load Camera", LoadCameraCB, this,
		"group='Camera'"
		" label='Load Camera' help='Save a Camera.'");
	bar->TwAddButton("Save Camera", SaveCameraCB, this,
		"group='Camera'"
		" label='Save Camera' help='Load a Camera.'");
	bar->TwAddButton("Push Camera", PushCameraCB, this, "group='Camera'");
	bar->TwAddVarRW("Active Camera Num", TW_TYPE_INT32, &num_active_camera, "group='Camera'");

	// ---------------------- LOADING ----------------------



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


	// Add 'g_Zoom' to 'bar': this is a modifable (RW) variable of type TW_TYPE_FLOAT. Its key shortcuts are [z] and [Z].
	bar->TwAddVarRW("Zoom", TW_TYPE_FLOAT, &camera.g_Zoom,
		" min=0.05 max=50 step=0.1 keyIncr=+ keyDecr=- help='Scale the object (1=original size).' group='Scene'");
	bar->TwAddVarRW("ObjTranslation", TW_TYPE_DIR3F, &camera.g_Translation,
		" group='Scene'"
		" label='Object translation' help='Change the object translation.' ");
	// Add 'g_Rotation' to 'bar': this is a variable of type TW_TYPE_QUAT4F which defines the object's orientation
	bar->TwAddVarRW("ObjRotation", TW_TYPE_QUAT4F, &camera.g_Rotation,
		" group='Scene'"
		" label='Object rotation' open help='Change the object orientation.' ");

	bar->TwAddVarRW("Rotate Speed", TW_TYPE_FLOAT, &camera.g_RotateSpeed, " ");
	bar->TwAddVarRW("Rotate Stop Angle", TW_TYPE_FLOAT, &camera.g_StopRotateAngle, " ");
	// Add callback to toggle auto-rotate mode (callback functions are defined above).
	bar->TwAddVarCB("AutoRotate", TW_TYPE_BOOLCPP, SetAutoRotateCB, GetAutoRotateCB, this,
		" label='Auto-rotate' help='Toggle auto-rotate mode.' key=m");
	bar->TwAddVarRW(" KeyRotate", TW_TYPE_BOOLCPP, &camera.g_KeyBoardRotate,
		" label='Key-rotate'");
	bar->TwAddButton("SnapView", snap_to_canonical_quaternionCB, this,
		" group='Scene'"
		" label='Snap to canonical view' key=Z "
		" help='Snaps view to nearest canonical view.'");
	bar->TwAddVarRW("TrackballSnapView", TW_TYPE_DOUBLE,
		&camera.trackball_snap_to_canonical,
		" group='Scene'"
		" label='Trackball snap' keyIncr=D keyDecr=S step=0.05 min=0.0 max=1.0"
		" help='Snaps view to nearest canonical view while using trackball. "
		" 0.0 means no snapping, 1 mean complete snapping'");

	bar->TwAddVarRW("Lighting", TW_TYPE_BOOLCPP, &light.use_lighting,
		" group='Scene'"
		" label='Lighting' key=L help='Use lighting when displaying model'");
	// Add 'g_LightMultiplier' to 'bar': this is a variable of type TW_TYPE_FLOAT. Its key shortcuts are [+] and [-].
	bar->TwAddVarRW("Multiplier", TW_TYPE_FLOAT, &light.g_LightMultiplier,
		" group='Scene'"
		" label='Light booster' min=0.1 max=4 step=0.02 help='Increase/decrease the light power.' ");
	// Add 'g_LightDirection' to 'bar': this is a variable of type TW_TYPE_DIR3F which defines the light direction
	bar->TwAddVarRW("LightDir", TW_TYPE_DIR3F, &light.g_LightDirection,
		" group='Scene'"
		" label='Light direction' open help='Change the light direction.' ");

	bar->TwAddVarRW("Background color", TW_TYPE_COLOR3F,
		&background_color,
		" help='Select a background color' group='Color' colormode=hls");

	bar->TwAddVarRW("Auto Refresh", TW_TYPE_BOOLCPP, &enable_autoRefresh,
		" group='Draw options'"
		" label='Auto Refresh' help='Enables continuously redrawing of screen'");
	bar->TwAddVarCB("Toggle Orthographic/Perspective", TW_TYPE_BOOLCPP, set_toggle_orthoCB, get_toggle_orthoCB, this,
		" group='Viewing Options'"
		" label='Orthographic view' "
		" help='Toggles orthographic / perspective view. Default: perspective.'");
	bar->TwAddButton("Align Camera Center", alignCameraCenterCB, this,
		" group='Viewing Options'"
		" label='Align Camera' key=A help='Set the center of the camera to the mesh center.'");

#ifndef PREVIEW3D_NO_SHADERS
	TwEnumVal ShaderModeEV[NUM_SHADER_MODE] = {
		{ MeshDisplay::OFF,"OFF" },
		{ MeshDisplay::ShaderMode::DIRECTIONAL_PER_PIXEL, "Per pixel lighting" }
	};
	TwType ShaderModeTW = igl::ReTwDefineEnum("ShaderMode", ShaderModeEV, NUM_SHADER_MODE);
	bar->TwAddVarCB(

		"Shader",
		ShaderModeTW,
		set_shader_modeCB,
		get_shader_modeCB,
		this,
		" group='Scene' help='Select shader in use' keyIncr='>' keyDecr='<'");
#endif
	// Colors
	bar->TwAddButton("Reset Colors", ResetColorCB, this, "group='Color'");

	TwEnumVal ColorPresetEV[NUM_COLOR_PRESET_IDS] = {
		{ GOLD_ON_DARK_BLUE, "Gold on dark blue" },
		{ SILVER_ON_DARK_BLUE, "Silver on dark blue" },
		{ SILVER_ON_WHITE, "Silver on white" },
		{ GOLD_ON_WHITE, "Gold on white" },
		{ CUSTOM,"CUSTOM" },
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


	// Add 'g_MatAmbient' to 'bar': this is a variable of type TW_TYPE_COLOR4F (4 floats color)
	// and is inserted into a group named 'Material'.
	bar->TwAddVarRW("Ambient", TW_TYPE_COLOR4F, &material.g_MatAmbient, " group='Material' ");
	// Add 'g_MatDiffuse' to 'bar': this is a variable of type TW_TYPE_COLOR4F (4 floats color)
	// and is inserted into group 'Material'.
	bar->TwAddVarRW("Diffuse", TW_TYPE_COLOR4F, &material.g_MatDiffuse, " group='Material' ");
	bar->TwAddVarRW("Specular", TW_TYPE_COLOR4F, &material.g_MatSpecular, " group='Material' ");
	bar->TwAddVarRW("Shininess", TW_TYPE_FLOAT, &material.g_MatShininess, " group='Material'"
		" min=0 max=128");

	bar->TwAddVarCB("Win X", TW_TYPE_INT32, SetWindowXCB, GetWindowXCB, this, " group='Window'");
	bar->TwAddVarCB("Win Y", TW_TYPE_INT32, SetWindowYCB, GetWindowYCB, this, " group='Window'");
	bar->TwAddVarCB("Width ", TW_TYPE_INT32, SetWindowWidthCB, GetWindowWidthCB, this, " group='Window'");
	bar->TwAddVarCB("Height", TW_TYPE_INT32, SetWindowHeightCB, GetWindowHeightCB, this, " group='Window'");

	// added by wangyu:
	TwEnumVal windowPosEV[NUM_WINDOW_POS_TYPE] = {
		{ WIN_POS_NO_BAR, "No Bar" },
		{ WIN_POS_ZERO, "Zero" },
		{ WIN_POS_DEFAULT, "Default" }
	};
	TwType windowPosT = TwDefineEnum("Win Pos Type", windowPosEV, NUM_WINDOW_POS_TYPE);
	bar->TwAddVarCB("Win Pos Type", windowPosT, SetWinPosTypeCB, GetWinPosTypeCB, this, " group='Window' key=n");

	int index = 0;
	for (auto pmd = pmesh_display.begin(); pmd != pmesh_display.end(); ++pmd)
	{
		auto md = *pmd;		
		md->init(("MeshDisplay"+to_string(index)));
		index++;
	}
	
	camera.SetRotateTime(start_time);

	CopyArray4(DARK_BLUE, background_color);

	if (false)// added by wangyu
	{
		set_color_preset(ColorPresetID::GOLD_ON_WHITE);
	}
}

ViewerBase::~ViewerBase()
{
	delete serializer;
}

bool ViewerBase::load_mesh_from_file(const char* mesh_file_name, int mesh_index)
{
	MeshDisplay& md = GetMainMesh();//mesh_display_list[mesh_index];

	if (!md.load_mesh_from_file(mesh_file_name))
		return false;

	md.get_scale_and_shift_to_fit_mesh(md.vertices, camera.zoom, camera.g_Translation);
	radius = 1 / camera.zoom;

	bool only_2d = md.vertices->cols() != 3 || (md.vertices->rows() == 0 && (md.vertices->col(2).maxCoeff() - md.vertices->col(2).minCoeff())<1e-6);

	if (only_2d)
	{
		camera.set_toggle_ortho(false, width, height);
		camera.enable_rotation = false;
	}
	else
	{
		camera.set_toggle_ortho(true, width, height);
		camera.view_xy_plane();
		camera.enable_rotation = true;//wangyu: we only want to see the surface of 3D model

	}

	return true;
}

bool ViewerBase::key_down(unsigned char key, int modifiers, int mouse_x, int mouse_y)
{

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
		camera.key_board_x_rotation += 0.1;
	}
	if (key == 's')
	{
		camera.key_board_x_rotation -= 0.1;
	}
	if (key == 'a')
	{
		camera.key_board_y_rotation -= 0.1;
	}
	if (key == 'd')
	{
		camera.key_board_y_rotation += 0.1;
	}
	if (key == ',')
	{
		SwitchCamera(true);
	}
	if (key == '.')
	{
		SwitchCamera(false);
	}

	return false;
}

bool ViewerBase::key_up(unsigned char /*key*/, int /*modifiers*/, int /*mouse_x*/, int /*mouse_y*/)
{
	return false;
}

bool ViewerBase::mouse_down(int mouse_x,
	int mouse_y,
	int button,
	int modifiers)
{


	down = true;

#ifdef MAYA_STYLE_CAMERA
	if (modifiers & ALT) {
		down_x = mouse_x;
		down_y = mouse_y;
		mouse_mode =
			button == GLUT_LEFT_BUTTON ? ROTATION :
			button == GLUT_MIDDLE_BUTTON ? TRANSLATE : ZOOM;
	}
	else {
		bool any_true = false;
		for (unsigned int i = 0; i <PluginManager().plugin_list_.size(); ++i)
			any_true |= PluginManager().plugin_list_[i]->mouseDownEvent(mouse_x, mouse_y, button, modifiers);
		if (any_true) return true;
		// wangyu  if (PluginManager().plugin_list_[i]->mouseDownEvent(mouse_x, mouse_y, button, modifiers))
		//      return true;
	}
#else
	switch (button)
	{
	case GLUT_LEFT_BUTTON:
	{

		if (modifiers & SHIFT)
		{
		}
		else if (modifiers == NO_KEY)
		{
			if (camera.enable_rotation)
			{
				//init track ball
				down_x = mouse_x;
				down_y = mouse_y;
				double coord[3];
				Eigen::RowVector3d center;
				MeshDisplay& md = GetMainMesh();
				md.compute_centroid(md.vertices, center);
				gluProject(center[0], center[1], center[2], camera.m_modelview_matrix, camera.m_projection_matrix, camera.m_viewport, &coord[0], &coord[1], &coord[2]);
				down_z = coord[2];
				down_rotation[0] = camera.g_Rotation[0];
				down_rotation[1] = camera.g_Rotation[1];
				down_rotation[2] = camera.g_Rotation[2];
				down_rotation[3] = camera.g_Rotation[3];
				mouse_mode = ROTATION;
			}
		}

		break;
	}

	case GLUT_RIGHT_BUTTON:
	{

		if (modifiers & SHIFT)
		{
		}
		else if (modifiers == NO_KEY)
		{
			down_x = mouse_x;
			down_y = mouse_y;
			double coord[3];
			Eigen::RowVector3d center;
			MeshDisplay& md = GetMainMesh();
			md.compute_centroid(md.vertices, center);
			gluProject(center[0], center[1], center[2], camera.m_modelview_matrix, camera.m_projection_matrix, camera.m_viewport, &coord[0], &coord[1], &coord[2]);
			down_z = coord[2];
			down_translation[0] = camera.g_Translation[0];
			down_translation[1] = camera.g_Translation[1];
			down_translation[2] = camera.g_Translation[2];
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

bool ViewerBase::mouse_up(int mouse_x,
	int mouse_y,
	int button,
	int modifiers)
{


	down = false;

	mouse_mode = NOTHING;

	return true;
}

bool ViewerBase::mouse_move(int mouse_x, int mouse_y)
{
	if (down)
	{
		glPushMatrix();
		glScaled(camera.g_Zoom, camera.g_Zoom, camera.g_Zoom);

		double model_view_matrix[16];
		double projection_matrix[16];
		int    view_port[4];
		glGetDoublev(GL_MODELVIEW_MATRIX, model_view_matrix);
		glGetDoublev(GL_PROJECTION_MATRIX, projection_matrix);
		glGetIntegerv(GL_VIEWPORT, view_port);
		double origin_x, origin_y, origin_z;
		gluProject(
			0., 0., 0.,
			model_view_matrix, projection_matrix,
			view_port, &origin_x, &origin_y, &origin_z);

		float center_x = 0., center_y = 0., half_width = 0., half_height = 0.;
		switch (mouse_mode)
		{
		case ROTATION:
		{
			if (camera.enable_rotation == false)
				return false;

			center_x = origin_x;
			center_y = origin_y;

			half_width = ((float)(view_port[2])) / 4.f;
			half_height = ((float)(view_port[3])) / 4.f;

#ifdef MAYA_STYLE_CAMERA
			float quat_hrz[4], quat_vrt[4];
			float axis_y[3] = { 0, 1, 0 };
			float axis_x[3] = { 1, 0, 0 };
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
				(center_x - down_x) / half_width,
				(down_y - center_y) / half_height,
				(center_x - mouse_x) / half_width,
				(mouse_y - center_y) / half_height);

			// I think we need to do this because we have z pointing out of the
			// screen rather than into the screen
			new_quaternion[2] = -new_quaternion[2];
			add_quats(down_rotation, new_quaternion, camera.g_Rotation);
#endif

			camera.snap_to_canonical_quaternion(camera.trackball_snap_to_canonical);

			break;
		}

		case TRANSLATE:
		{
			//translation
			GLfloat winX, winY;               // Holds Our X and Y Coordinates
			winX = (float)mouse_x;                  // Holds The Mouse X Coordinate
			winY = (float)mouse_y;
			winY = (float)camera.m_viewport[3] - winY;
			double pos1[3];
			gluUnProject(winX, winY, down_z, camera.m_modelview_matrix, camera.m_projection_matrix, camera.m_viewport, &(pos1[0]), &(pos1[1]), &(pos1[2]));


			winX = (float)down_x;                  // Holds The Mouse X Coordinate
			winY = (float)down_y;
			winY = (float)camera.m_viewport[3] - winY;
			double pos0[3];
			gluUnProject(winX, winY, down_z, camera.m_modelview_matrix, camera.m_projection_matrix, camera.m_viewport, &(pos0[0]), &(pos0[1]), &(pos0[2]));

#ifdef MAYA_STYLE_CAMERA
			g_Translation[0] += 5 * (pos1[0] - pos0[0]);
			g_Translation[1] += 5 * (pos1[1] - pos0[1]);
			g_Translation[2] += 5 * (pos1[2] - pos0[2]);
			down_x = mouse_x;
			down_y = mouse_y;
#else
			camera.g_Translation[0] = down_translation[0] + (pos1[0] - pos0[0]);
			camera.g_Translation[1] = down_translation[1] + (pos1[1] - pos0[1]);
			camera.g_Translation[2] = down_translation[2] + (pos1[2] - pos0[2]);
#endif


			break;
		}
		case ZOOM:
		{
			float delta = 0.001f * (mouse_x - down_x + mouse_y - down_y);
			camera.g_Zoom *= 1 + delta;
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

bool ViewerBase::mouse_scroll(int mouse_x, int mouse_y, float delta_y)
{
	double factor = 0.05f;
	camera.g_Zoom = (camera.g_Zoom + delta_y*factor > 0.1f ? camera.g_Zoom + delta_y*factor : 0.1f);
	return true;
}

void ViewerBase::clear_draw()
{
	// Clear frame buffer
	glClearColor(
		background_color[0],
		background_color[1],
		background_color[2],
		background_color[3]);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void ViewerBase::pre_draw(int current_time)
{
	std::vector<Camera*> pCameras;
	pCameras.push_back(&camera);
	//pCameras.push_back(&camera2);

	glEnable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glEnable(GL_NORMALIZE);

	light.set_lighting_GL();



	// Update Cameras
	for (size_t i = 0; i < pCameras.size(); i++)
	{
		pCameras[i]->UpdateCamera(current_time);
	}

	//glPushMatrix();
	//camera.SetGL();

	//push_scene(current_time); // TODO: remove this functions 
}

void ViewerBase::main_draw()
{
	std::vector<Camera*> pCameras;
	pCameras.push_back(&camera);
	//pCameras.push_back(&camera2);

	//for (auto md = mesh_display_list.begin(); md != mesh_display_list.end(); ++md)
	for (auto pmd = pmesh_display.begin(); pmd != pmesh_display.end(); ++pmd)
	{
		auto md = *pmd;

#ifndef PREVIEW3D_NO_SHADERS
		glUseProgram(md->shader_id);
#endif  

		md->draw_grid();

		material.set_material_GL();

		if ((
#ifndef PREVIEW3D_NO_SHADERS
			md->shader_mode == MeshDisplay::OFF &&
#endif        
			(md->vertex_colors->rows() > 0 ||
				md->face_colors->rows() > 0)))
		{
			// Match perpixel color shader which only colors diffuse
			glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
			glEnable(GL_COLOR_MATERIAL);
		}

		if (width > 1e-3 && height>1e-3)
		{
			md->preDraw(light);

#ifdef DEBUG_PLUGINS_SEQUENTIAL_ORDER
			printf("DEBUG_PLUGINS_SEQUENTIAL_ORDER: Viewer draw mesh.\n");
#endif

			for (size_t i = 0; i < pCameras.size(); i++)
			{
				glPushMatrix();
				pCameras[i]->SetGL();
				{
					ViewerBase::draw_mesh(*md);
				}
				glPopMatrix();
			}

			if (md->show_trackball)
				ViewerBase::DrawSphereIcon();
		}
	}

	glDisable(GL_COLOR_MATERIAL);


	double t = timer->getElapsedTimeInSec();
	fps = 1.0 / t;
	timer->start();

	//pop_scene();
	//glPopMatrix();

#ifndef PREVIEW3D_NO_SHADERS
	glUseProgram(0);
#endif

}

void ViewerBase::alignCameraCenter()
{
	MeshDisplay& md = GetMainMesh();
	md.get_scale_and_shift_to_fit_mesh(md.vertices, camera.zoom, camera.g_Translation);
}

void ViewerBase::push_scene(int current_time)
{
	// Rotate scene
	glPushMatrix();

	// not sure why this was here
	//glTranslatef(0.5f, -0.3f, 0.0f);

	//camera.UpdateCamera(current_time);

	camera.SetGL();

}

void ViewerBase::pop_scene()
{
	glPopMatrix();
}

void ViewerBase::resize(int w, int h)
{
	width = width_percentage*w;
	height = height_percentage*h;

	camera.resize(width, height);


	// Send the new window size to AntTweakBar
	barHeight = height - 50;
	if (barHeight > maxBarHeight)
		barHeight = maxBarHeight;
	stringstream barSize;
	barSize << " IGLViewer size='" << barWidth << " " << barHeight << "'";
	TwDefine(barSize.str().c_str());
	TwWindowSize(width, height);
}

void ViewerBase::set_color_preset(const ColorPresetID id)
{
	int nid = id;
	if (nid == NUM_COLOR_PRESETS) return;
	if (nid >= NUM_COLOR_PRESETS) nid = 0;
	CopyArray4(color_presets[nid].back, background_color);
	CopyArray4(color_presets[nid].ambient, material.g_MatAmbient);
	CopyArray4(color_presets[nid].diffuse, material.g_MatDiffuse);
	CopyArray4(color_presets[nid].specular, material.g_MatSpecular);
}

ColorPresetID ViewerBase::get_color_preset()
{
	ColorPreset c(material.g_MatAmbient, material.g_MatDiffuse, material.g_MatSpecular, background_color);
	// loop over presets and determine if currently using a preset
	int j = 0;
	for (; j<NUM_COLOR_PRESETS; j++)
	{
		if (c == color_presets[j])
		{
			break;
		}
	}
	return (ColorPresetID)j;
}


void ViewerBase::SetAutoRotate(bool value)
{
	camera.SetAutoRotate(value);

	if (camera.g_AutoRotate != 0)
	{
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



inline void write_PBS_mesh_attributes(GLuint program, const GLchar* name, const Eigen::MatrixXd & H, const Eigen::MatrixXd & W)
{

}

void ViewerBase::draw_mesh(MeshDisplay& md)
{
	switch (md.meshDrawingType)
	{
	case MESH_DRAWING_DEFAULT:
	{
		draw_mesh_default(md);
		break;
	}
	case MESH_DRAWING_PBS:
	{
#if 0
		draw_mesh_pbs(md);
#endif
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

#if 0
void ViewerBase::draw_mesh_pbs(MeshDisplay& md)
{
	GLuint shader_id_for_face_and_line = md.s_pbsShaderProgram.p;
#ifndef PREVIEW3D_NO_SHADERS
	int shader_id;
	glGetIntegerv(GL_CURRENT_PROGRAM, &shader_id);
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
			md.vertices,
			md.faces,
			md.vertex_normals,
			md.s_pbsShaderProgram,
			weight_gradient,
			weights_shader_info,
			camera,
			material,
			light,
			md.linewidth,
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
#endif 

void ViewerBase::draw_mesh_default(MeshDisplay& md)
{

	if (light.use_lighting)
		glEnable(GL_LIGHTING);
	else
		glDisable(GL_LIGHTING);

	//GLuint shader_id_for_face_and_line = shader_id;//s_pbsShaderProgram.p;//shader_id;//= 0;//addedby wangyu
	//GLuint SHADER_INDEX_OF_WEIGHTS = 4;
	//GLuint SHADER_INDEX_OF_WEIGHT_INDICES = 9;

#ifndef PREVIEW3D_NO_SHADERS
	int shader_id;
	glGetIntegerv(GL_CURRENT_PROGRAM, &shader_id);
	glUseProgram(0);//glUseProgram(shader_id_for_face_and_line);//added by wangyu
#endif

	md.draw_mesh();


#ifndef PREVIEW3D_NO_SHADERS
	// pop current shader
	glUseProgram(shader_id);
#endif

}



void DrawPlaneHandle()
{
	float r = 1.0;
	float dr = r / 10.0f;

	glBegin(GL_LINE_STRIP);
	glVertex3f(+r + dr, +r, 0.0);
	glVertex3f(+r, +r + dr, 0.0);
	glVertex3f(+r - dr, +r, 0.0);
	glVertex3f(+r, +r - dr, 0.0);
	glVertex3f(+r + dr, +r, 0.0);
	glEnd();
	glBegin(GL_LINE_STRIP);
	glVertex3f(-r + dr, -r, 0.0);
	glVertex3f(-r, -r + dr, 0.0);
	glVertex3f(-r - dr, -r, 0.0);
	glVertex3f(-r, -r - dr, 0.0);
	glVertex3f(-r + dr, -r, 0.0);
	glEnd();
}

void DrawCircle()
{
	int nside = 100;
	const double pi2 = 3.14159265 * 2.0;
	glBegin(GL_LINE_LOOP);
	for (double i = 0; i < nside; i++) {
		glNormal3d(cos(i * pi2 / nside), sin(i * pi2 / nside), 0.0);
		glVertex3d(cos(i * pi2 / nside), sin(i * pi2 / nside), 0.0);
	}
	glEnd();
	DrawPlaneHandle();
}

void ViewerBase::DrawSphereIcon()
{
	glPushAttrib(GL_TRANSFORM_BIT | GL_ENABLE_BIT | GL_LINE_BIT | GL_CURRENT_BIT | GL_LIGHTING_BIT);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	//Point3f center = tb->center + tb->track.InverseMatrix()*Point3f(0, 0, 0);
	glTranslatef(-camera.g_Translation[0], -camera.g_Translation[1], -camera.g_Translation[2]);
	glScalef(radius / camera.g_Zoom, radius / camera.g_Zoom, radius / camera.g_Zoom);

	float amb[4] = { .3f, .3f, .3f, 1.0f };
	float col[4] = { .5f, .5f, .8f, 1.0f };
	glEnable(GL_LINE_SMOOTH);
	glLineWidth(2);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glColor3fv(WHITE);
	glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, amb);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, col);

	DrawCircle();
	glRotatef(90, 1, 0, 0);
	DrawCircle();
	glRotatef(90, 0, 1, 0);
	DrawCircle();

	glPopMatrix();
	glPopAttrib();
}

void ViewerBase::copy_working_folder(const char *working_folder)
{
	strcpy(current_working_folder, working_folder);
}


// Screen Settings

void ViewerBase::SetFullScreen(ScreenResolutionType st)
{
	screenResolutionType = st;

	switch (screenResolutionType)
	{
	case SCREEN_RESOLUTION_FULL:
		glutFullScreen();
		return;
		break;
	case SCREEN_RESOLUTION_720P:
		width = 1280;
		height = 720;
		SetWinPosType(winPosType);
		glutReshapeWindow(1280, 720);
		break;
	case SCREEN_RESOLUTION_1080p:
		width = 1920;
		height = 1080;
		SetWinPosType(winPosType);
		glutReshapeWindow(1920, 1080);
		break;
	}
	//resize(width, height);


	// not change, just call it's re-position function
}

ScreenResolutionType ViewerBase::GetFullScreen() const
{
	return screenResolutionType;
}


void ViewerBase::grab_screen(char* filename)
{
#if 0
	unsigned char* bitmapData = new unsigned char[3 * width * height];

	for (int i = 0; i < height; i++)
	{
		glReadPixels(0, i, width, 1, GL_RGB, GL_UNSIGNED_BYTE,
			bitmapData + (width * 3 * ((height - 1) - i)));
	}

	stbi_write_png(filename, width, height, 3, bitmapData, width * 3);

	delete[] bitmapData;
#else
	igl::png::render_to_png(filename, width, height, true, false);
#endif
}

void ViewerBase::grab_screen(int index)
{
	char anim_filename[256];
	sprintf_s(anim_filename, 256, "recording/frame%04d.png", index);
	grab_screen(anim_filename);
}


void ViewerBase::SetWindowPosAndSize()
{
	glutReshapeWindow(width, height);
	glutPositionWindow(m_window_x, m_window_y);
}

void ViewerBase::SetWindowX(int x)
{
	m_window_x = x;
	SetWindowPosAndSize();
}

void ViewerBase::SetWindowY(int y)
{
	m_window_y = y;
	SetWindowPosAndSize();
}

void ViewerBase::SetWindowWidth(int w)
{
	width = w;
	SetWindowPosAndSize();
}

void ViewerBase::SetWindowHeight(int h)
{
	height = h;
	SetWindowPosAndSize();
}

void ViewerBase::SetWinPosType(WindowPosType wpt)
{
	winPosType = wpt;

	switch (winPosType)
	{
	case WIN_POS_NO_BAR:
		m_window_x = -8;
		m_window_y = -31;
		break;
	case WIN_POS_ZERO:
		m_window_x = 0;
		m_window_y = 0;
		break;
	case WIN_POS_DEFAULT:
		m_window_x = INIT_WINDOW_POS_X;
		m_window_y = INIT_WINDOW_POS_Y;
		break;
	default:
		break;
	}
	glutPositionWindow(m_window_x, m_window_y);
}

//  Callback function called when the 'AutoRotate' variable value of the tweak bar has changed
void TW_CALL ViewerBase::SetAutoRotateCB(const void *value, void *clientData)
{
	static_cast<ViewerBase *>(clientData)->SetAutoRotate(*static_cast<const bool *>(value));
}

void TW_CALL ViewerBase::GetAutoRotateCB(void *value, void *clientData)
{
	*static_cast<bool *>(value) = static_cast<ViewerBase*>(clientData)->camera.GetAutoRotate();
}

void TW_CALL ViewerBase::set_color_presetCB(const void *value, void *clientData)
{
	static_cast<ViewerBase *>(clientData)->set_color_preset(*static_cast<const ColorPresetID *>(value));
}

void TW_CALL ViewerBase::get_color_presetCB(void *value, void *clientData)
{
	*static_cast<ColorPresetID *>(value) = static_cast<ViewerBase*>(clientData)->get_color_preset();
}

void TW_CALL ViewerBase::view_xy_planeCB(void *clientData)
{
	static_cast<ViewerBase *>(clientData)->camera.view_xy_plane();
}

void TW_CALL ViewerBase::view_xz_planeCB(void *clientData)
{
	static_cast<ViewerBase *>(clientData)->camera.view_xz_plane();
}

void TW_CALL ViewerBase::view_yz_planeCB(void *clientData)
{
	static_cast<ViewerBase *>(clientData)->camera.view_yz_plane();
}

void TW_CALL ViewerBase::snap_to_canonical_quaternionCB(void *clientData)
{
	static_cast<ViewerBase *>(clientData)->camera.snap_to_canonical_quaternion();
}

void TW_CALL ViewerBase::get_toggle_orthoCB(void *value, void *clientData)
{
	*static_cast<bool *>(value) = static_cast<ViewerBase *>(clientData)->camera.get_toggle_ortho();
}

void TW_CALL ViewerBase::set_toggle_orthoCB(const void *value, void *clientData)
{
	static_cast<ViewerBase *>(clientData)->camera.set_toggle_ortho(
		*static_cast<const bool *>(value),
		static_cast<ViewerBase *>(clientData)->width,
		static_cast<ViewerBase *>(clientData)->height);
}

void TW_CALL ViewerBase::alignCameraCenterCB(void *clientData)
{
	static_cast<ViewerBase *>(clientData)->alignCameraCenter();
}

void TW_CALL ViewerBase::SaveCameraCB(void *clientData)
{
	static_cast<ViewerBase *>(clientData)->SaveCamera(static_cast<ViewerBase *>(clientData)->camera_index);
}

void TW_CALL ViewerBase::LoadCameraCB(void *clientData)
{
	char fname[FILE_DIALOG_MAX_BUFFER];
	get_open_file_path(fname);

	static_cast<ViewerBase *>(clientData)->LoadCamera(fname, static_cast<ViewerBase *>(clientData)->camera_index);
}

void TW_CALL ViewerBase::PushCameraCB(void *clientData)
{
	static_cast<ViewerBase *>(clientData)->PushCamera();
}

void ViewerBase::SaveCamera(int index /* = 0 */)
{
	// only support2 2 camera by now
	if (index == 0)
	{
		camera.saveCamera();
	}
	else if (index == 1)
	{
		camera2.saveCamera();
	}
}

void ViewerBase::LoadCamera(const char* fname, int index /* = 0 */)
{

	// only support2 2 camera by now
	if (index == 0)
	{
		camera.loadCamera(fname);
	}
	else if (index == 1)
	{
		camera2.loadCamera(fname);
	}
}

void ViewerBase::PushCamera()
{
	camera_list.push_back(camera);
	current_camera_index = camera_list.size() - 1;
}

void ViewerBase::UseCamera(int index)
{
	if (index >= 0 && index < camera_list.size())
	{
		current_camera_index = index;
		camera = camera_list[index];
	}
	else
	{
		printf("Error: Camera Index (%d) Invalid!", index);
	}
}

void ViewerBase::SwitchCamera(bool incremental /* = true */)
{
	if (camera_list.size() == 0)
	{
		return;
	}

	if (incremental)
	{
		UseCamera((current_camera_index + 1) % camera_list.size());
	}
	else
	{
		UseCamera((current_camera_index - 1) % camera_list.size());
	}
}



#ifndef PREVIEW3D_NO_SHADERS
void TW_CALL ViewerBase::set_shader_modeCB(const void *value, void *clientData)
{
	static_cast<ViewerBase *>(clientData)->GetMainMesh().set_shader_mode(
		*static_cast<const MeshDisplayUI::ShaderMode *>(value), 
		static_cast<ViewerBase *>(clientData)->light.use_lighting);
}
void TW_CALL ViewerBase::get_shader_modeCB(void *value, void *clientData)
{
	*static_cast<MeshDisplayUI::ShaderMode *>(value) = static_cast<ViewerBase*>(clientData)->GetMainMesh().get_shader_mode();
}
#endif
