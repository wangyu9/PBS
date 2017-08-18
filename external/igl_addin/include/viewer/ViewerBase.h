#ifndef VIEWER_BASE_H
#define VIEWER_BASE_H


// Compilation options

// Disable shaders (useful on virtual machines with problematic video drivers)
// #define PREVIEW3D_NO_SHADERS


#include "types.h"

#ifdef _WIN32
#include <GL/glew.h>
#endif

#ifdef __APPLE__
#define _MACOSX
#endif

#include <AntTweakBar.h>
#include <vector>
#include <set>

#ifdef __APPLE__
#   include <OpenGL/gl.h>
#else
#   ifdef _WIN32
#       include <windows.h>
#   endif
#   include <GL/gl.h>
#endif




#include <libigl_removed/xml/XMLSerializer.h> //old path #include "igl/xml/XMLSerializer.h"
#include <viewer/path_anttweak.h>

//#define DEBUG_PLUGINS_SEQUENTIAL_ORDER
//wangyu whether enable the printing out of DEBUG_PLUGINS_SEQUENTIAL_ORDER

// Forward declaration of the interface for meshplot.m 
class MatlabIO;

namespace igl
{
	class Timer;
	//  class ReTwBar;
	//  class XMLSerializer;
}

// Predefined color presets
enum ColorPresetID
{
	GOLD_ON_DARK_BLUE,
	SILVER_ON_DARK_BLUE,
	SILVER_ON_WHITE,
	GOLD_ON_WHITE,
	CUSTOM
};


#define INIT_WINDOW_POS_X 100
#define INIT_WINDOW_POS_Y 50

// A color preset
class ColorPreset
{
public:
	float ambient[4];
	float diffuse[4];
	float specular[4];
	float back[4];
	ColorPreset() {};
	ColorPreset(
		const float ambient[4],
		const float diffuse[4],
		const float specular[4],
		const float back[4])
	{
		this->ambient[0] = ambient[0];
		this->ambient[1] = ambient[1];
		this->ambient[2] = ambient[2];
		this->ambient[3] = ambient[3];
		this->diffuse[0] = diffuse[0];
		this->diffuse[1] = diffuse[1];
		this->diffuse[2] = diffuse[2];
		this->diffuse[3] = diffuse[3];
		this->specular[0] = specular[0];
		this->specular[1] = specular[1];
		this->specular[2] = specular[2];
		this->specular[3] = specular[3];
		this->back[0] = back[0];
		this->back[1] = back[1];
		this->back[2] = back[2];
		this->back[3] = back[3];
	};
	bool operator==(const ColorPreset &other) const
	{
		return
			this->ambient[0] == other.ambient[0] &&
			this->ambient[1] == other.ambient[1] &&
			this->ambient[2] == other.ambient[2] &&
			this->ambient[3] == other.ambient[3] &&
			this->diffuse[0] == other.diffuse[0] &&
			this->diffuse[1] == other.diffuse[1] &&
			this->diffuse[2] == other.diffuse[2] &&
			this->diffuse[3] == other.diffuse[3] &&
			this->specular[0] == other.specular[0] &&
			this->specular[1] == other.specular[1] &&
			this->specular[2] == other.specular[2] &&
			this->specular[3] == other.specular[3] &&
			this->back[0] == other.back[0] &&
			this->back[1] == other.back[1] &&
			this->back[2] == other.back[2] &&
			this->back[3] == other.back[3];

	};

};



enum ScreenResolutionType {
	SCREEN_RESOLUTION_720P,
	SCREEN_RESOLUTION_1080p,
	SCREEN_RESOLUTION_FULL
};
#define NUM_SCREEN_RESOLUTION_TYPE 3

enum WindowPosType {
	WIN_POS_NO_BAR,
	WIN_POS_ZERO,
	WIN_POS_DEFAULT
};
#define NUM_WINDOW_POS_TYPE 3



#define NUM_WEIGHTS_SLOTS_IN_SHADER 4
#define NUM_WEIGHT_GRADIENTS_SLOTS_IN_SHADER 4
//assert NUM_WEIGHT_GRADIENTS_SLOTS_IN_SHADER < NUM_WEIGHTS_SLOTS_IN_SHADER

////////////////////////////////////////////////////////////////////////////////
// ViewerBase Class
////////////////////////////////////////////////////////////////////////////////

#include "MeshUI.h"
#include "Camera.h"


static void drawgrid(const RowVector3 &aabb_min,
	const RowVector3 &aabb_max,
	const RowVector3 &aabb_center);

class ViewerBase
{
public:

	igl::ReTwBar* bar;

	/********* Data *********/
	std::vector<MeshDisplayUI*> pmesh_display;
	//std::vector<MeshDisplayUI> mesh_display_list;
	MeshDisplayUI mesh_display0;
	MeshDisplayUI mesh_display1;

	Material material;
	Light light;

	/********* Camera *********/

	int camera_index;
	int num_active_camera;
	Camera camera;
	Camera camera2;

	void SaveCamera(int index = 0);
	void LoadCamera(const char* fname, int index = 0);

	std::vector<Camera> camera_list;
	int current_camera_index;
	void PushCamera();
	void UseCamera(int index);
	void SwitchCamera(bool incremental = true);

	/********* Window Options *********/

	float background_color[4];

	double width_percentage;
	double height_percentage;
	int width;
	int height;
	int barWidth;
	int barHeight;
	int maxBarHeight;

	float radius;

	/***********************************/
	bool enable_autoRefresh;

	/********* Other Variables *********/

	igl::XMLSerializer* serializer;
	igl::Timer* timer;
	// number of frames per second
	double fps;
	float down_translation[3];

	static ColorPreset color_presets[NUM_COLOR_PRESETS];

	/********* User Interactions *********/

	enum MouseMode { NOTHING, ROTATION, ZOOM, PAN, TRANSLATE } mouse_mode;
	enum KeyModifier { NO_KEY = TW_KMOD_NONE, SHIFT = TW_KMOD_SHIFT, CTRL = TW_KMOD_CTRL, ALT = TW_KMOD_ALT } key_modifier;


	bool down;
	int last_x;
	int last_y;
	int down_x;
	int down_y;
	float down_z;

	float down_rotation[4];

	/***********************************/



	////////////////////////////////////////////////////////////////////////////
	// Functions
	////////////////////////////////////////////////////////////////////////////

	/********* Initialization *********/
	// Needs time at initialization so that auto-rotation can animate
	ViewerBase(int current_time);

	/********* De-Initialization *********/
	~ViewerBase();

	/********* Loading-Saving Mesh*********/
	bool load_mesh_from_file(const char* mesh_file_name, int mesh_index = 0);

	/********* Trackball *********/
	void DrawSphereIcon();


	/********* Handle key/mouse events*********/
	bool key_down(unsigned char key, int modifier, int mouse_x, int mouse_y);
	bool key_up(unsigned char key, int modifier, int mouse_x, int mouse_y);
	bool mouse_down(int mouse_x, int mouse_y, int button, int key_pressed);
	bool mouse_up(int mouse_x, int mouse_y, int button, int key_pressed);
	bool mouse_move(int mouse_x, int mouse_y);
	bool mouse_scroll(int mouse_x, int mouse_y, float delta_y);

	/********* Loading-Saving Scene*********/


	void alignCameraCenter();

	/********* Compiling / Drawing *********/


	void clear_draw();
	void pre_draw(int current_time);
	void main_draw();

	//void draw(int current_time);
	// push scene matrices
	void push_scene(int current_time);
	// pop scene matrices
	void pop_scene();
	// OpenGL context resize
	void resize(int w, int h);

	
	void draw_mesh(MeshDisplay& md);
	void draw_mesh_default(MeshDisplay& md);
#if 0
	void draw_mesh_pbs(MeshDisplay& md);
#endif



	//removed by wangyu static void draw_text(double x, double y, double z, std::string str, bool small_b);

	void build_vbo();
	void delete_vbo();
	void draw_vbos(bool color);

	/********* Setters/Getters *********/


	void set_color_preset(const ColorPresetID id);
	ColorPresetID get_color_preset();



	/********* Viewing *********/
	void SetAutoRotate(bool value);

	/********* Math Helpers (used to convert anttweakbar trackball to opengl matrices)*********/



private:
	/*********************************/

	/*******************ADDED by wangyu*********************/

public:
	MeshDisplay& GetMainMesh() { return mesh_display0; }
private:

	void SetFullScreen(ScreenResolutionType fs);
	ScreenResolutionType GetFullScreen() const;

	void copy_working_folder(const char *working_folder);

public:
	char current_working_folder[2048];

	void grab_screen(char* filename);
	void grab_screen(int index);

	ScreenResolutionType screenResolutionType;
	//bool m_full_screen;


	int m_window_x;
	int m_window_y;
	//int m_width; // no need to define it here, already define else where
	//int m_height;

	void SetWindowX(int x);
	void SetWindowY(int y);
	int GetWindowX() const { return m_window_x; }
	int GetWindowY() const { return m_window_y; }

	void SetWindowWidth(int w);
	void SetWindowHeight(int h);
	int GetWindowWidth() const { return width; }
	int GetWindowHeight() const { return height; }

	WindowPosType winPosType;
	void SetWinPosType(WindowPosType wpt);
	WindowPosType GetWinPosType() const
	{
		return winPosType;
	}

	void SetWindowPosAndSize();

	/********* AntTweakBar callbacks *********/

	

	static void TW_CALL SetFullScreenCB(const void *value, void *clientData)
	{
		static_cast<ViewerBase*>(clientData)->SetFullScreen(*static_cast<const ScreenResolutionType *>(value));
	}
	static void TW_CALL GetFullScreenCB(void *value, void *clientData)
	{
		*static_cast<ScreenResolutionType *>(value) = static_cast<const ViewerBase*>(clientData)->GetFullScreen();
	}

	static void TW_CALL SetAutoRotateCB(const void *value, void *clientData);
	static void TW_CALL GetAutoRotateCB(void *value, void *clientData);
	static void TW_CALL set_color_presetCB(const void *value, void *clientData);
	static void TW_CALL get_color_presetCB(void *value, void *clientData);

	static void TW_CALL view_xy_planeCB(void *clientData);
	static void TW_CALL view_xz_planeCB(void *clientData);
	static void TW_CALL view_yz_planeCB(void *clientData);
	static void TW_CALL snap_to_canonical_quaternionCB(void *clientData);


	static void TW_CALL set_toggle_orthoCB(const void *value, void *clientData);
	static void TW_CALL get_toggle_orthoCB(void *value, void *clientData);

	static void TW_CALL SaveCameraCB(void *clientData);//wangyu
	static void TW_CALL LoadCameraCB(void *clientData);//wangyu
	static void TW_CALL PushCameraCB(void *clientData);//wangyu
	static void TW_CALL alignCameraCenterCB(void *clientData);

	static void TW_CALL set_shader_modeCB(const void *value, void *clientData);
	static void TW_CALL get_shader_modeCB(void *value, void *clientData);
	static void TW_CALL ResetColorCB(void *clientData)
	{
		static_cast<ViewerBase*>(clientData)->GetMainMesh().ResetColor(static_cast<ViewerBase*>(clientData)->material.g_MatDiffuse);
	}

	static void TW_CALL SetWindowXCB(const void *value, void *clientData)
	{
		static_cast<ViewerBase*>(clientData)->SetWindowX(*static_cast<const int *>(value));
	}
	static void TW_CALL GetWindowXCB(void *value, void *clientData)
	{
		*static_cast<int *>(value) = static_cast<const ViewerBase*>(clientData)->GetWindowX();
	}
	static void TW_CALL SetWindowYCB(const void *value, void *clientData)
	{
		static_cast<ViewerBase*>(clientData)->SetWindowY(*static_cast<const int *>(value));
	}
	static void TW_CALL GetWindowYCB(void *value, void *clientData)
	{
		*static_cast<int *>(value) = static_cast<const ViewerBase*>(clientData)->GetWindowY();
	}

	static void TW_CALL SetWindowWidthCB(const void *value, void *clientData)
	{
		static_cast<ViewerBase*>(clientData)->SetWindowWidth(*static_cast<const int *>(value));
	}
	static void TW_CALL GetWindowWidthCB(void *value, void *clientData)
	{
		*static_cast<int *>(value) = static_cast<const ViewerBase*>(clientData)->GetWindowWidth();
	}
	static void TW_CALL SetWindowHeightCB(const void *value, void *clientData)
	{
		static_cast<ViewerBase*>(clientData)->SetWindowHeight(*static_cast<const int *>(value));
	}
	static void TW_CALL GetWindowHeightCB(void *value, void *clientData)
	{
		*static_cast<int *>(value) = static_cast<const ViewerBase*>(clientData)->GetWindowHeight();
	}

	static void TW_CALL SetWinPosTypeCB(const void *value, void *clientData)
	{
		static_cast<ViewerBase*>(clientData)->SetWinPosType(*static_cast<const WindowPosType *>(value));
	}
	static void TW_CALL GetWinPosTypeCB(void *value, void *clientData)
	{
		*static_cast<WindowPosType *>(value) = static_cast<const ViewerBase*>(clientData)->GetWinPosType();
	}

};



#endif