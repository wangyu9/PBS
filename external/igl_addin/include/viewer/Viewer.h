// Main class of the Viewer

#ifndef VIEWER_H
#define VIEWER_H

#include <Viewer/ViewerBase.h>
#include <FileDialog.h>
#include <time.h>

class Preview3D : public ViewerBase
{
	// The plugin class must be friend to be able to access the private fields of ViewerBase
	friend class PreviewPlugin;

public:
	Preview3D(int start_time);
	void InitPlugins();
	void Restart();

	void Draw(int current_time);

	bool load_mesh_from_file(const char* mesh_file_name);

	/********* Loading-Saving Scene*********/

	bool loadScene(const char* fname);
	bool saveScene();
	void grab_screen();

	bool key_down(unsigned char key, int modifier, int mouse_x, int mouse_y);
	bool key_up(unsigned char key, int modifier, int mouse_x, int mouse_y);
	bool mouse_down(int mouse_x, int mouse_y, int button, int key_pressed);
	bool mouse_up(int mouse_x, int mouse_y, int button, int key_pressed);
	bool mouse_move(int mouse_x, int mouse_y);
	bool mouse_scroll(int mouse_x, int mouse_y, float delta_y);

public:
	bool m_pause;
	float scroll_position;
	bool m_recording_screen;
	bool m_record_one_frame;
	int frame_index;
	
	std::vector<std::pair<int, int>> old_positions;
	void hide_all_tw_windows();

public:
	static void TW_CALL load_mesh_and_init_CB(void *clientData);

	static void TW_CALL SaveSceneCB(void *clientData);
	static void TW_CALL LoadSceneCB(void *clientData);

	static void TW_CALL record_frame_CB(void *clientData);

	static void TW_CALL HideAllTWWindowsCB(void *clientData)
	{
		static_cast<Preview3D*>(clientData)->hide_all_tw_windows();
	}
};

#endif