#ifndef PICKING_PLUGIN_BASE_H
#define PICKING_PLUGIN_BASE_H

//
//  PickingPluginBase.h
//  Preview3D
//
//  Created by Olga Diamanti on 9/21/11.
//  Copyright 2011 ETH Zurich. All rights reserved.
//
// This file has been modified by wangyu, search wangyu for details 


#include "ViewerPlugin.h"

#include <stack>
#include <vector> //added by wangyu

//#include "FAST/skinning/Animation.h"//wangyu
#include <Animation.h>
#include <utils/TimerWrapper.h>//wangyu
#include <lasso.h>

void TW_CALL SaveSelectionCB(void *clientData);
void TW_CALL LoadSelectionCB(void *clientData);
void TW_CALL SaveSelectedPosCB(void *clientData);
void TW_CALL SaveSelecetdIndicesCB(void *clientData);
void TW_CALL ResetCB(void *clientData);




class SelectionFrame
{
public:
	std::string name;
	Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic> Selection;
	SelectionFrame(std::string n, Eigen::MatrixXd S) :name(n), Selection(S) {}
};

class PickingPluginBase : public PreviewPlugin
{
public:

	void GetConstraint( Eigen::VectorXi& I, Eigen::MatrixXd& V);

#define NUM_MOUSE_MODE 3
	enum MouseMode { TRANSLATE_HANDLE, ROTATE_HANDLE, DELETE_SELECTION };

	PickingPluginBase();
	~PickingPluginBase();

	bool selectViaFace;
	
	double cursor_search_radius;
	double handle_radius;

	// implement Serializable interface
	bool Serialize(tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element);
	bool Deserialize(tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element);

	// initialization (runs every time a mesh is loaded or cleared)
	void init(Preview3D *preview);

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

	//saves the current selection to a file
	bool saveSelection();

	//loads a selection to a file
	bool loadSelection(const char* fname);

	bool saveSelectedPos(const char* weights_file_name);
	bool saveSelectedIndices(const char* fname);

	//colors selection with different colors per region
	void colorSelection();

	void reset_mesh();

	void translate_vertices(int region_index, float translation[3]);

	void get_constrained_positions(int r,
		std::vector<int > &constrained_vertices,
		Eigen::MatrixXd &constrained_positions,
		bool overwrite_original_positions);

	// This one means the user's selection has changed, but the constraint keeps the same and nothing moves.
	virtual void selection_changed();
	// This means the user has moved some of it's selected vertices
	virtual void selected_vertices_moved();
	// This means the constrained vertices's indices changed.
	virtual void constraint_changed();//

protected:

	Lasso lasso;

	void lassoDownAlt(int mouse_x, int mouse_y, int selected_index);
	void lassoDownShift(int mouse_x, int mouse_y, int selected_index);
	void lassoDownNone(int mouse_x, int mouse_y, int selected_index);
	void lassoUp(const std::vector<int>& indics_inside_lasso, int modifiers);
	void lassoMove(float translation[3]);

	void push_gl_settings();
	void pop_gl_settings();

	//enum, specifying what action the mouse will take
	MouseMode mouse_mode;

	//the key hit
	Preview3D::KeyModifier m_key_modifier;

protected:

	Eigen::MatrixXd* import_vertices;
	Eigen::MatrixXi* import_faces;

	// Vertex Neighbors
	std::vector<std::vector<IndexType> >* import_vertex_to_vertices;

	//color of handle
	bool random_colors;
	float handle_color[4];

	//vector of size equal to the number of vertices
	//if selection_id[i] = -1,  then vertex i is free (will be deformed)
	//if selection_id[i] = r, vertex i belongs to handle #r (will be either translated/rotated directly, or it will stay put)
	std::vector<int > selection_id;

	//contains all the free region ids
	std::stack<int> m_free_regions;

	//index of the handle that is currently picked, and will be translated/rotated (the non-fixed handle)
	int m_currentRegion;

	// array containing random colors
	Eigen::Matrix<double, Eigen::Dynamic, 3> region_colors;
	Eigen::Matrix<double, Eigen::Dynamic, 3> random_region_colors;

public:

	//list of constrained vertex indices and positions
	std::vector<int> m_constrained_vertices;
	Eigen::MatrixXd  m_constrained_vertex_positions;

protected:

	//specifies whether the mesh is valid
	bool m_meshIsLoaded;

	//the selection file name
	std::string selFilename;

protected:
	bool m_enable_mouse_response;
public:

	void init_Picking();

	// Key-framing
	// Varibles
	bool turn_on_handle_key_frame;
	TimerWrapper key_frame_timer;
	Animation<SelectionFrame> selectionKeyFrame;
	// Functions
	bool loadSelectionFile(const char * selFileName, Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic>& S);
	
	bool load_keyframing_config(const char * config_file_name);
	void update_keyframing_selection();

	void set_turn_on_key_framing(const bool t);
	bool get_turn_on_key_framing() const;

	static void TW_CALL load_dialog_keyframing_config(void *clientData);

	static void TW_CALL SetTurnOnKeyFramingCB(const void *value, void *clientData)
	{
		static_cast<PickingPluginBase*>(clientData)->set_turn_on_key_framing(*static_cast<const bool *>(value));
	}
	static void TW_CALL GetTurnOnKeyFramingCB(void *value, void *clientData)
	{
		*static_cast<bool *>(value) = static_cast<const PickingPluginBase*>(clientData)->get_turn_on_key_framing();
	}

	bool commandLine(std::string c, std::vector<std::string> cl);

};


#endif /*PICKING_PLUGIN_BASE_H*/
