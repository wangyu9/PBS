#ifndef HANDLE_PLUGIN_BASE_H
#define HANDLE_PLUGIN_BASE_H

#include <string>

#include "ViewerPlugin.h"
#include "./plugins/DeformerPicking.h"

#include <viewer/PickingPluginBase.h>
//#include "./plugins/PickingPlugin.h"

#include "./plugins/DeformPhys.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <lasso.h>
#include <points_set.h>
#include <handle_structure.h>
#include <linear_constraint.h>

#include "virtual_function_default.h"

typedef enum { WithDepth, NoDepth, NoDisp } HandleDispType;
typedef enum { AsRotationCenter, AsInsertedPoint, AsInsertedHandle } CursorUsageType;
typedef enum { Lasso_Regular, Lasso_AddActiveHandle, Lasso_RemoveActiveHandle, Lasso_AddPassiveHandle, Lasso_RemovePassiveHandle } LassoUsageType;

typedef enum { RotateAroundAvg, RotateAroundFirstPoint, RotateAroundCursor } RotateType;


class HandlePluginBase : public PreviewPlugin
{
public:
	HandlePluginBase();
	~HandlePluginBase();


	void recover_M2d_from_M3d(const Eigen::MatrixXd& M3d, Eigen::MatrixXd& M2d);
	Eigen::MatrixXi BoundaryCondition();

	Eigen::MatrixXd& HandleTrans();

	bool GetConstraint(LinearConstraint23d& lc);

	// Pointer to the tweak bar
	//igl::ReTwBar* bar;//wangyu moved to PreviewPlugin
	// implement Serializable interface
	bool Serialize(tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element);
	bool Deserialize(tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element);

	// initialization (runs every time a mesh is loaded or cleared)
	void init(Preview3D *preview);

public:

	//stuff that is drawn by the plugin before the previewer has displayed the mesh
	//first draw 3d, then 2d
	void preDraw(int currentTime);
	//stuff that is drawn by the plugin after the previewer has displayed the mesh
	//first draw 3d, then 2d
	void postDraw(int currentTime);

	// keyboard callback
	bool keyDownEvent(unsigned char key, int modifiers, int mouse_x, int mouse_y);

	// 
	bool enable_mouse;

	//mouse callback
	bool mouseDownEvent(int mouse_x, int mouse_y, int button, int modifiers);
	bool mouseUpEvent(int mouse_x, int mouse_y, int button, int modifiers);
	bool mouseMoveEvent(int mouse_x, int mouse_y);
	bool mouseScrollEvent(int mouse_x, int mouse_y, float delta);

	void push_gl_settings();
	void pop_gl_settings();

	virtual void BeginChangeHandlesOnFly() = 0;// { std::cerr << "Error. You should define BeginChangeHandlesOnFly() in derived class." << std::endl; }
	virtual void EndChangeHandlesOnFly(bool set_move) = 0;// { std::cerr << "Error. You should define EndChangeHandlesOnFly() in derived class." << std::endl; }
	virtual void notify_deformers() = 0;// { std::cerr << "Error. You should define notify_deformers() in derived class." << std::endl; }
	virtual void do_this_when_handle_changed() = 0;// { std::cerr << "Error. You should define do_this_when_handle_changed() in derived class." << std::endl; }
	virtual void do_this_when_handle_cleared() = 0;// { std::cerr << "Error. You should define do_this_when_handle_cleared() in derived class." << std::endl; }
	virtual void do_this_when_handle_moved() = 0;// { std::cerr << "Error. You should define do_this_when_handle_moved() in derived class." << std::endl; }
	virtual void vertices_moved(bool only_selected_active) = 0;// { std::cerr << "Error. You should define vertices_moved() in derived class." << std::endl; }
	virtual bool setup_handles_M_for_HS() = 0;// { std::cerr << "Error. You should define setup_handles_M_for_HS() in derived class." << std::endl; return false; }
	virtual void init_picking() = 0;// { std::cerr << "Error. You should define init_picking() in derived class." << std::endl; }
	virtual Eigen::MatrixXd* pointer_mesh_vertices() = 0;// { return NULL; }
	virtual void call_matlab_visualize() = 0;// VIRTUAL_VOID_DEFAULT_NO_DEFINATION(0)
	virtual void send_selected_handles_to_MATALB() = 0;// VIRTUAL_VOID_DEFAULT_NO_DEFINATION(0)
	//double &handle_radius;

	std::vector<handleData>& all_handle_list;
	std::vector<int>& select_handle_list;

	//void update_handle_trans();
	// moving from picking plugin
	void clear_selected_handles();
	void add_selected_handles(int index);
	void clear_all_handles();
	void add_all_handles(double x, double y, double z, int index = -1);
protected:
	Eigen::MatrixXd* import_vertices;
	Eigen::MatrixXi* import_faces;
	Eigen::MatrixXd& Trans;
	Eigen::MatrixXd& Trans2d;
	Eigen::MatrixXd& Vars;
	Eigen::MatrixXd& Vars2d;
public:
	Eigen::MatrixXd GetTrans(int dim) const;
	Eigen::MatrixXd GetVars(int dim) const;
	Eigen::MatrixXd ori_Handles;//rest position of handles, used in LBS
	Eigen::MatrixXd temp_Handles;

	HandleStructure handleStructure;

	Eigen::MatrixXd& Handles;
	Eigen::VectorXi& HandleIndices;// The indices of handles in the mesh
	Eigen::VectorXi& HandleIndicesInGroup;
	Eigen::VectorXi& HandleGroup;

	Eigen::MatrixXi& Map2Dto3D;

	Eigen::MatrixXi& HandleFaces;
	Eigen::MatrixXi& HandleTets;

	// Vertex Neighbors
	std::vector<std::vector<IndexType> > vertex_to_vertices;
	void SetHandleConnectivity();
	//void SetHandleConnectvityFromMesh();

	/******** Matlab wrappers ********/
	char str_visualize[2048];
	/********** Handles ************/

	// Old ones: still Compatible
	bool load_handle_group_from_file(const char *group_file_name);
	bool save_handle_group_to_file(const char* handle_file_name);
	bool load_handles_from_file(const char* handle_file_name);
	bool load_handle_mesh_from_file(const char* mesh_file_name);
	bool save_handle_mesh_to_file(const char* mesh_file_name);
	bool save_handles_to_file(const char* handle_file_name);


	bool load_active(const char* active_file_name);
	bool save_active(const char* active_file_name);
	void set_active_handles();

	bool load_handle_struct_from_file(const char * handle_sfile_name);
	bool save_handle_struct_to_file(const char * handle_sfile_name);

	bool export_handle_state_to_file(const char * fname);

	bool load_rotation_center_from_file(const char *fname);

	bool default_pure_point;
	bool build_up_handle_structure(const Eigen::MatrixXd& H, Eigen::VectorXi& HG);
	//bool add_one_handle(const Eigen::MatrixXd& P, const Eigen::VectorXi& I_in_handle);// should only be called from other inserting handle functions



	/********** Poses ************/

	bool load_pose_trans_from_file(const char* handle_file_name);
	bool save_pose_trans_to_file(const char* handle_file_name);

	bool load_pose_pos_from_file(const char* handle_file_name);
	bool save_pose_pos_to_file(const char* handle_file_name);

	bool load_pose_disp_from_file(const char* handle_file_name);
	bool save_pose_disp_to_file(const char* handle_file_name);

	bool load_pose_rot_from_file(const char* fname);
	bool save_pose_rot_to_file(const char* fname);

	bool load_pose_pos_or_disp_from_file(const char* handle_file_name, bool only_active = false, bool abs_pos = true);
	bool save_pose_pos_or_disp_to_file(const char* handle_file_name, bool only_active = false, bool abs_pos = true);

	bool set_pose_pos_or_disp(Eigen::MatrixXd handle_pose, bool only_active = false, bool abs_pos = true);
	/********** Handle Editing ************/
	int sample_handles_number;

	bool add_point_handle(double x = 0, double y = 0, double z = 0, int index = -1);
	bool add_region_handle(const Eigen::MatrixXd& RH);
	bool add_some_purepoint_handle(const Eigen::MatrixXd& PS);

	virtual bool add_region_handle_from_picking() = 0;
	bool add_region_handle_from_pointset();
	bool add_boneedge_handle();
	bool add_some_purepoint_handle_from_PS();
	bool sample_handles();
	bool delete_handles(int index);
	void edit_insert_new_handles(const Eigen::MatrixXd& new_P, const bool in_same_group);
	bool append_handle_struct_from_file(const char * handle_sfile_name);

	void save_selected_handles(const char* handle_file_name);
	/**************************************/

	bool clear_handles();
	bool print_handles();

	bool reset_handles();
	bool reset_rotation();
	void move_handle_onto_mesh(bool onto_deformed_mesh = true);// true for deformed mesh, false on rest pose mesh

	bool auto_marked_when_selected;
	void mark_region_handle_in_mesh();

	/************** Handle Modifying ****************/

	bool modify_on_deformed_shape;
	virtual void modify_add_region_handle_from_picking() = 0;

	/********** Display ************/
	double handle_scale;
	double old_handle_scale;
	HandleDispType handleDispType;
	float handle_color[4];
	double handle_radius;
	double ratio_of_passive_point_handle_radius;
	int max_handle_index_to_draw;
	int min_handle_index_to_draw;

	bool bDrawIndicesInMesh;
	bool bDrawIndicesInHandleStruct;
	bool bDrawRayTracedCursor;
	bool bDrawHandlePoints;
	bool bDrawRegionHandles;
	bool bDrawRegionHandleCenters;
	bool bDrawWireframe;
	bool bDrawHandleFrames2D;
	bool bDrawHandleFrames3D;

	void draw_wire_frame(float linewidth);

	void drawHandles();
	void drawIndicesInMesh();
	void drawIndicesInHandleStruct();
	void drawFrames(int dim);
	void drawRayTracedPoint();
	void drawPointsSet();

	float UNSELECTED_REGION_COLOR[4];
	float SELECTED_REGION_COLOR[4];
	/********** Transform ************/
	void translate_selected_handles(float * translation, bool write_constraint = true);
	void rotate_selected_handles(const float * rotation, const float scale);
	bool rotate_some_handles(const Eigen::MatrixXd& QuatMat, const std::vector<int> IndexMat);
	void positional_rotate_selected_handles(const float *rotation);
	void set_selected_handles_trans(const std::vector<Eigen::MatrixXd>& trans);

	bool only_positional_rotate;//whether only rotate the handles' positions around the group center
	//bool enable_handle_depth;
	bool is3DRotation;
	float *h_Rotation;
	//float h_Rotation[4];
	float h_old_Rotation[4];

	float rotate_2d_speed;;
	float h_Rotation_2D;
	float h_old_Rotation_2D;

	RotateType rotate_type;


	/********** Constraint ************/
	// Constraints to deformers
	bool b_constrain_lbs_affine_trans;// for lbs ture is to constrain the whole affine transformation, false is only position
	void set_handle_trans_constrained();
	void set_handle_pos_constrained();
	bool load_handle_constrained_type_from_file(const char* fname);


	void selection_changed();
	void constraint_changed();
	void selected_vertices_moved();

	void active_vertices_moved();

	/********** Visualize ************/
	bool visualize;
	bool visualize_in_matlab;
	int weights_dim_index_to_visualize;

	/********** On-fly Editing*********/
	void add_purepoint_handles_on_fly(const Eigen::MatrixXd& PH);
	void add_region_handle_on_fly(const Eigen::MatrixXd& PH, bool add_on_deformed_shape);
	void slide_existing_handles_on_fly(const Eigen::MatrixXd& PH, const Eigen::VectorXi& IH);

	/********** Cursor ************/
	bool enable_cursor;

	CursorUsageType cursorUsageType;
	double cursor_search_radius;
	double cursor_rotation_center[3];
	Point3D ray_traced_point;
	bool rayTraceMesh(const int mouse_x, const int mouse_y); // mouse location
	bool cursor_update(double x, double y, double z, int index, char key);

	void cursorDown(int mouse_x, int mouse_y);
	void set_cursor_from_PS();
	/********** Points Set ************/
	PointsSet pointsSet;
	void add_cursor_to_points_set();
	void clear_points_set();

	int current_selected_index;

	bool try_skeleton;
	bool with_symmetric;
	bool try_skeleton_volume;
	double ske_radius;
	int ske_dim;
	int ske_ring;
	int num_seg_try_skeleton;
	int num_per_seg_try_skeleton;
	bool ske_cylinder_or_ellipsoid;

	bool load_points_set_from_file(const char *fname);
	bool save_points_set_to_file(const char* fname);

	/********** Lasso ************/
	bool move_in_plane;
	LassoUsageType lassoUsageType;
	bool is_during_alt_sliding;
	bool m_meshIsLoaded;
	Lasso lasso;
	Lasso lasso2;

	virtual void lassoDownAlt(int mouse_x, int mouse_y, int selected_index) = 0;//VIRTUAL_VOID_DEFAULT_NO_DEFINATION(lassoDownAlt)
	virtual void lassoDownShift() = 0;//VIRTUAL_VOID_DEFAULT_NO_DEFINATION(lassoDownShift)
	virtual void lassoDownNone(int mouse_x, int mouse_y, int selected_index) = 0;//VIRTUAL_VOID_DEFAULT_NO_DEFINATION(lassoDownNone)
	virtual void lassoUp(const std::vector<int>& indics_inside_lasso) = 0;//VIRTUAL_VOID_DEFAULT_NO_DEFINATION(lassoUp)
	virtual void lassoMove(float translation[3]) = 0;// VIRTUAL_VOID_DEFAULT_NO_DEFINATION(lassoMove)

	virtual void lasso2DownAlt(int mouse_x, int mouse_y) = 0;// VIRTUAL_VOID_DEFAULT_NO_DEFINATION(lasso2DownAlt)
	virtual void lasso2DownShift() = 0;// VIRTUAL_VOID_DEFAULT_NO_DEFINATION(lasso2DownShift)
	virtual void lasso2DownNone(int mouse_x, int mouse_y, int selected_index) = 0;// VIRTUAL_VOID_DEFAULT_NO_DEFINATION(lasso2DownNone)
	virtual void lasso2Up(const std::vector<int>& indics_inside_lasso) = 0;// VIRTUAL_VOID_DEFAULT_NO_DEFINATION(lasso2Up)
	virtual void lasso2Move(float translation[3]) = 0;// VIRTUAL_VOID_DEFAULT_NO_DEFINATION(lasso2Move)

	/********** Keyframe ************/
	bool turn_on_handle_key_frame;
	TimerWrapper key_frame_timer;
	Animation<HandleFrame> handleKeyFrame;

	int current_handle_frame_index;
	std::vector<HandleFrame> handleFrames;
	bool load_keyframing_config(const char * config_file_name);
	void update_keyframing_skinning();
	void update_handles_for_keyframing();

public:

	void update_handles_from_list();



	void commit_temp_handles_only();



	int get_sample_handles_number();
	void set_sample_handles_number(int value);

	void SetRotation(const float* new_rotation);
	float* GetRotation() const;

	void SetRotation2D(const float* new_rotation);
	const float* GetRotation2D() const;

	void SetScale(const double s);
	double GetScale() const;

	void SetHandleDisp(HandleDispType ht);
	HandleDispType GetHandleDisp() const;
	void SetRotateType(RotateType rt);
	RotateType GetRotateType() const;
	void SetCursorUsage(CursorUsageType cut);
	CursorUsageType GetCursorUsage() const;
	void SetLassoUsage(LassoUsageType lut);
	LassoUsageType GetLassoUsage() const;
	void set_turn_on_key_framing(const bool t);
	bool get_turn_on_key_framing() const;

protected:


	static void TW_CALL SetRotation2DCB(const void *value, void *clientData)
	{
		static_cast<HandlePluginBase*>(clientData)->SetRotation2D(static_cast<const float *>(value));
	}
	static void TW_CALL GetRotation2DCB(void *value, void *clientData)
	{
		const float * rotation = static_cast<const HandlePluginBase*>(clientData)->GetRotation2D();
		*(static_cast<float *>(value)+0) = rotation[0];
	}
	static void TW_CALL SetRotationCB(const void *value, void *clientData)
	{
		static_cast<HandlePluginBase*>(clientData)->SetRotation(static_cast<const float *>(value));
	}
	static void TW_CALL GetRotationCB(void *value, void *clientData)
	{
		const float * rotation = static_cast<const HandlePluginBase*>(clientData)->GetRotation();
		*(static_cast<float *>(value)+0) = rotation[0];
		*(static_cast<float *>(value)+1) = rotation[1];
		*(static_cast<float *>(value)+2) = rotation[2];
		*(static_cast<float *>(value)+3) = rotation[3];
		/* alternative way:
		float * temp = static_cast<float *>(value);
		temp[0] = rotation[0];
		temp[1] = rotation[1];
		temp[2] = rotation[2];
		temp[3] = rotation[3];*/
	}
	static void TW_CALL SetScaleCB(const void *value, void *clientData)
	{
		static_cast<HandlePluginBase*>(clientData)->SetScale(*static_cast<const double *>(value));
	}
	static void TW_CALL GetScaleCB(void *value, void *clientData)
	{
		*static_cast<double *>(value) = static_cast<const HandlePluginBase*>(clientData)->GetScale();
	}
	static void TW_CALL SetRotateTypeCB(const void *value, void *clientData)
	{
		static_cast<HandlePluginBase*>(clientData)->SetRotateType(*static_cast<const RotateType *>(value));
	}
	static void TW_CALL GetRotateTypeCB(void *value, void *clientData)
	{
		*static_cast<RotateType *>(value) = static_cast<const HandlePluginBase*>(clientData)->GetRotateType();
	}
	static void TW_CALL SetHandleDispCB(const void *value, void *clientData)
	{
		static_cast<HandlePluginBase*>(clientData)->SetHandleDisp(*static_cast<const HandleDispType *>(value));
	}
	static void TW_CALL GetHandleDispCB(void *value, void *clientData)
	{
		*static_cast<HandleDispType *>(value) = static_cast<const HandlePluginBase*>(clientData)->GetHandleDisp();
	}
	static void TW_CALL SetCursorUsageCB(const void *value, void *clientData)
	{
		static_cast<HandlePluginBase*>(clientData)->SetCursorUsage(*static_cast<const CursorUsageType *>(value));
	}
	static void TW_CALL GetCursorUsageCB(void *value, void *clientData)
	{
		*static_cast<CursorUsageType *>(value) = static_cast<const HandlePluginBase*>(clientData)->GetCursorUsage();
	}
	static void TW_CALL SetLassoUsageCB(const void *value, void *clientData)
	{
		static_cast<HandlePluginBase*>(clientData)->SetLassoUsage(*static_cast<const LassoUsageType *>(value));
	}
	static void TW_CALL GetLassoUsageCB(void *value, void *clientData)
	{
		*static_cast<LassoUsageType *>(value) = static_cast<const HandlePluginBase*>(clientData)->GetLassoUsage();
	}
	//Handles new
	static void TW_CALL load_dialog_handle_struct(void *clientData);
	static void TW_CALL save_dialog_handle_struct(void *clientData);
	static void TW_CALL load_dialog_handle_struct_rotation_center(void *clientData);
	//Handles
	static void TW_CALL open_dialog_handle_group(void *clientData);
	static void TW_CALL save_dialog_handle_group(void *clientData);
	static void TW_CALL open_dialog_handles(void *clientData);
	static void TW_CALL load_dialog_handles_from_mesh(void *clientData);
	static void TW_CALL save_dialog_handles_to_mesh(void *clientData);
	static void TW_CALL save_dialog_handles(void *clientData);
	static void TW_CALL load_dialog_active(void *clientData);
	static void TW_CALL save_dialog_active(void *clientData);

	// Poses of handles

	static void TW_CALL open_dialog_pose_trans(void *clientData);
	static void TW_CALL save_dialog_pose_trans(void *clientData);
	static void TW_CALL open_dialog_pose_pos(void *clientData);
	static void TW_CALL save_dialog_pose_pos(void *clientData);
	static void TW_CALL open_dialog_pose_disp(void *clientData);
	static void TW_CALL save_dialog_pose_disp(void *clientData);
	static void TW_CALL open_dialog_pose_rot(void *clientData);
	static void TW_CALL save_dialog_pose_rot(void *clientData);
	// Editing of handles

	static void TW_CALL add_dialog_handles(void *clientData);
	static void TW_CALL add_dialog_handle_from_picking(void *clientData);
	static void TW_CALL add_dialog_handle_from_pointset(void *clientData);
	static void TW_CALL add_dialog_boneedge_handle(void *clientData);
	static void TW_CALL add_dialog_some_purepoint_handle(void *clientData);
	static void TW_CALL clear_dialog_handles(void *clientData);
	static void TW_CALL print_dialog_handles(void *clientData);
	static void TW_CALL save_dialog_selected_handles(void *clientData);
	static void TW_CALL send_dialog_selected_handles_to_MATLAB(void *clientData);
	static void TW_CALL sample_dialog_handles(void *clientData);
	static void TW_CALL reset_dialog_handles(void *clientData);
	static void TW_CALL reset_dialog_rotation(void *clientData);
	static void TW_CALL move_dialog_handle_onto_mesh(void *clientData);
	void TW_CALL get_sample_handles_numberCB(void *value, void *clientData);
	void TW_CALL set_sample_handles_numberCB(const void *value, void *clientData);

	int handle_to_delete;
	static void TW_CALL delete_dialog_handle(void *clientData);
	static void TW_CALL append_dialog_handle_struct_from_file(void *clientData);

	/************** Handle Modifying ****************/
	static void TW_CALL modify_dialog_add_region_handle_from_picking(void *clientData);

	// Display
	static void TW_CALL mark_dialog_region_handle_in_mesh(void *clientData);

	// Constraint
	static void TW_CALL set_dialog_handle_trans_constrained(void *clientData)
	{
		static_cast<HandlePluginBase*>(clientData)->set_handle_trans_constrained();
	}
	static void TW_CALL set_dialog_handle_pos_constrained(void *clientData)
	{
		static_cast<HandlePluginBase*>(clientData)->set_handle_pos_constrained();
	}
	static void TW_CALL load_dialog_handle_constrained(void *clientData);

	// Cursor
	static void TW_CALL set_cursor_from_PS_CB(void *clientData)
	{
		static_cast<HandlePluginBase*>(clientData)->set_cursor_from_PS();
	}

	// Points Set
	static void TW_CALL load_dialog_points_set(void *clientData);
	static void TW_CALL save_dialog_points_set(void *clientData);

	static void TW_CALL add_cursor_to_points_setCB(void *clientData);
	static void TW_CALL clear_points_setCB(void *clientData);

	// Key framing

	static void TW_CALL load_dialog_keyframing_config(void *clientData);
	static void TW_CALL SetTurnOnKeyFramingCB(const void *value, void *clientData)
	{
		static_cast<HandlePluginBase*>(clientData)->set_turn_on_key_framing(*static_cast<const bool *>(value));
	}
	static void TW_CALL GetTurnOnKeyFramingCB(void *value, void *clientData)
	{
		*static_cast<bool *>(value) = static_cast<const HandlePluginBase*>(clientData)->get_turn_on_key_framing();
	}
};


#endif /*HANDLE_PLUGIN_BASE_H*/