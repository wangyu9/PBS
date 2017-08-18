#ifndef DEFORM_DIRECT_BASE_H
#define DEFORM_DIRECT_BASE_H

#include "ViewerPlugin.h"
#include "./plugins/DeformerPicking.h"


#include <KeyFramer.h>
#include <lasso.h>
#include "ControlUnit.h"

class DeformDirectBase
{

public:
	DeformDirectBase();
	~DeformDirectBase();

protected:
	// External Output:
	virtual void WriteExternal(const std::vector<int>& selected, const std::vector<Eigen::MatrixXd>& trans) = 0;

protected:
	
	void initFromHandle(const HandleStructure& hs);
	void initControlStructActive(const HandleStructure& hs);
	void initControlStruct(const HandleStructure& hs, const Eigen::VectorXi& cs);
	void addControlStruct(const HandleStructure& hs, const Eigen::VectorXi& cs);

public:

	void UpdateConstraint(const Eigen::VectorXi& known, const Eigen::MatrixXd& knownPos, const Eigen::MatrixXd& Meq, const Eigen::MatrixXd& Peq);

public:

	ControlStruct controlStruct;

private:

	double control_radius;
	double control_scaling;

	bool draw_transwidget_center;
	bool draw_only_selected_transwidget;

public:

	//Keyframing
	TimerWrapper world_timer;
	FrameTimer frame_timer;
	KeyFramer<ControlStructState> keyFramer;

public:

	void save_control_struct_state(const char* fname);
	void load_control_struct_state(const char* fname);

	void set_control_radius(const double r);
	double get_control_radius() const;

	void set_control_scaling(const double s);
	double get_control_scaling() const;

	void set_draw_widget_center(const bool t);
	bool get_draw_widget_center() const;
	
	void set_draw_only_selected_widget(const bool t);
	bool get_draw_only_selected_widget() const;
	
	void update_key_frame();
	bool load_key_frame_config(const char * config_file_name);
	bool start_key_frame();

	void set_draw_skeleton(const bool t);
	bool get_draw_skeleton() const;

	// Key framing
	bool load_keyframing_config(const char * config_file_name);
	void update_keyframing_selection();

	void set_turn_on_key_framing(const bool t);
	bool get_turn_on_key_framing() const;

	void set_key_frame_time(const double t);
	double get_key_frame_time() const;
};


class DeformDirectBaseUI: public DeformDirectBase,  public PreviewPlugin
{

public:

	bool enabled_mouse;
	bool enabled_display;

	bool load_rotation_center_from_file(const char *fname);// To remove


public:
	DeformDirectBaseUI();
	~DeformDirectBaseUI();

	// initialization (runs every time a mesh is loaded or cleared)
	void init(Preview3D *preview);

	// implement Serializable interface
	bool Serialize(tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element);
	bool Deserialize(tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element);

	// keyboard callback
	bool keyDownEvent(unsigned char key, int modifiers, int mouse_x, int mouse_y) ;

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

protected:

	// Pointer to the tweak bar
	//igl::ReTwBar* bar; //wangyu moved to PreviewPlugin

	void push_gl_settings();
	void pop_gl_settings();


	// UI Display
	static void TW_CALL dialog_save_control_struct_state(void *clientData);
	static void TW_CALL dialog_load_control_struct_state(void *clientData);
	static void TW_CALL dialog_load_control_struct_rotation_center(void *clientData);

	static void TW_CALL dialog_load_keyframing_config(void *clientData);
	static void TW_CALL dialog_start_keyframing(void *clientData);

	static void TW_CALL SetDrawCenterCB(const void *value, void *clientData)
	{
		static_cast<DeformDirectBaseUI*>(clientData)->set_draw_widget_center(*static_cast<const bool *>(value));
	}
	static void TW_CALL GetDrawCenterCB(void *value, void *clientData)
	{
		*static_cast<bool *>(value) = static_cast<const DeformDirectBaseUI*>(clientData)->get_draw_widget_center();
	}

	static void TW_CALL SetDrawOnlySelectedCB(const void *value, void *clientData)
	{
		static_cast<DeformDirectBaseUI*>(clientData)->set_draw_only_selected_widget(*static_cast<const bool *>(value));
	}
	static void TW_CALL GetDrawOnlySelectedCB(void *value, void *clientData)
	{
		*static_cast<bool *>(value) = static_cast<const DeformDirectBaseUI*>(clientData)->get_draw_only_selected_widget();
	}

	static void TW_CALL SetControlRadiusCB(const void *value, void *clientData)
	{
		static_cast<DeformDirectBaseUI*>(clientData)->set_control_radius(*static_cast<const double *>(value));
	}
	static void TW_CALL GetControlRadiusCB(void *value, void *clientData)
	{
		*static_cast<double *>(value) = static_cast<const DeformDirectBaseUI*>(clientData)->get_control_radius();
	}

	static void TW_CALL SetControlScalingCB(const void *value, void *clientData)
	{
		static_cast<DeformDirectBaseUI*>(clientData)->set_control_scaling(*static_cast<const double *>(value));
	}
	static void TW_CALL GetControlScalingCB(void *value, void *clientData)
	{
		*static_cast<double *>(value) = static_cast<const DeformDirectBaseUI*>(clientData)->get_control_scaling();
	}

	static void TW_CALL SetDrawSkeletonCB(const void *value, void *clientData)
	{
		static_cast<DeformDirectBaseUI*>(clientData)->set_draw_skeleton(*static_cast<const bool *>(value));
	}
	static void TW_CALL GetDrawSkeletonCB(void *value, void *clientData)
	{
		*static_cast<bool *>(value) = static_cast<const DeformDirectBaseUI*>(clientData)->get_draw_skeleton();
	}

	/*
	float root_translation[3];
	void TranslateControlStructRoots(float *t);
	void SetRootTranslation(const float * t)
	{
		root_translation[0] = t[0];
		root_translation[1] = t[1];
		root_translation[2] = t[2];
		TranslateControlStructRoots(root_translation);
	}
	const float * GetRootTranslation() const
	{
		return root_translation;
	}

	static void TW_CALL SetRootTranslationCB(const void *value, void *clientData)
	{
		static_cast<DeformDirectBaseUI*>(clientData)->SetRootTranslation(static_cast<const float *>(value));
	}
	static void TW_CALL GetRootTranslationCB(void *value, void *clientData)
	{
		const float * t = static_cast<const DeformDirectBaseUI*>(clientData)->GetRootTranslation();
		*(static_cast<float *>(value)+0) = t[0];
		*(static_cast<float *>(value)+1) = t[1];
		*(static_cast<float *>(value)+2) = t[2];
	}*/

	static void TW_CALL SetTurnOnKeyFramingCB(const void *value, void *clientData)
	{
		static_cast<DeformDirectBaseUI*>(clientData)->set_turn_on_key_framing(*static_cast<const bool *>(value));
	}
	static void TW_CALL GetTurnOnKeyFramingCB(void *value, void *clientData)
	{
		*static_cast<bool *>(value) = static_cast<const DeformDirectBaseUI*>(clientData)->get_turn_on_key_framing();
	}

	static void TW_CALL SetKeyFrameTimeCB(const void *value, void *clientData)
	{
		static_cast<DeformDirectBaseUI*>(clientData)->set_key_frame_time(*static_cast<const double *>(value));
	}
	static void TW_CALL GetKeyFrameTimeCB(void *value, void *clientData)
	{
		*static_cast<double *>(value) = static_cast<const DeformDirectBaseUI*>(clientData)->get_key_frame_time();
	}

	static void TW_CALL SetTransitionTypeCB(const void *value, void *clientData)
	{
		static_cast<DeformDirectBaseUI*>(clientData)->keyFramer.SetTransitionType(*static_cast<const TransitionType *>(value));
	}
	static void TW_CALL GetTransitionTypeCB(void *value, void *clientData)
	{
		*static_cast<TransitionType *>(value) = static_cast<const DeformDirectBaseUI*>(clientData)->keyFramer.GetTransitionType();
	}
};


#endif /*DEFORM_DIRECT_BASE_H*/