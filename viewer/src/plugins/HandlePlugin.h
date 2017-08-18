#ifndef HANDLE_PLUGIN_H
#define HANDLE_PLUGIN_H

#include <viewer\HandlePluginBase.h>

class HandlePlugin : public HandlePluginBase
{
public:
	static HandlePlugin& GetReference();

	void ExternalCallTranslateSelectedHandles(float *translation);
	void ExternalCallRotateSelectedHandles(const float * rotation);
	void ExternalCallSetSelectedHandleIndices(std::vector<int> sl);
	void ExternalCallSetSelecetdHandleTrans(const std::vector<Eigen::MatrixXd>& trans);

	void SetVisFromW(const Eigen::MatrixXd& W);
	void SetVisFromM(const Eigen::MatrixXd& M);


	void UpdateForDeformSkinningPBS(const Eigen::MatrixXd& new_handle_pos);
	void UpdateForDeformSkinningLBS(const Eigen::MatrixXd& new_trans);
	void UpdateForDeformSkinningHS(const Eigen::MatrixXd& new_T);

	// Realization of Virtual Functions go here
	virtual bool add_region_handle_from_picking();
	virtual void modify_add_region_handle_from_picking();

	void BeginChangeHandlesOnFly();
	void EndChangeHandlesOnFly(bool set_move);
	void notify_deformers();
	void do_this_when_handle_changed();
	void do_this_when_handle_cleared();
	void do_this_when_handle_moved();
	void vertices_moved(bool only_selected_active);
	bool setup_handles_M_for_HS();
	void init_picking();
	Eigen::MatrixXd* pointer_mesh_vertices();
	void call_matlab_visualize();
	void send_selected_handles_to_MATALB();

	// EXTERNAL CALLs
	void commit_temp_handles();
	void MoveHandlesOntoMesh(){
		move_handle_onto_mesh();
	}

	void lassoDownAlt(int mouse_x, int mouse_y, int selected_index);
	void lassoDownShift();
	void lassoDownNone(int mouse_x, int mouse_y, int selected_index);
	void lassoUp(const std::vector<int>& indics_inside_lasso);
	void lassoMove(float translation[3]);

	void lasso2DownAlt(int mouse_x, int mouse_y);
	void lasso2DownShift();
	void lasso2DownNone(int mouse_x, int mouse_y, int selected_index);
	void lasso2Up(const std::vector<int>& indics_inside_lasso);
	void lasso2Move(float translation[3]);

	/******** Matlab wrappers ********/
	bool call_tetgen_with_handle();
	bool call_triangle_with_handle();

	const HandleStructure& GetHandleStructRef()
	{
		return handleStructure;
	}

	// over-writing existing one
	void init(Preview3D *preview);
	bool keyDownEvent(unsigned char key, int modifiers, int mouse_x, int mouse_y);

	static void TW_CALL call_dialog_tetgen_with_handle(void *clientData);
	static void TW_CALL call_dialog_triangle_with_handle(void *clientData);

	bool commandLine(std::string c, std::vector<std::string> cl);
};


#endif HANDLE_PLUGIN_H