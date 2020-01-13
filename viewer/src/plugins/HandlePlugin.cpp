#include "HandlePlugin.h"

//#define ADD_LIM_DEFORM
#define ADD_SKINNING_DEFORM
//#define ADD_VEGA_DEFORM
#define ADD_PHYS_DEFORM

#include "PickingPlugin.h"
#include "DeformSkinning.h"

#ifdef ADD_LIM_DEFORM
#include "DeformLocallyInjective.h"
#endif
#ifdef ADD_SKINNING_DEFORM
#include "DeformSkinning.h"
#endif
#ifdef ADD_VEGA_DEFORM
#include "DeformVega.h"
#endif
#ifdef ADD_PHYS_DEFORM
#include "DeformPhys.h"
#endif



HandlePlugin  HandlePluginInstance;
HandlePlugin&  HandlePluginInstance_(){ return HandlePluginInstance; }

HandlePlugin& HandlePlugin::GetReference()
{
	return HandlePluginInstance;
}

void HandlePlugin::ExternalCallSetSelecetdHandleTrans(const std::vector<Eigen::MatrixXd>& trans)
{
	set_selected_handles_trans(trans);
}

void HandlePlugin::ExternalCallTranslateSelectedHandles(float *translation)
{
	translate_selected_handles(translation);
}

void HandlePlugin::ExternalCallRotateSelectedHandles(const float * rotation)
{
	rotate_selected_handles(rotation, 1.0);
}

inline bool same_unorder_int_list(const std::vector<int>& sa, const std::vector<int>& sb)
{
	if (sa.size() != sb.size())
		return false;

	std::vector<int> ssa = sa;
	std::sort(ssa.begin(), ssa.end());

	std::vector<int> ssb = sb;
	std::sort(ssb.begin(), ssb.end());

	for (int i = 0; i < ssa.size(); i++)
	{
		if (ssa[i] != ssb[i])
			return false;
	}

	return true;

	//int size = sa.size();
	//int sa_max = -1;
	//int sb_max = -1;
	//for (int i=0; i<size; i++)
	//{
	//	if (sa[i]>sa_max)
	//		sa_max = sa[i];
	//	if (sb[i]>sb_max)
	//		sb_max = sb[i];
	//}

	//if(sa_max!=sb_max)
	//{
	//	return false;
	//}

	//bool* blist_sa = new bool[sa_max+1];//true if selected
	//bool* blist_sb = new bool[sa_max+1];//true if selected
	//for (int i=0; i<sa_max; i++)
	//{
	//	blist_sa[i] = false;
	//	blist_sb[i] = false;
	//}
	//for (int i=0; i<sa.size(); i++)
	//{
	//}

	//delete [] blist_sa;
	//delete [] blist_sb;
}

void HandlePlugin::ExternalCallSetSelectedHandleIndices(std::vector<int> sl)
{
	bool same_as_old = same_unorder_int_list(sl, select_handle_list);

	if (same_as_old)
	{
		return;
	}
	else
	{
		clear_selected_handles();
		for (int i = 0; i < sl.size(); i++)
		{
			add_selected_handles(sl[i]);
		}
		selection_changed();
	}
}

void HandlePlugin::SetVisFromW(const Eigen::MatrixXd& W)
{
		assert(W.cols() == all_handle_list.size());
		for (int i = 0; i < all_handle_list.size(); i++)
		{
			Eigen::MatrixXd Vis(W.rows(), 4);
			Vis.col(0) = W.col(i);
			Vis.col(1) = W.col(i);
			Vis.col(2) = W.col(i);
			Vis.col(3) = W.col(i);

			all_handle_list[i].SetVis(Vis);
		}
}

void HandlePlugin::SetVisFromM(const Eigen::MatrixXd& M)
{
	//std::vector<std::pair<int,Eigen::VectorXd>> cols_to_draw;
	//for (int i=0; i<select_handle_list.size(); i++)
	for (int s = 0; s < all_handle_list.size(); s++)
	{
		//int s = select_handle_list[i];
		int num_slots = Map2Dto3D(s, 1);
		const Eigen::MatrixXd& P = m_preview->GetMainMesh().rest_vertices;
		switch (all_handle_list[s].type())
		{
		case HandleTypePoint:
			assert(num_slots == 1);
			all_handle_list[s].SetVis(M.col(Map2Dto3D(s, 0)));
			//cols_to_draw.push_back( std::make_pair(Map2Dto3D(s,0)+k, Eigen::VectorXd::Ones(P.rows()) ) );

			break;
		case HandleTypePointFrame:
		case HandleTypeVertices:
		case HandleTypeBoneEdge:
			assert(num_slots == 4);
			if (false)
			{
				// write only effective weights
				Eigen::MatrixXd Vis(M.rows(), 4);
				Vis = M.middleCols(Map2Dto3D(s, 0), 4);
				int k = 0;
				for (k = 0; k < 3; k++)// the "3" is intentional
				{
					Vis.col(k) = M.col(Map2Dto3D(s, 0) + k).array() / P.col(k).array();
				}
				all_handle_list[s].SetVis(Vis);
			}
			else
			{
				Eigen::MatrixXd Vis(M.rows(), 8);
				int k = 0;
				for (k = 0; k < 3; k++)
				{
					Vis.col(k) = M.col(Map2Dto3D(s, 0) + k).cwiseQuotient(P.col(k));
					Vis.col(k + 4) = M.col(Map2Dto3D(s, 0) + k);
				}
				Vis.col(3) = Vis.col(7) = M.col(Map2Dto3D(s, 0) + 3);
				all_handle_list[s].SetVis(Vis);
			}


			break;
		}

	}
}

void HandlePlugin::UpdateForDeformSkinningPBS(const Eigen::MatrixXd& new_handle_pos)
{// write handles to user moving buffer handles

	assert(new_handle_pos.cols() == 3);
	assert(new_handle_pos.rows() == all_handle_list.size());

	for (int i = 0; i < new_handle_pos.rows(); i++)
	{
		if (all_handle_list[i].type() == HandleTypePoint)
		{
			//if (all_handle_list[i].active==false)// Do not need this because passive handle could have some active DOFs for LBS

			all_handle_list[i].updateFromPoint3D(new_handle_pos(i, 0), new_handle_pos(i, 1), new_handle_pos(i, 2));
		}
	}

	update_handles_from_list();

	return;
}

void HandlePlugin::UpdateForDeformSkinningLBS(const Eigen::MatrixXd& new_trans)
{// write handles to user moving buffer handles

	assert(new_trans.cols() == 3);
	assert(new_trans.rows() == 4 * all_handle_list.size());


	//int n = ori_Handles.rows();

	//Eigen::MatrixXd M;
	//Eigen::MatrixXd Weights(n,n);
	//Weights.setIdentity();

	//igl::lbs_matrix(ori_Handles,Weights,M);
	//Handles = M*Trans;

	for (int i = 0; i < Handles.rows(); i++)
	{
		if (all_handle_list[i].type() == HandleTypePointFrame)
		{
			//if (all_handle_list[i].type()==HandleTypePoint)// Do not need this because passive handle could have some active DOFs for LBS
			Eigen::MatrixXd Tr = new_trans.block(4 * i, 0, 4, 3);
			all_handle_list[i].updateTrans(Tr);
			//all_handle_list[i].updateFromPoint3D(Handles(i,0),Handles(i,1),Handles(i,2));
		}
	}

	update_handles_from_list();

	return;
}

void HandlePlugin::UpdateForDeformSkinningHS(const Eigen::MatrixXd& new_TH)
{
	handleStructure.update(new_TH);
	return;
}

/***************** Virtual Functions go here ******************/

bool HandlePlugin::add_region_handle_from_picking()
{
	const Eigen::MatrixXd P = PickingPlugin::GetReference().GetConstrainedPos();
	assert(P.cols() == 3);

	add_region_handle(P);

	return true;
}

void HandlePlugin::modify_add_region_handle_from_picking()
{
	const Eigen::MatrixXd RH = PickingPlugin::GetReference().GetConstrainedPos();
	assert(RH.cols() == 3);

	BeginChangeHandlesOnFly();
	add_region_handle_on_fly(RH, modify_on_deformed_shape);
	EndChangeHandlesOnFly(true);
}

void HandlePlugin::BeginChangeHandlesOnFly()
{
	//DeformPhysUI::GetReference().enable_deformer = false;
}

void HandlePlugin::EndChangeHandlesOnFly(bool set_move)
{
	DeformSkinningUI::GetReference().load_weights_from_MATLAB();

	if (set_move)
		selected_vertices_moved();

	// Assuming rotation group file has been loaded
	//DeformPhysUI::GetReference().enable_deformer = true;
	DeformPhysUI::GetReference().prepare_connect();
	DeformPhysUI::GetReference().restartSolver();//prepare_connect();
	DeformPhysUI::GetReference().runSolver = true;

	selected_vertices_moved();// this is because the resizing of constraints vector S in Defrom Phys Only changes in the function of initDeformer function

}

void HandlePlugin::notify_deformers()
{
	// 2014.7.30
	//hasLoadHandles = true;
	//if(hasLoadHandles&&hasLoadWeights)
	//{
	//	update_skinning = true;
	//}

	DeformSkinning::GetReference().SetUpdateSkinning();
	DeformSkinning::GetReference().SetHasLoadHandle(true);
}

bool HandlePlugin::setup_handles_M_for_HS()
{

	//const Eigen::MatrixXd& Weights = DeformSkinning::GetReference().GetWeights();
	//const Eigen::MatrixXd& M3d_lbs = DeformSkinning::GetReference().GetM_lbs(3);
	//const Eigen::MatrixXd& M2d_lbs = DeformSkinning::GetReference().GetM_lbs(2);
	//int n = Weights.rows();
	//int m = Weights.cols();

	Eigen::MatrixXd RG3d;
	Eigen::MatrixXd RG2d;
	handleStructure.get_M(RG3d, RG2d);

	DeformSkinning::GetReference().set_M_hs_2nd(RG3d, RG2d);
	//DeformSkinning::GetReference().setM(M3d_hs, M2d_hs);

	//PickingPlugin::GetReference().init_Picking();
	return true;
}

void HandlePlugin::init_picking()
{
	m_meshIsLoaded = import_vertices->rows() > 0;
}

inline void write_to_constraint_format(
	const std::vector<handleData>& all_handle_list,
	const bool only_active_handle, //false for all, true for only active
	std::vector<IndexType>& m_constrained_vertices,
	Eigen::MatrixXd&  m_constrained_vertex_positions,
	const Eigen::MatrixXd& all_handle_trans,
	Eigen::MatrixXd& constrained_trans)
{

	m_constrained_vertices.clear();

	for (int i = 0; i < all_handle_list.size(); i++)
	{
		bool write_to_constraint = true;
		if (only_active_handle)
		{
			write_to_constraint = all_handle_list[i].active;
		}
		if (write_to_constraint)
		{
			m_constrained_vertices.push_back((IndexType)i);// Important, here it should NOT be all_handle_list[i].index, which is the index in all vertices
		}
	}

	m_constrained_vertex_positions.resize(m_constrained_vertices.size(), 3);
	constrained_trans.resize(m_constrained_vertices.size() * 4, 3);

	int index_active = 0;
	for (int i = 0; i < all_handle_list.size(); i++)
	{
		bool write_to_constraint = true;
		if (only_active_handle)
		{
			write_to_constraint = all_handle_list[i].active;
		}
		if (write_to_constraint)
		{
			m_constrained_vertex_positions(index_active, 0) = all_handle_list[i].x();
			m_constrained_vertex_positions(index_active, 1) = all_handle_list[i].y();
			m_constrained_vertex_positions(index_active, 2) = all_handle_list[i].z();
			constrained_trans.block(index_active * 4, 0, 4, 3) = all_handle_trans.block(i * 4, 0, 4, 3);

			index_active++;
		}
	}

}

// if only_selected_active==true, only selected active handles are sent as constraints
// if only_selected_active==false, all active handles (selected||unselected) are sent as constraints
void HandlePlugin::vertices_moved(bool only_selected_active)
{
	//update_handle_trans();

	std::vector<IndexType> m_constrained_vertices;
	Eigen::MatrixXd  m_constrained_vertex_positions;
	Eigen::MatrixXd constrained_trans;

	write_to_constraint_format(all_handle_list, false,
		m_constrained_vertices,
		m_constrained_vertex_positions,
		HandleTrans(),
		constrained_trans);

#ifdef ADD_LIM_DEFORM
	DeformLocallyInjective::GetReference().UpdateConstraintVertexPositions(m_constrained_vertices, m_constrained_vertex_positions);
#endif
#ifdef ADD_SKINNING_DEFORM
	//printf("to be implemented!\n");
	DeformSkinning::GetReference().UpdateConstraintVertexPositions(m_constrained_vertices, m_constrained_vertex_positions, !CONNECT_PICKING_TO_SKINNING);
#endif
#ifdef ADD_VEGA_DEFORM
	DeformVega::GetReference().UpdateConstraintVertexPositions(m_constrained_vertices, m_constrained_vertex_positions);
#endif
#ifdef ADD_PHYS_DEFORM
	if (true)//keep same in selection_changed and selected_vertices_moved
	{
		std::vector<IndexType> m_active_handles;
		Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic>  m_active_handle_positions;
		Eigen::MatrixXd constrained_trans;

		write_to_constraint_format(all_handle_list, only_selected_active,
			m_active_handles,
			m_active_handle_positions,
			HandleTrans(),
			constrained_trans);

		LinearConstraint23d lc;

		GetConstraint(lc);

		DeformPhysUI::GetReference().UpdateConstraint(lc);


	}
	//DeformPhys::GetReference().UpdateConstraintVertexPositions(m_constrained_vertices, m_constrained_vertex_positions);
#endif
}

void HandlePlugin::do_this_when_handle_changed()
{
	setup_handles_M_for_HS();
	SetHandleConnectivity();

	notify_deformers();
	init_picking();
	vertices_moved(false);
}

void HandlePlugin::do_this_when_handle_cleared()
{
	DeformSkinning::GetReference().SetHasLoadHandle(false);
}

void HandlePlugin::do_this_when_handle_moved()
{
	//applySkinning(false);
	DeformSkinning::GetReference().ApplySkinning();
}

Eigen::MatrixXd* HandlePlugin::pointer_mesh_vertices()
{
	return DeformSkinning::GetReference().pV;
}

/**************************************************************/

void HandlePlugin::commit_temp_handles()
{
	//commit the change to temp_handles;
	temp_Handles = Handles;
	if (only_positional_rotate)
	{
		h_Rotation[0] = 0.0;
		h_Rotation[1] = 0.0;
		h_Rotation[2] = 0.0;
		h_Rotation[3] = 1.0;
		h_old_Rotation[0] = 0.0;
		h_old_Rotation[1] = 0.0;
		h_old_Rotation[2] = 0.0;
		h_old_Rotation[3] = 1.0;

		h_old_Rotation_2D = 0.0;
	}
	else
	{

	}
}

bool HandlePlugin::keyDownEvent(unsigned char key, int modifiers, int mouse_x, int mouse_y)
{
	switch (key)
	{
	case '[':
	{
		if (handleFrames.size() > 0)
		{
			current_handle_frame_index = (current_handle_frame_index - 1) % handleFrames.size();
			printf("Current Handle: %s\n", handleFrames[current_handle_frame_index].name);
			Handles = handleFrames[current_handle_frame_index].Handles;
			update_handles_for_keyframing();
			DeformSkinning::GetReference().SetUpdateSkinning();
		}
		return true;
	}
	break;

	case ']':
	{
		if (handleFrames.size() > 0)
		{
			current_handle_frame_index = (current_handle_frame_index + 1) % handleFrames.size();
			printf("Current Handle: %s\n", handleFrames[current_handle_frame_index].name);
			Handles = handleFrames[current_handle_frame_index].Handles;
			update_handles_for_keyframing();
			DeformSkinning::GetReference().SetUpdateSkinning();
		}
		return true;
	}
	break;
	default:
		break;
	}

	return HandlePluginBase::keyDownEvent(key, modifiers, mouse_x, mouse_y);
}

void HandlePlugin::init(Preview3D *preview)
{
	bool init_antweakbar = (bar==NULL);

	HandlePluginBase::init(preview);

	init_antweakbar = init_antweakbar && (bar != NULL);

	if (init_antweakbar)
	{
		bar->TwAddButton("Call Tetgen with Handle", call_dialog_tetgen_with_handle, this, " group='New'");
		bar->TwAddButton("Call Triangle with Handle", call_dialog_triangle_with_handle, this, " group='New'");
	}
}

/******************************************************************/
/*******************          Lasso           *********************/
/******************************************************************/

void HandlePlugin::lassoDownAlt(int mouse_x, int mouse_y, int selected_index)
{
	//int hit_index;
	//double ray_traced[3];
	//if(HandlePluginBase::GetReference().rayTraceMesh(mouse_x,mouse_y,ray_traced[0],ray_traced[1],ray_traced[2],hit_index))
	//	HandlePluginBase::GetReference().cursor_update(ray_traced[0],ray_traced[1],ray_traced[2],hit_index,'l');
	//rayTraceMesh(mouse_x,mouse_y);
	//selection_changed();

	//lassoDownNone(mouse_x, mouse_y, selected_index);
	//is_during_alt_sliding = true;
}
void HandlePlugin::lassoDownShift()
{
	clear_selected_handles();
	selection_changed();
}
void HandlePlugin::lassoDownNone(int mouse_x, int mouse_y, int selected_index)
{
	if (selected_index<0)
	{
		return;
	}

	bool already_selected = false;
	bool is_selection_changed = false;

	if (all_handle_list.size()>0)
		already_selected = all_handle_list[selected_index].selected;
	if (already_selected || all_handle_list.size() == 0)//|| modifiers == (Preview3D::CTRL | Preview3D::SHIFT)  || modifiers == Preview3D::CTRL)
	{
		;// Nothing //selection_changed();
	}
	else
	{
		clear_selected_handles();
		is_selection_changed = true;
	}

	if (selected_index >= 0)//&&selected_index<all_handle_list.size())
	{
		if (!already_selected)
		{
			add_selected_handles(selected_index);
			is_selection_changed = true;
		}
		//printf("Selected ");HandlePluginBase::GetReference().printf_handle_list(all_handle_list,select_handle_list);
		//selection_changed();
		//WARNING! THIS SHOULD NOT BE CORRECT, WHAT THE RIGHT SHOULD BE?
#ifdef ADD_SKINNING_DEFORM
		if (visualize || visualize_in_matlab)
		{
			VectorX new_col = Eigen::VectorXd::Zero(all_handle_list[0].GetVis().rows());
			for (int i = 0; i<select_handle_list.size(); i++)
			{
				int s = select_handle_list[i];
				Eigen::MatrixXd Vis = all_handle_list[s].GetVis();
				switch (all_handle_list[s].type())
				{
				case HandleTypePoint:
				case HandleTypePointFrame:
					new_col += all_handle_list[s].GetVis();
					break;
				case HandleTypeVertices:
				case HandleTypeBoneEdge:
					if (weights_dim_index_to_visualize >= 0 && weights_dim_index_to_visualize<8)
					{
						new_col += all_handle_list[s].GetVis().col(weights_dim_index_to_visualize);
					}
					break;
				}
			}

			if (visualize)
			{
				m_preview->GetMainMesh().draw_colors_on_mesh(new_col);
				/**m_preview->vertex_property = new_col;
				m_preview->compute_isolevels();
				m_preview->compute_vertex_colors(
				m_preview->vertex_property,
				m_preview->vertex_colors);
				m_preview->update_colors();*/
			}
			if (visualize_in_matlab)
			{
				call_matlab_visualize();
			}


		}
#endif
	}

	if (is_selection_changed)
	{
		selection_changed();
	}
}

inline void add_or_remove_handles(const std::vector<int> & indices_inside_lasso, const LassoUsageType & lut, std::vector<handleData> & all_handle_list)
{
	switch (lut)
	{
	case Lasso_Regular:
	{
		break;
	}
	case Lasso_AddActiveHandle:
	{
		for (int i = 0; i < indices_inside_lasso.size(); i++)
		{
			int k = indices_inside_lasso[i];
			all_handle_list[k].active = true;
		}
		break;
	}
	case Lasso_RemoveActiveHandle:
	{
		for (int i = 0; i < indices_inside_lasso.size(); i++)
		{
			int k = indices_inside_lasso[i];
			all_handle_list[k].active = false;
		}
		break;
	}
	case Lasso_AddPassiveHandle:
	{
		for (int i = 0; i < indices_inside_lasso.size(); i++)
		{
			int k = indices_inside_lasso[i];
			all_handle_list[k].active = false;
		}
		break;
	}
	case Lasso_RemovePassiveHandle:
	{
		for (int i = 0; i < indices_inside_lasso.size(); i++)
		{
			int k = indices_inside_lasso[i];
			all_handle_list[k].active = true;
		}
		break;
	}
	}
}


void HandlePlugin::lassoUp(const std::vector<int>& indics_inside_lasso)
{
	add_or_remove_handles(indics_inside_lasso, lassoUsageType, all_handle_list);

	for (int i = 0; i<indics_inside_lasso.size(); i++)
	{
		add_selected_handles(indics_inside_lasso[i]);
	}

	selection_changed();
}
void HandlePlugin::lassoMove(float translation[3])
{
	translate_selected_handles(translation, false);//HandlePluginBase::GetReference().translate_selected_handles();
	if (!is_during_alt_sliding)
		selected_vertices_moved();//wangyu 2014.9.11 selection_changed();
}

void HandlePlugin::lasso2DownAlt(int mouse_x, int mouse_y)
{
}
void HandlePlugin::lasso2DownShift()
{
}
void HandlePlugin::lasso2DownNone(int mouse_x, int mouse_y, int selected_index)
{
	current_selected_index = selected_index;
}
void HandlePlugin::lasso2Up(const std::vector<int>& indics_inside_lasso)
{
	current_selected_index = -1;
}
void HandlePlugin::lasso2Move(float translation[3])
{
	if (current_selected_index >= 0)
	{
		pointsSet.move(
			current_selected_index,
			translation[0],
			translation[1],
			translation[2]);
	}
}

/******************************************************************/
/*******************       Antweakbar         *********************/
/******************************************************************/

void TW_CALL HandlePlugin::call_dialog_tetgen_with_handle(void *clientData)
{
	static_cast<HandlePlugin *>(clientData)->call_tetgen_with_handle();
}

void TW_CALL HandlePlugin::call_dialog_triangle_with_handle(void *clientData)
{
	static_cast<HandlePlugin *>(clientData)->call_triangle_with_handle();
}

/******************************************************************/
/*******************      MATLAB wrappers     *********************/
/******************************************************************/

#include <igl/matlab/matlabinterface.h>
#include <print_matlab.h>
#include "matlab_folder_path.h"
bool HandlePlugin::call_tetgen_with_handle()
{
#ifdef USE_MATLAB_ENGINE
	using namespace igl::matlab;

	Engine **matlabEngine;
	matlabEngine = DeformSkinning::GetReference().matlabEngine;
	//matlabEngine = new (Engine*);

	std::string str_cd_folder = std::string("cd ") + std::string(MATLAB_FOLDER_PATH);
	mleval(matlabEngine, str_cd_folder);

	const Eigen::MatrixXd& V_rest = m_preview->GetMainMesh().rest_vertices;//*m_preview3d->vertices;
	Eigen::MatrixXi F = *m_preview->GetMainMesh().faces;
	Eigen::MatrixXd P = Handles.cast<double>();

	mlsetmatrix(matlabEngine, "tetgenV", V_rest);
	mlsetmatrix(matlabEngine, "tetgenF", F);
	mlsetmatrix(matlabEngine, "tetgenIV", P);

	mleval(matlabEngine, "ExternalCallList_Tetgen;");

	return true;
#else
	return false;
#endif
}
bool HandlePlugin::call_triangle_with_handle()
{
#ifdef USE_MATLAB_ENGINE
	using namespace igl::matlab;

	Engine **matlabEngine;
	matlabEngine = DeformSkinning::GetReference().matlabEngine;
	//matlabEngine = new (Engine*);

	std::string str_cd_folder = std::string("cd ") + std::string(MATLAB_FOLDER_PATH);
	mleval(matlabEngine, str_cd_folder);

	const Eigen::MatrixXd& V_rest = m_preview->GetMainMesh().rest_vertices;//*m_preview3d->vertices;
	Eigen::MatrixXi F = *m_preview->GetMainMesh().faces;
	Eigen::MatrixXd P = Handles.cast<double>();

	mlsetmatrix(matlabEngine, "triV", V_rest);
	mlsetmatrix(matlabEngine, "triF", F);
	mlsetmatrix(matlabEngine, "triC", P);

	mleval(matlabEngine, "ExternalCallList_Triangle;");

	return true;
#else
	return false;
#endif
}

void HandlePlugin::call_matlab_visualize()
{
#ifdef USE_MATLAB_ENGINE
	using namespace igl::matlab;

	Engine **matlabEngine;
	matlabEngine = DeformSkinning::GetReference().matlabEngine;

	const Eigen::MatrixXd vertex_property_ = m_preview->GetMainMesh().vertex_property->cast<double>();
	const Eigen::MatrixXd face_property_ = m_preview->GetMainMesh().face_property->cast<double>();
	print_matlab("vertex_property", vertex_property_);
	//print_matlab("face_property", face_property_);

	mleval(matlabEngine, str_visualize);
#endif
}

void HandlePlugin::send_selected_handles_to_MATALB()
{
	int num = 0;
	for (int i = 0; i<select_handle_list.size(); i++)
	{
		num += all_handle_list[select_handle_list[i]].Pos().rows();
	}
	Eigen::MatrixXd P(num, 3);
	Eigen::MatrixXi B(num, 1);
	int r = 0;
	for (int i = 0; i<select_handle_list.size(); i++)
	{
		int dr = all_handle_list[select_handle_list[i]].Pos().rows();
		P.block(r, 0, dr, 3) = all_handle_list[select_handle_list[i]].Pos();
		B.block(r, 0, dr, 1) = all_handle_list[select_handle_list[i]].IndexInHandles();
		r += dr;
	}
#ifdef USE_MATLAB_ENGINE
	Eigen::MatrixXd allH = Handles.cast<double>();
	print_matlab("allH", allH);
	print_matlab("SP", P);
	print_matlab("SB", B);
#else
#endif

}




/********************** Command Line ***********************/

#include <viewer/CommandLineBase.h>
bool CommandLine(HandlePlugin& plugin, std::vector<std::string> cl)
{
	if (cl.size() < 1)
	{
		printf("Error: No Command Line for HandlePlugin.\n");
		return false;
	}
	else
	{
		printf("HandlePlugin");
		print_out_argv(cl);
		printf("\n");
	}

	if (cl[0] == std::string("load_handle_struct_from_file"))
	{
		if (cl.size() < 2)
		{
			printf("Error: No handle struct file name for load_handle_struct_from_file.\n");
			return false;
		}
		return plugin.load_handle_struct_from_file(cl[1].c_str());
	}
	else if (cl[0] == std::string("load_handle_mesh_from_file"))
	{
		if (cl.size() < 2)
		{
			printf("Error: No handle struct file name for load_handle_mesh_from_file.\n");
			return false;
		}
		return plugin.load_handle_mesh_from_file(cl[1].c_str());
	}

	else if (cl[0] == std::string("load_pose_rot_from_file"))
	{
		if (cl.size() < 2)
		{
			printf("Error: No handle struct file name for load_pose_rot_from_file.\n");
			return false;
		}
		return plugin.load_pose_rot_from_file(cl[1].c_str());
	}
	else if (cl[0] == std::string("load_pose_pos_from_file"))
	{
		if (cl.size() < 2)
		{
			printf("Error: No handle struct file name for load_pose_pos_from_file.\n");
			return false;
		}
		return plugin.load_pose_pos_from_file(cl[1].c_str());
	}
	else if (cl[0] == std::string("load_rotation_center_from_file"))
	{
		if (cl.size() < 2)
		{
			printf("Error: No rotation center file name for load_rotation_center_from_file.\n");
			return false;
		}
		return plugin.load_rotation_center_from_file(cl[1].c_str());
	}
	else if (cl[0] == std::string("load_points_set_from_file"))
	{
		if (cl.size() < 2)
		{
			printf("Error: No points set file name for load_points_set_from_file.\n");
			return false;
		}
		return plugin.load_points_set_from_file(cl[1].c_str());
	}
	else if (cl[0] == std::string("load_active_from_file"))
	{
		if (cl.size() < 2)
		{
			printf("Error: No active file name for load_active_from_file.\n");
			return false;
		}
		return plugin.load_active(cl[1].c_str());
	}
	else if (cl[0] == std::string("sample_handles"))
	{
		if (cl.size() < 2)
		{
			printf("Error: No sample number for sample_handles.\n");
			return false;
		}
		plugin.sample_handles_number = atoi(cl[1].c_str());
		return plugin.sample_handles();
	}
	else if (cl[0] == std::string("append_handle_struct_from_file"))
	{
		if (cl.size() < 2)
		{
			printf("Error: No sample number for append_handle_struct_from_file.\n");
			return false;
		}
		return plugin.append_handle_struct_from_file((cl[1].c_str()));
	}
	else if (cl[0] == std::string("add_some_purepoint_handle_from_PS"))
	{
		return plugin.add_some_purepoint_handle_from_PS();
	}
	else if (cl[0] == std::string("add_region_handle_from_picking"))
	{
		return plugin.add_region_handle_from_picking();
	}
	else if (cl[0] == std::string("add_region_handle_from_pointset"))
	{
		return plugin.add_region_handle_from_pointset();
	}
	else if (cl[0] == std::string("move_handle_onto_mesh"))
	{
		plugin.move_handle_onto_mesh();
		return true;
	}
	else if (cl[0] == std::string("set_active_handles"))
	{
		plugin.set_active_handles();
		return true;
	}
	else if (cl[0] == std::string("clear_points_set"))
	{
		plugin.clear_points_set();
		return true;
	}
	else if (cl[0] == std::string("set_handle_trans_constrained"))
	{
		plugin.set_handle_trans_constrained();
		return true;
	}
	else
	{
		printf("Error: Unknown Command Line Type for Viewer.\n");
		return false;
	}
}

bool HandlePlugin::commandLine(std::string c, std::vector<std::string> cl)
{
	if (c == std::string("Handle"))
	{
		return CommandLine(*this, cl);
	}
	return false;
}


