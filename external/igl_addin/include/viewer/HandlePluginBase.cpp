#define NOMINMAX
// to eliminate the bug of redefine of max min

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

#include <unsupported/Eigen/OpenGLSupport>

#include "HandlePluginBase.h"


#include "PluginManager.h"

#include "FileDialog.h"

#include "path_anttweak.h"

#include <iostream>

// Not header mode goes here
#include <draw_handle_struct.h>

#ifdef USING_IGL_HEADER_ONLY_MODE
#define IGL_HEADER_ONLY 
#endif

#define ENABLE_TETGEN
#ifdef ENABLE_TETGEN
#include "igl/tetgen/tetrahedralize.h"
#endif


#include <Eigen/Core>
#include <igl/readDMAT.h>
#include <igl/writeDMAT.h>

#include "igl/readOBJ.h"
#include "igl/writeOBJ.h"
#include "igl/readMESH.h"
#include "igl/adjacency_list.h"
#include <igl/lbs_matrix.h>

//#include <FAST/skinning/draw_point.h>
//#include <FAST/skinning/draw_directed_line_segment.h>
#include <draw_point.h>
#include <draw_directed_line_segment.h>

//IGL ADDIN
#include <lbs.h>

#include <draw_frame.h>
#include <transform.h>
#include <picking_util.h>

#include <readHANDLE.h>
#include <writeHANDLE.h>
#include <skeleton_sampling.h>
#include <math_helper.h>


// TOREMV
Eigen::MatrixXd& HandlePluginBase::HandleTrans()
{
	return Trans;
}

#define DEFAULT_HANDLE_RADIUS 10.0

HandlePluginBase::HandlePluginBase() :
ray_traced_point(0, 0, 0),
cursor_search_radius(4.0),//PickingPlugin::GetReference().cursor_search_radius),
handle_radius(1.0),//PickingPlugin::GetReference().handle_radius),

Trans(handleStructure.Trans),
Trans2d(handleStructure.Trans2d),
Vars(handleStructure.Vars),
Vars2d(handleStructure.Vars2d),
Handles(handleStructure.Handles),
HandleIndices(handleStructure.HandleIndices),
HandleIndicesInGroup(handleStructure.HandleIndicesInGroup),
HandleGroup(handleStructure.HandleGroup),
Map2Dto3D(handleStructure.Map2Dto3D),
HandleFaces(handleStructure.HandleFaces),
HandleTets(handleStructure.HandleTets),
all_handle_list(handleStructure.all_handle_list),
select_handle_list(handleStructure.select_handle_list)
{
	//check in with the manager
	PluginManager().register_plugin(this);

	bar = NULL;

	HandleIndices.resize(0);
	HandleGroup.resize(0);

	Handles.resize(0, 3);
	HandleFaces.resize(0, 3);
	HandleTets.resize(0, 4);

	// Editing
	handle_to_delete = -1;

	enable_mouse = true;

	//enable_handle_depth = false;
	handleDispType = NoDepth;
	sample_handles_number = 100;
	only_positional_rotate = false;

	// Cursor
	enable_cursor = true;
	cursorUsageType = AsInsertedHandle;

	// Lasso
	move_in_plane = false;
	lassoUsageType = LassoUsageType::Lasso_RemoveActiveHandle;//LassoUsageType::Lasso_Regular;
	is_during_alt_sliding = false;

	current_handle_frame_index = 0;

	turn_on_handle_key_frame = false;

	is3DRotation = true;
	h_Rotation = new float[4];
	h_Rotation[0] = 0.0;
	h_Rotation[1] = 0.0;
	h_Rotation[2] = 0.0;
	h_Rotation[3] = 1.0;

	rotate_2d_speed = 0.0;
	h_Rotation_2D = 0.0;

	h_old_Rotation[0] = 0.0;
	h_old_Rotation[1] = 0.0;
	h_old_Rotation[2] = 0.0;
	h_old_Rotation[3] = 1.0;

	h_old_Rotation_2D = 0.0;

	rotate_type = RotateAroundAvg;

	handle_scale = 1;
	old_handle_scale = 1;

	cursor_rotation_center[0] = 0.0;
	cursor_rotation_center[1] = 0.0;
	cursor_rotation_center[2] = 0.0;

	// Display

	handle_color[0] = 255;
	handle_color[1] = 0;
	handle_color[2] = 255;
	handle_color[3] = 0;

	//handle_color[0] = 100.;
	//handle_color[1] = 150.;
	//handle_color[2] = 180.;
	//handle_color[3] = 0;

	UNSELECTED_REGION_COLOR[0] = 100. / 255.;
	UNSELECTED_REGION_COLOR[1] = 150. / 255.;
	UNSELECTED_REGION_COLOR[2] = 180. / 255.;
	UNSELECTED_REGION_COLOR[3] = 0.;

	SELECTED_REGION_COLOR[0] = 0.;
	SELECTED_REGION_COLOR[1] = 0.;
	SELECTED_REGION_COLOR[2] = 1.;
	SELECTED_REGION_COLOR[3] = 0.;

	auto_marked_when_selected = true;

	handle_radius = DEFAULT_HANDLE_RADIUS;

	ratio_of_passive_point_handle_radius = 1.0;
	min_handle_index_to_draw = 0;
	max_handle_index_to_draw = 60000;

	bDrawHandlePoints = true;
	bDrawRegionHandles = false;
	bDrawRegionHandleCenters = false;

	bDrawIndicesInMesh = false;
	bDrawIndicesInHandleStruct = false;
	bDrawRayTracedCursor = false;
	bDrawWireframe = false;
	bDrawHandleFrames2D = false;
	bDrawHandleFrames3D = false;

	//
	modify_on_deformed_shape = false;

	// points set
	try_skeleton = false;
	try_skeleton_volume = false;
	with_symmetric = false;
	ske_dim = 3;
	ske_radius = 20.;
	ske_ring = 2;
	ske_cylinder_or_ellipsoid = true;
	num_per_seg_try_skeleton = 3;
	num_seg_try_skeleton = 10;

	current_selected_index = -1;


	m_meshIsLoaded = false;

	b_constrain_lbs_affine_trans = true;

	//Display
	visualize = false;
	visualize_in_matlab = false;
	weights_dim_index_to_visualize = 3;
	strcpy(str_visualize, "trimesh(F,V(:,1),V(:,2),vertex_property);");

	default_pure_point = true;


}

HandlePluginBase::~HandlePluginBase()
{
	delete[] h_Rotation;
}

Eigen::MatrixXd HandlePluginBase::GetTrans(int dim) const
{
	if (dim == 3)
	{
		return Trans.cast<double>();
	}
	else
	{
		return Trans2d;
	}
}

Eigen::MatrixXd HandlePluginBase::GetVars(int dim) const
{
	if (dim == 3)
	{
		return Vars.cast<double>();
	}
	else
	{
		return Vars2d;
	}
}

Eigen::MatrixXi HandlePluginBase::BoundaryCondition()
{
	//assert(HandleGroup.maxCoeff()+1==all_handle_list.size());
	Eigen::MatrixXi BC;
	handleStructure.get_boundary_condition(BC);
	return BC;
}

void HandlePluginBase::init(Preview3D *preview)
{

	PreviewPlugin::init(preview);

	//if(IMPORT_HANDLES_IN_PICKING)
	//{
	//	//import_vertices = &DeformSkinning::GetReference().Handles;
	//	//import_faces = &DeformSkinning::GetReference().HandleFaces;
	//	//import_vertex_to_vertices = &DeformSkinning::GetReference().vertex_to_vertices;
	//	import_vertices = &Handles;
	//	import_faces = &HandleFaces;
	//}
	//else
	{
		import_vertices = m_preview->GetMainMesh().vertices;
		import_faces = m_preview->GetMainMesh().faces;
	}

	// init menu bar
	if (bar == NULL)
	{
		// Create a tweak bar
		bar = new igl::ReTwBar;
		bar->TwNewBar("HandlePluginBase");
		TwDefine(" HandlePluginBase size='250 1000' color='76 76 127' position='1580 16' label='HandlePluginBase' "); // change default tweak bar size and color

		bar->TwAddVarRW("Enable Mouse Response", TW_TYPE_BOOLCPP, &enable_mouse, " group='Mouse'");

		/************************** Handles ************************/
		bar->TwAddButton("Load Handle Group", open_dialog_handle_group, this,
			" group='Handles'");
		bar->TwAddButton("Save Handle Group", save_dialog_handle_group, this,
			" group='Handles'");
		bar->TwAddButton("Load Handles", open_dialog_handles, this,
			" group='Handles'");
		bar->TwAddButton("Save Handles", save_dialog_handles, this,
			" group='Handles'");
		bar->TwAddButton("Load Handles from Mesh", load_dialog_handles_from_mesh, this,
			" group='Handles'");
		bar->TwAddButton("Save Handles to Mesh", save_dialog_handles_to_mesh, this,
			" group='Handles'");
		bar->TwAddVarRW("Default as Pure Point", TW_TYPE_BOOLCPP, &default_pure_point, " group='Handles'");

		bar->TwAddButton("Load Active List", load_dialog_active, this,
			" group='Handles'");
		bar->TwAddButton("Save Active List", save_dialog_active, this,
			" group='Handles'");


		/************************** Handles ************************/
		bar->TwAddButton("Load Handle Struct", load_dialog_handle_struct, this, " group='New'");
		bar->TwAddButton("Save Handle Struct", save_dialog_handle_struct, this, " group='New'");
		/************************** Poses ************************/

		bar->TwAddButton("Load Pose Trans", open_dialog_pose_trans, this, " group='Poses'");
		bar->TwAddButton("Save Pose Trans", save_dialog_pose_trans, this, " group='Poses'");
		bar->TwAddButton("Load Pose Pos", open_dialog_pose_pos, this, " group='Poses'");
		bar->TwAddButton("Save Pose Pos", save_dialog_pose_pos, this, " group='Poses'");
		bar->TwAddButton("Load Pose Disp", open_dialog_pose_disp, this, " group='Poses'");
		bar->TwAddButton("Save Pose Disp", save_dialog_pose_disp, this, " group='Poses'");
		bar->TwAddButton("Load Pose Rot (Quaternion Matrix)", open_dialog_pose_rot, this, " group='Poses'");
		bar->TwAddButton("Save Pose Rot (Quaternion Matrix)", save_dialog_pose_rot, this, " group='Poses'");

		/************************* Editing ***********************/

		bar->TwAddButton("Add Region Handle From Picking", add_dialog_handle_from_picking, this, " group='Editing'");
		bar->TwAddButton("Add Region Handle From Point Set", add_dialog_handle_from_pointset, this, " group='Editing'");
		bar->TwAddButton("Add Bone Edge", add_dialog_boneedge_handle, this, " group='Editing' key='e'");
		bar->TwAddButton("Add Some Pure Points", add_dialog_some_purepoint_handle, this, " group='Editing'");
		bar->TwAddButton("Add Handles", add_dialog_handles, this,
			" group='Editing'");
		bar->TwAddButton("Move Handles onto Mesh", move_dialog_handle_onto_mesh, this,
			" group='Editing'");
		bar->TwAddButton("Clear Handles", clear_dialog_handles, this,
			" group='Editing'");
		bar->TwAddButton("Print Handles", print_dialog_handles, this,
			" group='Editing'");
		bar->TwAddButton("Save Selected Handle Pos", save_dialog_selected_handles, this, " group='Editing'");
		bar->TwAddButton("Send Selected Handle to MATLAB", send_dialog_selected_handles_to_MATLAB, this, " group='Editing'");
		bar->TwAddButton("Reset Handles", reset_dialog_handles, this,
			" group='Editing'");
		bar->TwAddVarRW("Sample #", TW_TYPE_INT32, &sample_handles_number,
			" group='Editing'");
		bar->TwAddButton("Sample Handles", sample_dialog_handles, this,
			" group='Editing'");
		bar->TwAddVarRW("Handle Index to Delete", TW_TYPE_INT32, &handle_to_delete, " group='Editing'");
		bar->TwAddButton("Delete Handle", delete_dialog_handle, this, " group='Editing'");
		bar->TwAddButton("Append Handle Struct", append_dialog_handle_struct_from_file, this, " group='Editing'");

		/************************* Modifying ***********************/
		bar->TwAddVarRW("Modify: On deformed shape", TW_TYPE_BOOLCPP, &modify_on_deformed_shape, " group='Modifying'");
		bar->TwAddButton("Modify: Add Region Handle From Picking", modify_dialog_add_region_handle_from_picking, this, " group='Modifying'");

		/*********************** Visualize **********************/
		bar->TwAddVarRW("Visualize Weights", TW_TYPE_BOOLCPP, &visualize, " group='Visualize'");
		bar->TwAddVarRW("Send Weights to Matlab", TW_TYPE_BOOLCPP, &visualize_in_matlab, " group='Visualize'");

		bar->TwAddVarRW("Command to Matlab", TW_TYPE_CSSTRING(sizeof(str_visualize)), str_visualize, " group='Visualize'");
		bar->TwAddVarRW("Weights Dim to Visualize", TW_TYPE_INT32, &weights_dim_index_to_visualize, " group='Visualize'");

		/************************* Display ***********************/


		bar->TwAddVarRW("Indices: Mesh", TW_TYPE_BOOLCPP, &bDrawIndicesInMesh, " group='Display'");
		bar->TwAddVarRW("Indices: HandleStruct", TW_TYPE_BOOLCPP, &bDrawIndicesInHandleStruct, " group='Display'");
#define HandleDispCount 3
		TwEnumVal handleDispEV[HandleDispCount] = { { WithDepth, "With Depth" }, { NoDepth, "No Depth" }, { NoDisp, "None" } };
		TwType handleDispT = TwDefineEnum("Handle Display", handleDispEV, HandleDispCount);
		bar->TwAddVarCB("Handle Display", handleDispT, SetHandleDispCB, GetHandleDispCB, this, "group='Display'");
		bar->TwAddVarRW("Handle Color", TW_TYPE_COLOR3F,
			&handle_color,
			" group='Display'"
			" label='Handle color' ");
		bar->TwAddVarRW("Handle Radius", TW_TYPE_DOUBLE, &handle_radius, " group='Display' label='Handle Radius' min=0.01");
		bar->TwAddVarRW("Handle Radius Ratio of Passive Point Handle", TW_TYPE_DOUBLE, &ratio_of_passive_point_handle_radius, " group='Display' min=0.0 step=0.1");
		bar->TwAddVarRW("Draw Handle Points", TW_TYPE_BOOLCPP, &bDrawHandlePoints, " group='Display'");
		bar->TwAddVarRW("Draw Region Handle Centers", TW_TYPE_BOOLCPP, &bDrawRegionHandleCenters, " group='Display'");
		bar->TwAddVarRW("Draw Region Handles", TW_TYPE_BOOLCPP, &bDrawRegionHandles, " group='Display'");
		bar->TwAddVarRW("Draw Wireframe", TW_TYPE_BOOLCPP, &bDrawWireframe, " group='Display'");
		bar->TwAddVarRW("Draw 2D Trans of Handles", TW_TYPE_BOOLCPP, &bDrawHandleFrames2D, " group='Display'");
		bar->TwAddVarRW("Draw 3D Trans of Handles", TW_TYPE_BOOLCPP, &bDrawHandleFrames3D, " group='Display'");

		bar->TwAddButton("Mark Region Handles on Mesh", mark_dialog_region_handle_in_mesh, this, " group='Display'");
		bar->TwAddVarRW("Auto Marked when Selected", TW_TYPE_BOOLCPP, &auto_marked_when_selected, " group='Display'");
		bar->TwAddVarRW("Region Handle Color", TW_TYPE_COLOR3F, &UNSELECTED_REGION_COLOR, " group='Display'");

		bar->TwAddVarRW("Min Index to Draw", TW_TYPE_INT32, &min_handle_index_to_draw, " group='Display'");
		bar->TwAddVarRW("Max Index to Draw", TW_TYPE_INT32, &max_handle_index_to_draw, " group='Display'");
		/**********************  Transform  **************************/
		bar->TwAddVarRW("Positionally Rotate", TW_TYPE_BOOLCPP, &only_positional_rotate, "group='Transform'");
		bar->TwAddVarRW("Rotate in 3D", TW_TYPE_BOOLCPP, &is3DRotation, " group='Transform'");
#define RotateTypeCount 3
		TwEnumVal rotateTypeEV[RotateTypeCount] = { { RotateAroundAvg, "Around Average" }, { RotateAroundFirstPoint, "Around First Point" }, { RotateAroundCursor, "Around Cursor" } };
		TwType rotateTypeT = TwDefineEnum("Rotate Type", rotateTypeEV, RotateTypeCount);
		bar->TwAddVarCB("Rotate Around", rotateTypeT, SetRotateTypeCB, GetRotateTypeCB, this, " group='Transform'");

		//bar->TwAddVarRW( "Rotate Handles", TW_TYPE_QUAT4F, &h_Rotation, 
		//	" group='Transform'"
		//	" label='Rotate Handles' open help='Change the object orientation.' ");
		bar->TwAddVarCB("Rotation", TW_TYPE_QUAT4F, SetRotationCB, GetRotationCB, this, " group='Transform'");
		bar->TwAddVarCB("Scaling", TW_TYPE_DOUBLE, SetScaleCB, GetScaleCB, this, " min=0.05 max=50 step=0.1 help='Scale the object (1=original size).' group='Transform'");

		bar->TwAddVarCB("Rotate in 2D", TW_TYPE_DOUBLE, SetRotation2DCB, GetRotation2DCB, this, " group='Transform' step=0.01");
		bar->TwAddButton("Reset Rotation", reset_dialog_rotation, this,
			" group='Transform'"
			" label='Reset Rotation'");


		/*********************** Constraints *********************/

		//bar->TwAddVarRW("LBS: Trans or Pos", TW_TYPE_BOOLCPP, &b_constrain_lbs_affine_trans, " group='Constraints'");

		bar->TwAddButton("Set Transformation Constrained (for all non-point handle)", set_dialog_handle_trans_constrained, this, " group='Constraints'");
		bar->TwAddButton("Set Position Constrained (for all non-point handle)", set_dialog_handle_pos_constrained, this, " group='Constraints'");
		bar->TwAddButton("Load Boolean Constrained File (for all non-point handle)", load_dialog_handle_constrained, this, " group='Constraints'");

		/************************* Editing ***********************/
		bar->TwAddVarRW("On-fly Editing", TW_TYPE_BOOLCPP, &is_during_alt_sliding, " group='Cursor'");

		/************************* Cursor ***********************/

		bar->TwAddVarRW("Enable Cursor", TW_TYPE_BOOLCPP, &enable_cursor, " group='Cursor'");
#define CursorUsageCount 3
		TwEnumVal cursorUsageEV[CursorUsageCount] = { { AsInsertedHandle, "AsInsertedHandle" }, { AsInsertedPoint, "AsInsertedPoint" }, { AsRotationCenter, "AsRotationCenter" } };
		TwType cursorUsageT = TwDefineEnum("Cursor Usage", cursorUsageEV, CursorUsageCount);
		bar->TwAddVarCB("Cursor Usage", cursorUsageT, SetCursorUsageCB, GetCursorUsageCB, this, " group='Cursor'");
		bar->TwAddVarRW("Draw Cursor", TW_TYPE_BOOLCPP, &bDrawRayTracedCursor, " group='Cursor'");
		bar->TwAddButton("Set from Points Set 1st Point", set_cursor_from_PS_CB, this, " group='Cursor'");

		/********************** Points Set **********************/

		bar->TwAddButton("Load Points Set", load_dialog_points_set, this, " group='Points Set'");
		bar->TwAddButton("Save Points Set", save_dialog_points_set, this, " group='Points Set'");

		bar->TwAddButton("Add Cursor to Set", add_cursor_to_points_setCB, this, " group='Points Set'");
		bar->TwAddButton("Clear the Set", clear_points_setCB, this, " group='Points Set'");

		bar->TwAddVarRW("Skeleton Line: Try", TW_TYPE_BOOLCPP, &try_skeleton, " group='Points Set'");
		bar->TwAddVarRW("Skeleton: Try", TW_TYPE_BOOLCPP, &try_skeleton_volume, " group='Points Set'");
		bar->TwAddVarRW("Skeleton: Number Segment", TW_TYPE_INT32, &num_seg_try_skeleton, " group='Points Set' min=1");
		bar->TwAddVarRW("Skeleton: Sample per Segment", TW_TYPE_INT32, &num_per_seg_try_skeleton, " group='Points Set' min=1");
		bar->TwAddVarRW("Skeleton: Ring", TW_TYPE_INT32, &ske_ring, " group='Points Set' min=1");
		bar->TwAddVarRW("Skeleton: Radius", TW_TYPE_DOUBLE, &ske_radius, " group='Points Set'");
		bar->TwAddVarRW("Skeleton: Cylinder or Ellipsoid", TW_TYPE_BOOLCPP, &ske_cylinder_or_ellipsoid, " group='Points Set'");
		bar->TwAddVarRW("Skeleton: Dim", TW_TYPE_INT32, &ske_dim, " group='Points Set' min=2 max=3");
		bar->TwAddVarRW("Skeleton: Symmetric", TW_TYPE_BOOLCPP, &with_symmetric, " group='Points Set'");
		/************************* Lasso ***********************/

		bar->TwAddVarRW("Lasso: move in plane", TW_TYPE_BOOLCPP, &move_in_plane, " group='Lasso'");
#define LassoUsageCount 5
		TwEnumVal lassoUsageEV[LassoUsageCount] = { { Lasso_Regular, "Regular" },
		{ Lasso_AddActiveHandle, "AddActiveHandle" },
		{ Lasso_RemoveActiveHandle, "RmActiveHandle" },
		{ Lasso_AddPassiveHandle, "AddPassiveHandle" },
		{ Lasso_RemovePassiveHandle, "RmPassiveHandle" }
		};
		TwType lassoUsageT = TwDefineEnum("Lasso Usage", lassoUsageEV, LassoUsageCount);
		bar->TwAddVarCB("Lasso Usage", lassoUsageT, SetLassoUsageCB, GetLassoUsageCB, this, " group='Lasso'");

		/*********************** KeyFraming **********************/

		bar->TwAddVarCB("Turn on Keyframing", TW_TYPE_BOOLCPP, SetTurnOnKeyFramingCB, GetTurnOnKeyFramingCB, this, " group='KeyFraming'");
		bar->TwAddButton("Load Keyframing Config", load_dialog_keyframing_config, this, " group='KeyFraming'");

	}

	double max_z = -1e10;
	double min_z = 1e10;
	for (int i = 0; i<m_preview->GetMainMesh().vertices->rows(); i++)
	{
		if ((*m_preview->GetMainMesh().vertices)(i, 2)>max_z)
		{
			max_z = (*m_preview->GetMainMesh().vertices)(i, 2);
		}
		if ((*m_preview->GetMainMesh().vertices)(i, 2)<min_z)
		{
			min_z = (*m_preview->GetMainMesh().vertices)(i, 2);
		}
	}
	if (max_z<min_z + 0.0000001)
	{
		is3DRotation = false;
	}
	else
	{
		//is3DRotation = true;
		is3DRotation = false;
	}

	init_picking();
	//vertices_moved(false);
}

bool HandlePluginBase::Serialize(tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element)
{
	return igl::save_ReAntTweakBar(bar, doc);
}

bool HandlePluginBase::Deserialize(tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element)
{
	return igl::load_ReAntTweakBar(bar, doc);
}

bool HandlePluginBase::keyDownEvent(unsigned char key, int modifiers, int mouse_x, int mouse_y)
{

	switch (key)
	{
	case '1':
	{
		//Usage 1
		//h_Rotation_2D += 0.01;
		rotate_2d_speed += 0.002;
		//
		return true;
	}
	break;

	case '2':
	{
		//Usage 1
		//h_Rotation_2D -= 0.03;
		rotate_2d_speed -= 0.002;
		//
		return true;
	}
	break;
	}

	return false;
}

#include <GL_helper.h>
void HandlePluginBase::push_gl_settings()
{
	float mat[4 * 4];
	ConvertQuaternionToMatrix(m_preview->camera.g_Rotation, mat);
	float scale[3];
	scale[0] = scale[1] = scale[2] = m_preview->camera.g_Zoom * m_preview->camera.zoom;
	float trans[3];
	trans[0] = m_preview->camera.g_Translation[0];
	trans[1] = m_preview->camera.g_Translation[1];
	trans[2] = m_preview->camera.g_Translation[2];
	bool enable_depth = true;


	push_settings(mat, scale, trans, enable_depth);
}

void HandlePluginBase::pop_gl_settings()
{
	bool enable_depth = true;
	pop_settings(enable_depth);
}

bool HandlePluginBase::mouseDownEvent(int mouse_x, int mouse_y, int button, int modifiers)
{
	if (!enable_mouse)
		return false;
	if (modifiers == Preview3D::CTRL)
		return false;
	bool any_response = false;
	if (m_meshIsLoaded)
	{
		push_gl_settings();

		{// lasso 
			bool callLassoDown = false;
			int inserted_index = -1;
			any_response |= lasso.mouseDown(
				mouse_x, mouse_y,
				modifiers,
				handleStructure.centerPos(),
				callLassoDown,
				inserted_index
				);
			if (callLassoDown)
			{
				switch (modifiers)
				{
				case Preview3D::ALT:
					lassoDownAlt(mouse_x, mouse_y, inserted_index);
					break;
				case Preview3D::SHIFT:
					lassoDownShift();
					break;
				case Preview3D::CTRL:
					break;
				default:
					lassoDownNone(mouse_x, mouse_y, inserted_index);
				}
			}

			if (is_during_alt_sliding)
			{
				BeginChangeHandlesOnFly();
			}
		}
		{// lasso 2 
			bool callLassoDown = false;
			int inserted_index = -1;
			any_response |= lasso2.mouseDown(
				mouse_x, mouse_y,
				modifiers,
				pointsSet.toMat(),
				callLassoDown,
				inserted_index
				);
			if (callLassoDown)
			{
				switch (modifiers)
				{
				case Preview3D::ALT:
					lasso2DownAlt(mouse_x, mouse_y);
					break;
				case Preview3D::SHIFT:
					lasso2DownShift();
					break;
				case Preview3D::CTRL:
					break;
				default:
					lasso2DownNone(mouse_x, mouse_y, inserted_index);
				}
			}
		}

		if (modifiers == Preview3D::ALT)
		{
			cursorDown(mouse_x, mouse_y);
		}

		pop_gl_settings();
	}
	return any_response;
}

bool HandlePluginBase::mouseUpEvent(int mouse_x, int mouse_y, int, int modifiers)
{
	if (!enable_mouse)
		return false;
	if (modifiers == Preview3D::CTRL)
		return false;
	bool any_response = false;
	if (m_meshIsLoaded)
	{
		push_gl_settings();

		{//lasso
			bool callLassoUp = false;
			bool callLassoMove = false;
			std::vector<int> indics_inside_lasso;
			bool shouldMove = lasso.is_activate();
			any_response |= lasso.mouseUp(
				mouse_x, mouse_y,
				handleStructure.centerPos(),
				callLassoUp,
				callLassoMove,
				indics_inside_lasso
				);
			if (shouldMove) lassoMove(lasso.translation);
			if (callLassoUp) lassoUp(indics_inside_lasso);


			if (is_during_alt_sliding)
			{
				Eigen::VectorXi IH(select_handle_list.size());
				Eigen::MatrixXd PH(select_handle_list.size(), 3);
				//const Eigen::MatrixXd & vertices_to_search = *m_preview->vertices;
				for (int i = 0; i < IH.rows(); i++)
				{
					IH(i) = select_handle_list[i];
					const Eigen::MatrixXd Pi = handleStructure.all_handle_list[IH(i)].CenterPos();
					PH(i, 0) = Pi(i, 0); //vertices_to_search(IH(i), 0);
					PH(i, 1) = Pi(i, 1);
					PH(i, 2) = Pi(i, 2);
				}
				if (IH.rows()>0)
				{
					slide_existing_handles_on_fly(PH, IH);
					EndChangeHandlesOnFly(true);
					is_during_alt_sliding = false;
				}

			}
		}
		{//lasso2
			bool callLassoUp = false;
			bool callLassoMove = false;
			std::vector<int> indics_inside_lasso;
			bool shouldMove = lasso2.is_activate();
			any_response |= lasso2.mouseUp(
				mouse_x, mouse_y,
				pointsSet.toMat(),
				callLassoUp,
				callLassoMove,
				indics_inside_lasso
				);
			if (shouldMove) lasso2Move(lasso2.translation);
			if (callLassoUp) lasso2Up(indics_inside_lasso);
		}


		pop_gl_settings();
	}
	return any_response;
}

bool HandlePluginBase::mouseMoveEvent(int mouse_x, int mouse_y)
{
	if (!enable_mouse)
		return false;
	bool any_response = false;
	if (m_meshIsLoaded)
	{
		push_gl_settings();

		{//lasso
			bool callLassoMove = false;
			bool shouldMove = lasso.is_activate();
			any_response |= lasso.mouseMove(mouse_x, mouse_y, callLassoMove, move_in_plane);
			if (shouldMove) lassoMove(lasso.translation);
		}
		{//lasso2
			bool callLassoMove = false;
			bool shouldMove = lasso2.is_activate();
			any_response |= lasso2.mouseMove(mouse_x, mouse_y, callLassoMove);
			if (shouldMove) lasso2Move(lasso2.translation);
		}

		pop_gl_settings();
	}
	return any_response;
}

bool HandlePluginBase::mouseScrollEvent(int mouse_x, int mouse_y, float delta)
{
	if (!enable_mouse)
		return false;
	return false;
}

void HandlePluginBase::preDraw(int currentTime)
{
	//h_Rotation_2D += rotate_2d_speed;


	bool is_rotated =
		h_old_Rotation[0] != h_Rotation[0] ||
		h_old_Rotation[1] != h_Rotation[1] ||
		h_old_Rotation[2] != h_Rotation[2] ||
		h_old_Rotation[3] != h_Rotation[3] ||
		h_old_Rotation_2D != h_Rotation_2D;

	bool is_scaled = old_handle_scale != handle_scale;


	if (is_rotated)
	{

	}

	//if (only_positional_rotate)
	//{
	//	positional_rotate_selected_handles(h_Rotation);
	//	//3D rotate_selected_handles_around(h_Rotation, false);
	//	//2D rotateSelectedHandlesAroundAxis(v,false);
	//}
	//else
	//{
	//	rotate_selected_handles(h_Rotation);
	//	//write_quarterion_into_selected_handles(h_Rotation);
	//}

	if (turn_on_handle_key_frame)
	{
		update_keyframing_skinning();
	}

	//if(is_scaled)
	//{
	//	scale_selected_handles(handle_scale);
	//}
	//if(is_rotated || is_scaled)
	//{
	//	DeformSkinning::GetReference().SetUpdateSkinning();
	//}
	//for(int i=0; i<4; i++)
	//{
	//	h_old_Rotation[i] = h_Rotation[i];
	//}
	//old_handle_scale = handle_scale;
	h_old_Rotation_2D = h_Rotation_2D;

}

#include <skeleton_sampling.h>

void HandlePluginBase::postDraw(int currentTime)
{

#ifdef DEBUG_PLUGINS_SEQUENTIAL_ORDER
	printf("DEBUG_PLUGINS_SEQUENTIAL_ORDER: HandlePluginBase Draw Handles.\n");
#endif

	bool enable_depth = handleDispType != NoDepth;

	drawHandles();

	if (bDrawHandleFrames2D)
	{// draw frame at each handle
		drawFrames(2);
	}
	if (bDrawHandleFrames3D)
	{// draw frame at each handle
		drawFrames(3);
	}
	if (bDrawIndicesInMesh)
	{
		drawIndicesInMesh();
	}
	if (bDrawIndicesInHandleStruct)
	{
		drawIndicesInHandleStruct();
	}

	if (bDrawRayTracedCursor)
	{
		drawRayTracedPoint();
	}
	if (true)// points set
	{
		drawPointsSet();
	}

	// Then goes 2D drawing
	lasso.draw();
}

/******************************************************************/
/*******************       Set and Get         ********************/
/******************************************************************/

void HandlePluginBase::SetHandleDisp(HandleDispType ht)
{
	handleDispType = ht;
}

HandleDispType HandlePluginBase::GetHandleDisp() const
{
	return handleDispType;
}

void HandlePluginBase::SetRotateType(RotateType rt)
{
	rotate_type = rt;
}

RotateType HandlePluginBase::GetRotateType() const
{
	return rotate_type;
}

void HandlePluginBase::SetCursorUsage(CursorUsageType cut)
{
	cursorUsageType = cut;
}

CursorUsageType HandlePluginBase::GetCursorUsage() const
{
	return cursorUsageType;
}

void HandlePluginBase::SetLassoUsage(LassoUsageType lut)
{
	lassoUsageType = lut;
}

LassoUsageType HandlePluginBase::GetLassoUsage() const
{
	return lassoUsageType;
}

int HandlePluginBase::get_sample_handles_number()
{
	return sample_handles_number;
}

void HandlePluginBase::set_sample_handles_number(int value)
{
	sample_handles_number = value;
}

/******************************************************************/
/*******************       Antweakbar         *********************/
/******************************************************************/

void TW_CALL HandlePluginBase::open_dialog_handle_group(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_open_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<HandlePluginBase *>(clientData)->load_handle_group_from_file(fname);
}

void TW_CALL HandlePluginBase::save_dialog_handle_group(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_save_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<HandlePluginBase *>(clientData)->save_handle_group_to_file(fname);
}

void TW_CALL HandlePluginBase::load_dialog_handle_struct(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_open_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<HandlePluginBase *>(clientData)->load_handle_struct_from_file(fname);
}

void TW_CALL HandlePluginBase::save_dialog_handle_struct(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_save_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<HandlePluginBase *>(clientData)->save_handle_struct_to_file(fname);
}

void TW_CALL HandlePluginBase::load_dialog_handle_struct_rotation_center(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_open_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<HandlePluginBase *>(clientData)->load_rotation_center_from_file(fname);
}

void TW_CALL HandlePluginBase::open_dialog_handles(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_open_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<HandlePluginBase *>(clientData)->load_handles_from_file(fname);
}

void TW_CALL HandlePluginBase::load_dialog_handles_from_mesh(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_open_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<HandlePluginBase *>(clientData)->load_handle_mesh_from_file(fname);
}

void TW_CALL HandlePluginBase::save_dialog_handles_to_mesh(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_save_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<HandlePluginBase *>(clientData)->save_handle_mesh_to_file(fname);
}

void TW_CALL HandlePluginBase::move_dialog_handle_onto_mesh(void *clientData)
{
	static_cast<HandlePluginBase *>(clientData)->move_handle_onto_mesh();
}

void TW_CALL HandlePluginBase::open_dialog_pose_trans(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_open_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<HandlePluginBase *>(clientData)->load_pose_trans_from_file(fname);
}

void TW_CALL HandlePluginBase::save_dialog_pose_trans(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_save_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<HandlePluginBase *>(clientData)->save_pose_trans_to_file(fname);
}

void TW_CALL HandlePluginBase::open_dialog_pose_pos(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_open_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<HandlePluginBase *>(clientData)->load_pose_pos_from_file(fname);
}

void TW_CALL HandlePluginBase::save_dialog_pose_pos(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_save_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<HandlePluginBase *>(clientData)->save_pose_pos_to_file(fname);
}

void TW_CALL HandlePluginBase::open_dialog_pose_disp(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_open_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<HandlePluginBase *>(clientData)->load_pose_disp_from_file(fname);
}

void TW_CALL HandlePluginBase::save_dialog_pose_disp(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_save_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<HandlePluginBase *>(clientData)->save_pose_disp_to_file(fname);
}

void TW_CALL HandlePluginBase::open_dialog_pose_rot(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_open_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<HandlePluginBase *>(clientData)->load_pose_rot_from_file(fname);
}

void TW_CALL HandlePluginBase::save_dialog_pose_rot(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_save_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<HandlePluginBase *>(clientData)->save_pose_rot_to_file(fname);
}

void TW_CALL HandlePluginBase::add_dialog_handle_from_picking(void *clientData)
{
	static_cast<HandlePluginBase *>(clientData)->add_region_handle_from_picking();
}

void TW_CALL HandlePluginBase::add_dialog_handle_from_pointset(void *clientData)
{
	static_cast<HandlePluginBase *>(clientData)->add_region_handle_from_pointset();
}

void TW_CALL HandlePluginBase::add_dialog_boneedge_handle(void *clientData)
{
	static_cast<HandlePluginBase *>(clientData)->add_boneedge_handle();
}

void TW_CALL HandlePluginBase::add_dialog_some_purepoint_handle(void *clientData)
{
	static_cast<HandlePluginBase *>(clientData)->add_some_purepoint_handle_from_PS();
}

void TW_CALL HandlePluginBase::add_dialog_handles(void *clientData)
{
	static_cast<HandlePluginBase *>(clientData)->add_point_handle();
}

void TW_CALL HandlePluginBase::clear_dialog_handles(void *clientData)
{
	static_cast<HandlePluginBase *>(clientData)->clear_handles();
}

void TW_CALL HandlePluginBase::print_dialog_handles(void *clientData)
{
	static_cast<HandlePluginBase *>(clientData)->print_handles();
}

void TW_CALL HandlePluginBase::save_dialog_selected_handles(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_save_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<HandlePluginBase *>(clientData)->save_selected_handles(fname);
}

void TW_CALL HandlePluginBase::send_dialog_selected_handles_to_MATLAB(void *clientData)
{
	static_cast<HandlePluginBase *>(clientData)->send_selected_handles_to_MATALB();
}

void TW_CALL HandlePluginBase::sample_dialog_handles(void *clientData)
{
	static_cast<HandlePluginBase *>(clientData)->sample_handles();
}

void TW_CALL HandlePluginBase::delete_dialog_handle(void *clientData)
{
	static_cast<HandlePluginBase *>(clientData)->delete_handles(static_cast<HandlePluginBase *>(clientData)->handle_to_delete);
}

void TW_CALL HandlePluginBase::append_dialog_handle_struct_from_file(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_open_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<HandlePluginBase *>(clientData)->append_handle_struct_from_file(fname);
}

void TW_CALL HandlePluginBase::save_dialog_handles(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_save_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<HandlePluginBase *>(clientData)->save_handles_to_file(fname);
}

void TW_CALL HandlePluginBase::save_dialog_active(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_save_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<HandlePluginBase *>(clientData)->save_active(fname);
}

void TW_CALL HandlePluginBase::load_dialog_active(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_open_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<HandlePluginBase *>(clientData)->load_active(fname);
}

void TW_CALL HandlePluginBase::reset_dialog_handles(void *clientData)
{
	static_cast<HandlePluginBase *>(clientData)->reset_handles();
}

void TW_CALL HandlePluginBase::reset_dialog_rotation(void *clientData)
{
	static_cast<HandlePluginBase *>(clientData)->reset_rotation();
}

void TW_CALL HandlePluginBase::set_sample_handles_numberCB(const void *value, void *clientData)
{
	static_cast<HandlePluginBase *>(clientData)->set_sample_handles_number(*static_cast<const int *>(value));
}

void TW_CALL HandlePluginBase::get_sample_handles_numberCB(void *value, void *clientData)
{
	*static_cast<int *>(value) = static_cast<HandlePluginBase *>(clientData)->get_sample_handles_number();
}

// Modifying
void TW_CALL HandlePluginBase::modify_dialog_add_region_handle_from_picking(void *clientData)
{
	static_cast<HandlePluginBase *>(clientData)->modify_add_region_handle_from_picking();
}

void TW_CALL HandlePluginBase::load_dialog_keyframing_config(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_open_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<HandlePluginBase*>(clientData)->load_keyframing_config(fname);
}

// Constraint
void TW_CALL HandlePluginBase::load_dialog_handle_constrained(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_open_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<HandlePluginBase *>(clientData)->load_handle_constrained_type_from_file(fname);
}

// Points Set
void TW_CALL HandlePluginBase::add_cursor_to_points_setCB(void *clientData)
{
	static_cast<HandlePluginBase*>(clientData)->add_cursor_to_points_set();
}

void TW_CALL HandlePluginBase::clear_points_setCB(void *clientData)
{
	static_cast<HandlePluginBase*>(clientData)->clear_points_set();
}

void TW_CALL HandlePluginBase::load_dialog_points_set(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_open_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<HandlePluginBase *>(clientData)->load_points_set_from_file(fname);
}

void TW_CALL HandlePluginBase::save_dialog_points_set(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_save_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<HandlePluginBase *>(clientData)->save_points_set_to_file(fname);
}

// Display
void TW_CALL HandlePluginBase::mark_dialog_region_handle_in_mesh(void *clientData)
{
	static_cast<HandlePluginBase*>(clientData)->mark_region_handle_in_mesh();
}

/******************************************************************/
/*******************          Handles         *********************/
/******************************************************************/

void HandlePluginBase::recover_M2d_from_M3d(const Eigen::MatrixXd& M3d, Eigen::MatrixXd& M2d)
{
	// The decode of Map2Dto3D should be the same as encode of Map2Dto3D
	int cols_2d = 0;
	for (int i = 0; i<Map2Dto3D.rows(); i++)
	{
		switch (Map2Dto3D(i, 1))
		{
		case 1:
			cols_2d += 1;
			break;
		case 4:
			cols_2d += 3;
			break;
		}
	}

	M2d.resize(M3d.rows(), cols_2d);

	int current_cols_2d = 0;
	for (int i = 0; i<Map2Dto3D.rows(); i++)
	{
		switch (Map2Dto3D(i, 1))
		{
		case 1:
		{
			int s = Map2Dto3D(i, 0);
			M2d.col(current_cols_2d) = M3d.col(s);
			current_cols_2d += 1;
			break;
		}
		case 4:
		{
			int s = Map2Dto3D(i, 0);
			M2d.col(current_cols_2d + 0) = M3d.col(s + 0);
			M2d.col(current_cols_2d + 1) = M3d.col(s + 1);
			M2d.col(current_cols_2d + 2) = M3d.col(s + 3);// Be careful with this!
			current_cols_2d += 3;
			break;
		}
		}
	}
}

// This should only be used for Handle operations that requires rebuild/update all structures
void HandlePluginBase::update_handles_from_list()
{
	handleStructure.update_from_list();
}

// Handle operations that requires rebuild all structures

void HandlePluginBase::move_handle_onto_mesh(bool onto_deformed_mesh)
{
	const Eigen::MatrixXd * vertices_to_search = (onto_deformed_mesh) ? m_preview->GetMainMesh().vertices : &(m_preview->GetMainMesh().rest_vertices);
	const Eigen::MatrixXd * vertices_rest_position = &(m_preview->GetMainMesh().rest_vertices);
	handleStructure.attach_onto_mesh_vertices(vertices_to_search, vertices_rest_position);
	update_handles_from_list();
}

void HandlePluginBase::mark_region_handle_in_mesh()
{
	Eigen::MatrixXd color(1, 3);
	for (int i = 0; i < all_handle_list.size(); i++)
	{
		if (i<min_handle_index_to_draw || i >= max_handle_index_to_draw)
		{
			continue;
		}


		if (all_handle_list[i].type() != HandleTypePoint)
		{
			if (all_handle_list[i].selected)
			{
				color << SELECTED_REGION_COLOR[0], SELECTED_REGION_COLOR[1], SELECTED_REGION_COLOR[2];
			}
			else
			{
				color << UNSELECTED_REGION_COLOR[0], UNSELECTED_REGION_COLOR[1], UNSELECTED_REGION_COLOR[2];
			}


			bool mark_per_vertex_color = false;
			if (mark_per_vertex_color)
			{
				m_preview->GetMainMesh().set_vertex_colors(m_preview->material.g_MatDiffuse, all_handle_list[i].IndexInMesh(), color);
			}
			else // mark per face color
			{
				m_preview->GetMainMesh().set_face_colors_from_vertex(all_handle_list[i].IndexInMesh(), color);
			}
		}
	}
}

bool HandlePluginBase::load_handle_group_from_file(const char *group_file_name)
{
	if (group_file_name != NULL)
	{
		Eigen::VectorXi hg;
		if (igl::readDMAT(group_file_name, hg))
		{
			return handleStructure.set_group(hg);
		}
		printf("Format error with the handle group file!\n");
	}

	printf("No handle group file provided, set each handle to be a unique group.\n");
	handleStructure.set_unique_group();

	return true;
}

bool HandlePluginBase::build_up_handle_structure(const Eigen::MatrixXd& H, Eigen::VectorXi& HG)
{
	handleStructure.build_old(default_pure_point, H, HG);

	// No need to set Handles=H, alrady set in update_handles_from_list();
	ori_Handles = H;
	temp_Handles = H;

	do_this_when_handle_changed();

	return true;
}

bool HandlePluginBase::load_handles_from_file(const char* handle_file_name)
{
	Eigen::MatrixXd handle_file;
	igl::readDMAT(handle_file_name, handle_file);
	assert(handle_file.cols() == 3);

	//Handles = handle_file.leftCols<3>();
	printf("Handle Matrix:(%d,%d)\n", handle_file.rows(), handle_file.cols());

	build_up_handle_structure(handle_file, HandleGroup);

	return true;
}

bool HandlePluginBase::load_handle_struct_from_file(const char * handle_sfile_name)
{
	Eigen::MatrixXd VV;
	Eigen::MatrixXi TT, FF, EE, PP, BE, CF;
	std::vector<std::vector<Eigen::VectorXi>> GG;

	readHANDLE(handle_sfile_name, VV, TT, FF, EE, PP, BE, CF, GG);

	//build_up_handle_structure(VV,HandleGroup);

	/****/

	handleStructure.build(VV, TT, FF, EE, PP, BE, CF, GG);

	// No need to set Handles=H, alrady set in update_handles_from_list();
	//ori_Handles = H;
	//temp_Handles = H;

	do_this_when_handle_changed();
	/****/

	return true;
}

bool HandlePluginBase::save_handle_struct_to_file(const char * handle_sfile_name)
{
	return handleStructure.writeToFile(handle_sfile_name);
}

bool HandlePluginBase::export_handle_state_to_file(const char * fname)
{
	return false;
}

bool HandlePluginBase::load_rotation_center_from_file(const char *fname)
{
	Eigen::MatrixXd RC;
	if (!igl::readDMAT(fname, RC))
	{
		printf("Error: fails to load the rotation center file!\n");
		return false;
	}
	if (RC.rows()<1 || RC.cols() != 4)
	{
		printf("Error: the dimension of rotation center file (%d, %d) is incorrect!\n", RC.rows(), RC.cols());
		return false;
	}

	Eigen::MatrixXd RCs = RC.leftCols<3>();// first 3 columns are positions
	Eigen::MatrixXi Is = RC.col(3).cast<int>();// last column is indices

	handleStructure.set_rotation_centers(RCs, Is);

	return true;
}

template <typename DerivedV, typename DerivedF, typename DerivedT>
inline bool read_mesh(const char* mesh_file_name, Eigen::PlainObjectBase<DerivedV>& Handles, Eigen::PlainObjectBase<DerivedT>& HandleTets, Eigen::PlainObjectBase<DerivedF>& HandleFaces)
{
	std::string mesh_file_name_string = std::string(mesh_file_name);
	//filename = mesh_file_name;
	//clear_mesh();

	//Eigen::MatrixXd* all_vertices;
	//Eigen::MatrixXi* all_faces;

	size_t last_dot = mesh_file_name_string.rfind('.');
	if (last_dot == std::string::npos)
	{
		// No file type determined
		printf("Error: No file extension found in %s\n", mesh_file_name);
		return false;
	}
	std::string extension = mesh_file_name_string.substr(last_dot + 1);
	//if(extension == "off" || extension =="OFF")
	//{
	//	if(!igl::readOFF(mesh_file_name_string, *vertices, *faces))
	//	{
	//		return false;
	//	}
	//}else 
	if (extension == "obj" || extension == "OBJ")
	{
		if (!(igl::readOBJ(mesh_file_name_string, Handles, HandleFaces)))//, *corner_normals, *fNormIndices, *texCoords, *fTexIndices)))
		{
			return false;
			//vector< vector< IndexType > > faces_poly;
			//if(igl::readOBJPoly(mesh_file_name_string, *vertices, faces_poly, *corner_normals, *fNormIndices, *texCoords, *fTexIndices))
			//{
			//	// Triangulate all faces
			//	vector<vector<int> > tri;
			//	vector< Vector3 > tri_normals;

			//	for (unsigned i=0; i<faces_poly.size(); ++i)
			//	{
			//		vector<IndexType> f = faces_poly[i];
			//		int iter = f.size()-2;

			//		Vector3 e1 = vertices->row(f[0+2]) - vertices->row(f[0]);
			//		Vector3 e2 = vertices->row(f[0+1]) - vertices->row(f[0]);
			//		Vector3 n = e2.cross(e1);
			//		n.normalize();

			//		for (int j=0; j<iter; ++j)
			//		{
			//			vector<int> t(3);
			//			t[0] = f[0];
			//			t[1] = f[1];
			//			t[2] = f[2];
			//			f.erase(f.begin()+1);
			//			tri.push_back(t);
			//			tri_normals.push_back(n);
			//		}
			//		assert(f.size() == 2);

			//	}

			//	faces->resize(tri.size(),3);
			//	face_normals->resize(tri.size(),3);
			//	for (unsigned i=0; i < tri.size();++i)
			//	{
			//		(*faces)(i,0) = tri[i][0];
			//		(*faces)(i,1) = tri[i][1];
			//		(*faces)(i,2) = tri[i][2];
			//		face_normals->row(i) = tri_normals[i];
			//	}

			//	// Add the polygonal edges
			//	//        lines.clear();
			//	for (unsigned i=0; i<faces_poly.size(); ++i)
			//	{
			//		vector<IndexType> f = faces_poly[i];
			//		for (unsigned j=0; j<f.size(); ++j)
			//		{
			//			vector<double> t(9);
			//			t[0] = (*vertices)(f[j  ],0);
			//			t[1] = (*vertices)(f[j  ],1);
			//			t[2] = (*vertices)(f[j  ],2);
			//			t[3] = (*vertices)(f[(j+1)%f.size()],0);
			//			t[4] = (*vertices)(f[(j+1)%f.size()],1);
			//			t[5] = (*vertices)(f[(j+1)%f.size()],2);
			//			t[6] = 1; t[7] = 0; t[8] = 0;
			//			lines.push_back(t);
			//		}
			//	}
			//}
			//else
			//	return false;
		}
	}
	//else if (extension == "mp" || extension =="MP")
	//{
	//	delete_on_exit = true;
	//	MatlabIO mio;
	//	mio.deserialize(mesh_file_name);
	//	if (mio.error)
	//	{
	//		return false;
	//	} else
	//	{
	//		vertices->resize(mio.V.size(),3);
	//		for (unsigned int i = 0; i <mio.V.size(); ++i)
	//			vertices->row(i) << mio.V[i][0], mio.V[i][1], mio.V[i][2];

	//		if (mio.F.size()==0)
	//			faces->resize(mio.F.size(),3);
	//		else
	//			faces->resize(mio.F.size(),mio.F[0].size());

	//		for (unsigned int i = 0; i <mio.F.size(); ++i)
	//			for(unsigned int j=0;j<mio.F[0].size();++j)
	//				(*faces)(i,j) = mio.F[i][j];

	//		vertex_colors->resize(mio.VC.size(),3);
	//		for (unsigned int i = 0; i <mio.VC.size(); ++i)
	//			vertex_colors->row(i) << mio.VC[i][0], mio.VC[i][1], mio.VC[i][2];

	//		vertex_property->resize(mio.VP.size(),1);
	//		for (unsigned int i = 0; i <mio.VP.size(); ++i)
	//			(*vertex_property)(i,0) = mio.VP[i][0];

	//		texCoords->resize(mio.TC.size(),3);
	//		for (unsigned int i = 0; i <mio.TC.size(); ++i)
	//			texCoords->row(i) << mio.TC[i][0], mio.TC[i][1];

	//		std::vector<vector<double> > tf = mio.TF;
	//		if (tf.size())
	//		{
	//			fUseTexture.clear();
	//			fUseTexture.resize(faces->rows(),true);
	//			for (int fi = 0; fi < faces->rows(); ++fi)
	//				for(int vit = 0; vit < faces->cols(); ++vit)
	//				{
	//					fUseTexture[fi] = fUseTexture[fi] && (tf[(*faces)(fi,vit)][0] >0)   ;
	//				}
	//		}

	//		face_colors->resize(mio.FC.size(),3);
	//		for (unsigned int i = 0; i <mio.FC.size(); ++i)
	//			face_colors->row(i) << mio.FC[i][0], mio.FC[i][1], mio.FC[i][2];

	//		face_property->resize(mio.FP.size(),1);
	//		for (unsigned int i = 0; i <mio.FP.size(); ++i)
	//			(*face_property)(i,0) = mio.FP[i][0];


	//		vertex_normals->resize(mio.VN.size(),3);
	//		for (unsigned int i = 0; i <mio.VN.size(); ++i)
	//			vertex_normals->row(i) << mio.VN[i][0], mio.VN[i][1], mio.VN[i][2];

	//		face_normals->resize(mio.FN.size(),3);
	//		for (unsigned int i = 0; i <mio.FN.size(); ++i)
	//			face_normals->row(i) << mio.FN[i][0], mio.FN[i][1], mio.FN[i][2];

	//		lines = mio.L;
	//		points = mio.P;
	//		textp = mio.TEXTP;
	//		texts = mio.TEXTS;

	//		if (vertex_property->rows() !=0)
	//		{
	//			show_isolines = true;

	//		}


	//		if (delete_on_exit)
	//			remove(mesh_file_name);
	//	}
	//}
	else if (extension == "mesh" || extension == "MESH")
	{//added by wangyu
		if (!igl::readMESH(mesh_file_name_string, Handles, HandleTets, HandleFaces))
		{
			return false;
		}
	}
	else
	{
		// unrecognized file type
		printf("Error: %s is not a recognized file type.\n", extension.c_str());
		return false;
	}

	//number_of_vertices = import_vertices->rows();
	//number_of_faces = import_faces->rows();

	//show_faces = true;
	//normals_type = PER_FACE;

	//is_compiled = false;
	//get_scale_and_shift_to_fit_mesh(vertices,zoom,g_Translation);
	//radius = 1/zoom;

	//if (face_normals->rows() == 0 || face_normals->cols() != 3)
	//	recompute_face_normals();

	//if (vertex_normals->rows() == 0  || vertex_normals->cols() != 3)
	//	recompute_vertex_normals();

	//std::vector<std::vector<IndexType> > vfi;
	//igl::vf(*vertices, *faces, vertex_to_faces, vfi);
}

bool HandlePluginBase::load_handle_mesh_from_file(const char* mesh_file_name)
{
	Eigen::MatrixXd handle_file;
	if (!read_mesh(mesh_file_name, handle_file, HandleTets, HandleFaces))
	{
		return false;
	}

	assert(handle_file.cols() == 3);

	//Handles = handle_file.leftCols<3>();
	printf("Handle Matrix:(%d,%d)\n", handle_file.rows(), handle_file.cols());

	build_up_handle_structure(handle_file, HandleGroup);

	return true;

	/* old code
	SetHandleConnectivity();
	printf("Handle Matrix:(%d,%d)\n",Handles.rows(),Handles.cols());
	assert(Handles.cols()==3);

	clear_selected_handles();
	clear_all_handles();
	for(int i=0; i<Handles.rows(); i++)
	{

	printf("Print Point(%f,%f,%f)\n",
	(double) Handles(i,0),
	(double) Handles(i,1),
	(double) Handles(i,2));

	add_all_handles(
	(double) Handles(i,0),
	(double) Handles(i,1),
	(double) Handles(i,2));

	}
	ori_Handles = Handles;
	temp_Handles = Handles;


	size_t last_dir_marker = mesh_file_name_string.rfind('\\');
	std::string mesh_file_name_only_string = mesh_file_name_string.substr(last_dir_marker+1);


	handleFrames.push_back(HandleFrame(mesh_file_name_only_string,Handles));
	current_handle_frame_index = handleFrames.size()-1;

	notify_deformers();

	PickingPlugin::GetReference().init_Picking();
	return true;*/
}

// Handle operations that requires only update all structures

bool HandlePluginBase::save_handle_group_to_file(const char* handle_file_name)
{
	igl::writeDMAT(handle_file_name, HandleGroup);
	return true;
}

bool HandlePluginBase::save_handles_to_file(const char* handle_file_name)
{
	igl::writeDMAT(handle_file_name, Handles);
	return true;
}

bool HandlePluginBase::save_handle_mesh_to_file(const char* mesh_file_name)
{
	igl::writeOBJ(mesh_file_name, Handles, HandleFaces);
	return true;
}

bool HandlePluginBase::clear_handles()
{
	Handles.resize(0, 3);
	ori_Handles = Handles;
	temp_Handles = Handles;
	clear_all_handles();
	selection_changed();

	do_this_when_handle_cleared();

	return true;
}

bool HandlePluginBase::print_handles()
{
	printf("Handle Matrix:\n");
	for (int i = 0; i<Handles.rows(); i++)
	{
		//printf("Print Point(%f,%f,%f)\n",			
		//	(double) Handles.coeffRef(i+Handles.rows()*0),
		//	(double) Handles.coeffRef(i+Handles.rows()*1),
		//	(double) Handles.coeffRef(i+Handles.rows()*2));
		printf("Print Point(%f,%f,%f)\n",
			(double)Handles(i, 0),
			(double)Handles(i, 1),
			(double)Handles(i, 2));
	}
	return true;
}

void HandlePluginBase::SetHandleConnectivity()
{
	if (HandleFaces.rows() != 0)
	{
		igl::adjacency_list(HandleFaces, vertex_to_vertices);
	}
	else
	{
		// Initialize vertices connectivity
		vertex_to_vertices.clear();
		vertex_to_vertices.resize(Handles.rows());
		for (int i = 0; i<Handles.rows(); i++)
		{
			vertex_to_vertices[i].clear();
		}
	}
}

//void HandlePluginBase::SetHandleConnectvityFromMesh()
//{
//	if (HandleFaces.rows()!=0)
//	{
//		igl::adjacency_list(HandleFaces, vertex_to_vertices);
//	}
//}

bool HandlePluginBase::load_active(const char* active_file_name)
{
	Eigen::MatrixXi active;
	if (!igl::readDMAT(active_file_name, active))
	{
		printf("Warning: cannot find active list file, set all handle to be active!\n");
		handleStructure.set_all_active();
		return false;
	}
	handleStructure.set_active(active);
	return true;
}

bool HandlePluginBase::save_active(const char* active_file_name)
{
	Eigen::VectorXi active;
	handleStructure.get_active(active);
	return igl::writeDMAT(active_file_name, active);
}

/******************************************************************/
/*******************  Handles Editing         *********************/
/******************************************************************/

// This is old codes and should be removed/disabled at some point
void HandlePluginBase::edit_insert_new_handles(const Eigen::MatrixXd& new_P, const bool in_same_group)
{
	handleStructure.edit_insert_new_handles(new_P, in_same_group);
}

bool HandlePluginBase::add_region_handle_from_pointset()
{
	const Eigen::MatrixXd P = pointsSet.toMat(); // this is the only difference from from_picking
	assert(P.cols() == 3);

	add_region_handle(P);

	return true;
}

#include <plane_symmetric.h>
bool HandlePluginBase::add_boneedge_handle()
{
	float diameter = std::min(m_preview->GetMainMesh().diameter*0.004, m_preview->GetMainMesh().avg_edge_length*0.3);
	Eigen::MatrixXd PS = pointsSet.toMat();
	Eigen::MatrixXd ske;
	if (PS.rows() >= 2)
	{
		Eigen::VectorXd s(3);
		s(0) = PS(0, 0);
		s(1) = PS(0, 1);
		s(2) = PS(0, 2);
		Eigen::VectorXd e(3);
		e(0) = PS(1, 0);
		e(1) = PS(1, 1);
		e(2) = PS(1, 2);
		volumetric_skeleton_sampling(
			s, e, ske_dim,
			num_seg_try_skeleton,
			num_per_seg_try_skeleton,
			ske_ring,
			ske_radius*diameter,
			ske,
			true,
			ske_cylinder_or_ellipsoid);
	}
	else
	{
		printf("Error: no bone edge to add!\n");
		return false;
	}


	add_region_handle(ske);
	if (with_symmetric)
	{
		Eigen::MatrixXd ske2 = plane_symmetric(ske);
		add_region_handle(ske2);
	}

	return true;
}

bool HandlePluginBase::add_some_purepoint_handle_from_PS()
{
	Eigen::MatrixXd PS = pointsSet.toMat();
	return add_some_purepoint_handle(PS);
}

bool HandlePluginBase::add_some_purepoint_handle(const Eigen::MatrixXd& PS)
{
	for (int i = 0; i < PS.rows(); i++)
	{
		handleStructure.edit_add_point(PS.row(i));
	}

	handleStructure.build();

	do_this_when_handle_changed();

	return true;
}

bool HandlePluginBase::add_point_handle(double x/* =0 */, double y/* =0 */, double z/* =0 */, int index/* -1 */)
{

	Eigen::MatrixXd P(1, 3);
	P << x, y, z;

	edit_insert_new_handles(P, true);
	const Eigen::MatrixXd handlefile = Handles.cast<double>();
	build_up_handle_structure(handlefile, HandleGroup);

	return true;
}

bool HandlePluginBase::add_region_handle(const Eigen::MatrixXd& RH)
{
#if 0
	edit_insert_new_handles(RH, true);
	// the old codes
	const Eigen::MatrixXd handlefile = Handles.cast<double>();
	build_up_handle_structure(handlefile, HandleGroup);
#else

	handleStructure.edit_add_boneedge(RH);
	handleStructure.build();

	do_this_when_handle_changed();

#endif

	return true;
}

#include <farthest_point_sampling.h>
bool HandlePluginBase::sample_handles()
{
	Eigen::MatrixXd * VERTICES_MATRIX = pointer_mesh_vertices();

	std::vector<Point3D> sampled_point_list;
	int seed_index_when_start_without_handle = 0;
	for (int i = 0; i<all_handle_list.size(); i++)
	{
		const Eigen::MatrixXd pos = all_handle_list[i].Pos();
		const Eigen::VectorXi index = all_handle_list[i].IndexInMesh();
		for (int k = 0; k < pos.rows(); k++)
		{
			sampled_point_list.push_back(Point3D(pos(k, 0), pos(k, 1), pos(k, 2), index(k)));
		}
		//sampled_point_list.push_back(Point3D(Handles(i,0),Handles(i,1),Handles(i,2),HandleIndices(i)));
	}
	int start_num = sampled_point_list.size();
	bool r = farthest_point_sampling(*VERTICES_MATRIX, seed_index_when_start_without_handle, sample_handles_number, sampled_point_list);

	Eigen::MatrixXd P(sampled_point_list.size() - start_num, 3);
	for (int i = 0; i<P.rows(); i++)
	{
		P(i, 0) = sampled_point_list[start_num + i].x;
		P(i, 1) = sampled_point_list[start_num + i].y;
		P(i, 2) = sampled_point_list[start_num + i].z;
	}
#if 0
	// the old codes
	edit_insert_new_handles(P, false);
	const Eigen::MatrixXd handlefile = Handles.cast<double>();
	build_up_handle_structure(handlefile, HandleGroup);
#else

	for (int i = 0; i < P.rows(); i++)
	{
		handleStructure.edit_add_point(P.row(i));
	}

	handleStructure.build();

	setup_handles_M_for_HS();
	SetHandleConnectivity();

	notify_deformers();
	init_picking();
	vertices_moved(false);
#endif

	return true;
}

bool HandlePluginBase::delete_handles(int index)
{

	handleStructure.edit_delete_handle(index);

	handleStructure.build();

	do_this_when_handle_changed();

	return true;
}

bool HandlePluginBase::append_handle_struct_from_file(const char * handle_sfile_name)
{
	Eigen::MatrixXd VV;
	Eigen::MatrixXi TT, FF, EE, PP, BE, CF;
	std::vector<std::vector<Eigen::VectorXi>> GG;

	readHANDLE(handle_sfile_name, VV, TT, FF, EE, PP, BE, CF, GG);

	//build_up_handle_structure(VV,HandleGroup);

	/****/

	handleStructure.append(VV, TT, FF, EE, PP, BE, CF, GG);

	handleStructure.build();

	// No need to set Handles=H, alrady set in update_handles_from_list();
	//ori_Handles = H;
	//temp_Handles = H;

	do_this_when_handle_changed();

	/****/

	return true;
}

void HandlePluginBase::save_selected_handles(const char* handle_file_name)
{
	int num = 0;
	for (int i = 0; i<select_handle_list.size(); i++)
	{
		num += all_handle_list[select_handle_list[i]].Pos().rows();
	}
	Eigen::MatrixXd P(num, 3);
	int r = 0;
	for (int i = 0; i<select_handle_list.size(); i++)
	{
		int dr = all_handle_list[select_handle_list[i]].Pos().rows();
		P.block(r, 0, dr, 3) = all_handle_list[select_handle_list[i]].Pos();
		r += dr;
	}
	igl::writeDMAT(handle_file_name, P);
}

/******************************************************************/
/******************  Handles Modifying         ********************/
/******************************************************************/



/******************************************************************/
/*******************  Handles Pose Loading    *********************/
/******************************************************************/
#include <file_helper.h>

bool HandlePluginBase::load_pose_trans_from_file(const char* handle_file_name)
{
	Eigen::MatrixXd Pose;

	igl::readDMAT(handle_file_name, Pose);
	printf("Pose Matrix:(%d,%d)\n", Pose.rows(), Pose.cols());
	if (false)
	{// old implementation to remove later
		handleStructure.set_pos_trans(Pose);
	}
	else
	{
		Eigen::VectorXi mask;
		handleStructure.get_active_mask(mask);
		handleStructure.set_trans(Pose, mask);
	}

	notify_deformers();

	active_vertices_moved();

	return true;
}

bool HandlePluginBase::load_pose_pos_from_file(const char* handle_file_name)
{
	bool only_active = true;
	return load_pose_pos_or_disp_from_file(handle_file_name, only_active, true);
}

bool HandlePluginBase::load_pose_disp_from_file(const char* handle_file_name)
{
	bool only_active = true;
	return load_pose_pos_or_disp_from_file(handle_file_name, only_active, false);
}

bool HandlePluginBase::load_pose_pos_or_disp_from_file(const char* handle_file_name, bool only_active, bool abs_pos)
{
	Eigen::MatrixXd handle_pos;

	std::string handle_file_name_string = std::string(handle_file_name);
	std::string extension;

	if (!extract_extension(handle_file_name_string, extension))
	{
		return false;
	}

	if ( extension == "dmat")
	{
		igl::readDMAT(handle_file_name, handle_pos);
		
	} 
	else if (extension == "obj")
	{
		Eigen::MatrixXi handle_face_no_use;
		igl::readOBJ(handle_file_name, handle_pos, handle_face_no_use);
	}
	else
	{
		printf("Error: Unknown handle pose format!\n");
		return false;
	}

	printf("Handle Position Matrix:(%d,%d)\n", handle_pos.rows(), handle_pos.cols());

	set_pose_pos_or_disp(handle_pos, only_active, abs_pos);

	return true;
}

bool HandlePluginBase::set_pose_pos_or_disp(Eigen::MatrixXd handle_pose, bool only_active, bool abs_pos)
{
	assert(only_active == false);// TODO implement the other case

	Eigen::VectorXi mask;
	handleStructure.get_active_mask(mask);

	handleStructure.set_pos(handle_pose, mask, abs_pos);

	//update_handles_from_list();

	//ori_Handles = Handles;
	// Important: not set ori_Handle here since rest pose do not change, just change current pose
	temp_Handles = Handles;// Not sure whether to change this or not

	notify_deformers();

	//SetHandleConnectivity(); // Important: should not set this!

	active_vertices_moved();

	return true;
}

bool HandlePluginBase::load_pose_rot_from_file(const char* fname)
{
	Eigen::MatrixXd PoseQuatMatrix;

	if (!igl::readDMAT(fname, PoseQuatMatrix))
	{
		printf("Fail to load Pose Rotations (Quaternions) Matrix!\n");
		return false;
	}

	std::vector<int> indics_rot;

	// Rotate the handles on the top list.
	for (int i = 0; i < PoseQuatMatrix.rows(); i++)
	{
		indics_rot.push_back(i);
	}

	if (!rotate_some_handles(PoseQuatMatrix, indics_rot))
	{
		return false;
	}

	return true;
}

bool HandlePluginBase::save_pose_trans_to_file(const char* handle_file_name)
{

	if (false)
	{
		// old implementation to remove later
		int n = Handles.rows();
		assert(Trans.rows() == 4 * n);
		igl::writeDMAT(handle_file_name, Trans);
	}
	else
	{
		Eigen::MatrixXd TPose;

		Eigen::VectorXi mask;
		handleStructure.get_active_mask(mask);

		handleStructure.get_trans(TPose, mask);

		igl::writeDMAT(handle_file_name, TPose);
	}

	return true;
}

bool HandlePluginBase::save_pose_pos_to_file(const char* handle_file_name)
{
	bool only_active = true;
	return save_pose_pos_or_disp_to_file(handle_file_name, only_active, true);
}

bool HandlePluginBase::save_pose_disp_to_file(const char* handle_file_name)
{
	bool only_active = true;
	return save_pose_pos_or_disp_to_file(handle_file_name, only_active, false);
}

bool HandlePluginBase::save_pose_pos_or_disp_to_file(const char* handle_file_name, bool only_active/* = false*/, bool abs_pos/* = true*/)
{
	Eigen::MatrixXd handle_pos;

	Eigen::VectorXi mask;
	handleStructure.get_active_mask(mask);

	handleStructure.get_pos(handle_pos, mask, abs_pos);

	igl::writeDMAT(handle_file_name, handle_pos);

	return true;
}

bool HandlePluginBase::save_pose_rot_to_file(const char* fname)
{
	printf("Error: save_pose_rot_to_file() not implemented yet!\n");
	return false;
}

bool HandlePluginBase::reset_handles()
{
	Handles = ori_Handles;

	clear_selected_handles();
	clear_all_handles();
	selection_changed();
	for (int i = 0; i<Handles.rows(); i++)
	{
		add_all_handles(
			(double)Handles(i, 0),
			(double)Handles(i, 1),
			(double)Handles(i, 2));
	}

	notify_deformers();

	return true;
}

bool HandlePluginBase::reset_rotation()
{
	h_Rotation[0] = 0;
	h_Rotation[1] = 0;
	h_Rotation[2] = 0;
	h_Rotation[3] = 1;
	h_Rotation_2D = 0.0;
	return true;
}

void HandlePluginBase::commit_temp_handles_only()
{
	temp_Handles = Handles;
}

void HandlePluginBase::update_handles_for_keyframing()
{// write handles to user moving buffer handles

	assert(Handles.cols() == 3);
	assert(Handles.rows() == all_handle_list.size());

	for (int i = 0; i<Handles.rows(); i++)
	{
		all_handle_list[i].updateFromPoint3D(Handles(i, 0), Handles(i, 1), Handles(i, 2));
	}
	ori_Handles = Handles;
	temp_Handles = Handles;

	update_handles_from_list();

	notify_deformers();

	return;
}

// Plan to remove the following 4 functions

void HandlePluginBase::clear_selected_handles()
{
	handleStructure.clear_selected_list();
}

void HandlePluginBase::add_selected_handles(int index)
{
	handleStructure.add_selected(index);
}

void HandlePluginBase::clear_all_handles()
{
	handleStructure.clear();
}

void HandlePluginBase::add_all_handles(double x, double y, double z, int index)
{
	//all_handle_list.push_back(handleData::Point(x,y,z,index));
}


/******************************************************************/
/*******************         Display          *********************/
/******************************************************************/
#include <modified/draw_skeleton_vector_graphics.h>
#include <igl/material_colors.h>
void HandlePluginBase::drawHandles()
{
	float diameter = std::min(m_preview->GetMainMesh().diameter*0.004, m_preview->GetMainMesh().avg_edge_length*0.3);

	if (bDrawHandlePoints)
	{
		std::vector<bool> bToDraw(all_handle_list.size());
		for (int i = 0; i < bToDraw.size(); i++)
			bToDraw[i] = (i < max_handle_index_to_draw) && (i >= min_handle_index_to_draw);
		draw_point_handles( handle_radius, handle_color, diameter, ratio_of_passive_point_handle_radius, bDrawRegionHandles, !bDrawRegionHandleCenters, handleStructure, bToDraw);
	}
	if (bDrawWireframe)
	{
		draw_wire_frame(1.5 * handle_radius / DEFAULT_HANDLE_RADIUS);
		for (int i = 0; i < all_handle_list.size(); i++)
		{
			if (i < min_handle_index_to_draw || i >= max_handle_index_to_draw)
				continue;

			if (all_handle_list[i].type() == HandleTypeBoneEdge)
			{
				Eigen::MatrixXd C(2, 3);

				if (false)
				{
					const Eigen::MatrixXd& P = all_handle_list[i].Pos();
					C <<
						P(0, 0), P(0, 1), P(0, 2),
						P(1, 0), P(1, 1), P(1, 2);
				}
				else
				{
					int bs = (*all_handle_list[i].BE)(0, 0);
					int be = (*all_handle_list[i].BE)(0, 1);
					if (bs >= 0 && be >= 0)
					{
						C <<
							Handles(bs, 0), Handles(bs, 1), Handles(bs, 2),
							Handles(be, 0), Handles(be, 1), Handles(be, 2);
					}
					else
					{
						// tmp code, BE is not initialized yet.

						const Eigen::MatrixXd& P = all_handle_list[i].Pos();
						C <<
							P(0, 0), P(0, 1), P(0, 2),
							P(1, 0), P(1, 1), P(1, 2);
					}

				}

				// push depth setting
				GLboolean old_depth_test;
				glGetBooleanv(GL_DEPTH_TEST, &old_depth_test);
				glDisable(GL_DEPTH_TEST);

				if (false)
				{

					// a simple line segment
					draw_directed_line_segment2(
						C(0, 0), C(0, 1), C(0, 2),
						C(1, 0), C(1, 1), C(1, 2),
						5 * diameter, 1, 1, 1, false);
				}
				else
				{
					Eigen::MatrixXi BE(1, 2);
					BE << 0, 1;
					if (C(0, 0) != C(1, 0) || C(0, 1) != C(1, 1) || C(0, 2) != C(1, 2))// a hack to prevent the elephant example, remove later.
						igl::draw_skeleton_vector_graphics(C, BE, igl::BBW_POINT_COLOR, igl::BBW_LINE_COLOR, handle_radius / DEFAULT_HANDLE_RADIUS);
				}

				// pop depth setting
				if (old_depth_test)	glEnable(GL_DEPTH_TEST);
			}
		}
	}

	//	glPushMatrix();
	//
	//	float mat[4*4];
	//	Preview3D::ConvertQuaternionToMatrix(m_preview->g_Rotation, mat);
	//	glMultMatrixf(mat);
	//	glScaled(m_preview->g_Zoom, m_preview->g_Zoom, m_preview->g_Zoom);
	//	glScaled(m_preview->zoom, m_preview->zoom, m_preview->zoom);
	//	glTranslatef(m_preview->g_Translation[0],m_preview->g_Translation[1],m_preview->g_Translation[2]);
	//
	//	// Render flipped tets
	//	glDisable(GL_LIGHTING);
	//	if(!enable_depth)
	//	{
	//		glDisable(GL_DEPTH_TEST);
	//	}
	//	glDisable(GL_CULL_FACE);
	//	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	//
	//	glColor3f(0,1,0);
	//
	//	//mainly draw the handles here:
	//	//assert(Handles.rows()==all_handle_list.size());
	//	for(int i=0; i<all_handle_list.size(); i++)
	//	{
	//		if(handleDispType==WithDepth)
	//		{
	//			if(true)
	//			{
	//				//adaptive handle size
	//#define Adaptive_Handle_size
	//#ifdef Adaptive_Handle_size
	//				double diameter = std::min(m_preview->diameter*0.004,m_preview->avg_edge_length*0.3);
	//#endif
	//				float color[4];
	//				color[0] = handle_color[0];
	//				color[1] = handle_color[1];
	//				color[2] = handle_color[2];
	//				color[3] = handle_color[3];
	//
	//				if(all_handle_list[i].active)
	//				{
	//					color[0] = 255;
	//					color[1] = 0;
	//					color[2] = 0;
	//					color[3] = 0;
	//				}
	//				if(all_handle_list[i].selected&&all_handle_list[i].active)
	//				{
	//					color[0] = 0;
	//					color[1] = 0;
	//					color[2] = 255;
	//					color[3] = 0;
	//				}
	//
	//				const Eigen::MatrixXd& P = all_handle_list[i].Pos();
	//				if (P.rows()==1)
	//				{
	//					quadricDrawPoint(
	//						P(0,0),
	//						P(0,1),
	//						P(0,2), 
	//						diameter*handle_radius,
	//						color);
	//				}
	//				else
	//				{
	//					for (int k=0; k<P.rows(); k++)
	//					{
	//						quadricDrawPoint(
	//							P(k,0),
	//							P(k,1),
	//							P(k,2), 
	//							0.3*diameter*handle_radius,
	//							color);
	//					}
	//				}
	//
	//				const Eigen::MatrixXd& PC = all_handle_list[i].CenterPos();
	//				if(all_handle_list[i].active)
	//				{
	//
	//					draw_point(
	//						PC(0,0),
	//						PC(0,1),
	//						PC(0,2), 
	//						handle_radius,
	//						true);
	//				}
	//				
	//				//glPushMatrix();
	//				//glPushMatrix();
	//				//GLUquadricObj *quadric;
	//				//quadric = gluNewQuadric();
	//				//gluQuadricDrawStyle(quadric, GLU_FILL );
	//
	//				//glTranslatef(Handles(i,0),Handles(i,1),Handles(i,2));
	//				//glColor3d(handle_color[0],handle_color[1],handle_color[2]);
	//				//gluSphere(quadric,handle_radius,7,3);
	//
	//				//gluDeleteQuadric(quadric);
	//				//glPopMatrix();
	//				//glPopMatrix();
	//			}
	//			else
	//			{// old // To remove
	//
	//				const Eigen::MatrixXd& P = all_handle_list[i].Pos();
	//				for (int k=0; k<P.rows(); k++)
	//				{
	//					draw_point_with_depth(
	//						P(k,0),
	//						P(k,1),
	//						P(k,2), 
	//						handle_radius,
	//						all_handle_list[i].selected,
	//						handle_color[0],handle_color[1],handle_color[2]);
	//				}
	//			}
	//		}
	//		else
	//		{
	//
	//			const Eigen::MatrixXd& P = all_handle_list[i].CenterPos();
	//			if(all_handle_list[i].selected&&all_handle_list[i].active)
	//			{
	//				draw_point(
	//					P(0,0),
	//					P(0,1),
	//					P(0,2), 
	//					handle_radius,
	//					all_handle_list[i].selected);
	//			}
	//		}
	//		//draw_point(
	//		//	(double) PickingPlugin::GetReference().all_handle_list,
	//		//	);
	//	}
	//
	//	//for(int i=0; i<HandleFaces.rows(); i++)
	//	//{
	//	//	int v1 = HandleFaces(i,0);
	//	//	int v2 = HandleFaces(i,1);
	//	//	int	v3 = HandleFaces(i,2);
	//
	//	//	draw_directed_line_segment2(Handles(v1,0),Handles(v1,1),Handles(v1,2),
	//	//								Handles(v2,0),Handles(v2,1),Handles(v2,2),3,0,0,0);
	//	//	draw_directed_line_segment2(Handles(v2,0),Handles(v2,1),Handles(v2,2),
	//	//								Handles(v3,0),Handles(v3,1),Handles(v3,2),3,0,0,0);
	//	//	draw_directed_line_segment2(Handles(v3,0),Handles(v3,1),Handles(v3,2),
	//	//								Handles(v1,0),Handles(v1,1),Handles(v1,2),3,0,0,0);
	//	//}
	//
	//
	//	glPopMatrix();
	//
	//	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	//	if(!enable_depth)
	//	{
	//		glEnable(GL_DEPTH_TEST);
	//	}
	//	glEnable(GL_CULL_FACE);
}

void HandlePluginBase::draw_wire_frame(float linewidth)
{
	for (int i = 0; i<HandleFaces.rows(); i++)
	{
		int v1 = HandleFaces(i, 0);
		int v2 = HandleFaces(i, 1);
		int	v3 = HandleFaces(i, 2);

		draw_directed_line_segment2(Handles(v1, 0), Handles(v1, 1), Handles(v1, 2),
			Handles(v2, 0), Handles(v2, 1), Handles(v2, 2), linewidth, 0, 0, 0, true);
		draw_directed_line_segment2(Handles(v2, 0), Handles(v2, 1), Handles(v2, 2),
			Handles(v3, 0), Handles(v3, 1), Handles(v3, 2), linewidth, 0, 0, 0, true);
		draw_directed_line_segment2(Handles(v3, 0), Handles(v3, 1), Handles(v3, 2),
			Handles(v1, 0), Handles(v1, 1), Handles(v1, 2), linewidth, 0, 0, 0, true);
	}
}

void HandlePluginBase::drawRayTracedPoint()
{
	glDisable(GL_LIGHTING);
	glDisable(GL_CULL_FACE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	glColor3f(1, 0, 0);

	//mainly draw here

	if (handleDispType == WithDepth)
	{
		draw_point_with_depth(
			ray_traced_point.x,//(double) PickingPlugin::GetReference().ray_traced_point.x,
			ray_traced_point.y,//(double) PickingPlugin::GetReference().ray_traced_point.y,
			ray_traced_point.z,//(double) PickingPlugin::GetReference().ray_traced_point.z,
			7,
			false, 1, 0, 0);
	}
	else
	{
		draw_point(
			ray_traced_point.x,//(double) PickingPlugin::GetReference().ray_traced_point.x,
			ray_traced_point.y,//(double) PickingPlugin::GetReference().ray_traced_point.y,
			ray_traced_point.z,//(double) PickingPlugin::GetReference().ray_traced_point.z,
			7,
			false);
	}



	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glEnable(GL_CULL_FACE);
}

inline void HandlePluginBase::drawIndicesInMesh()
{
	for (int i = 0; i<Handles.rows(); i++)
	{
		glColor3f(0, 0, 0);
		draw_text(
			Handles(i, 0),
			Handles(i, 1),
			Handles(i, 2),
			std::to_string((long long)HandleIndices(i)),//HandleIndices(i)
			false);
		// to_sting bug fix: http://stackoverflow.com/questions/14617950/ambiguous-call-to-overloaded-function-stdto-string
	}
}

inline void HandlePluginBase::drawIndicesInHandleStruct()
{
	for (int i = 0; i<all_handle_list.size(); i++)
	{
		const Eigen::MatrixXd& Pi = all_handle_list[i].CenterPos();
		glColor3f(0, 0, 0);
		draw_text(
			Pi(0, 0),
			Pi(0, 1),
			Pi(0, 2),
			std::to_string((long long)(i)),//HandleIndices(i)
			false);
		// to_sting bug fix: http://stackoverflow.com/questions/14617950/ambiguous-call-to-overloaded-function-stdto-string
	}
}

#include <draw_primitives.h>
inline void HandlePluginBase::drawFrames(int d)
{

	double diameter = std::min(m_preview->GetMainMesh().diameter*0.004, m_preview->GetMainMesh().avg_edge_length*0.3);

	for (int i = 0; i<all_handle_list.size(); i++)
	{
		if (i<min_handle_index_to_draw || i >= max_handle_index_to_draw)
		{
			continue;
		}

		if (all_handle_list[i].type() == HandleTypePoint)
		{
			continue;
		}

		const Eigen::MatrixXd& P = all_handle_list[i].CenterPos();
		const Eigen::MatrixXd& T3d = all_handle_list[i].Trans(3);

		double length = 15.;

		Vector3 from(
			P(0, 0),
			P(0, 1),
			P(0, 2));

		Eigen::MatrixXd Color(3, 3);
		Color <<
			1., 0., 0.,
			0., 1., 0.,
			0., 0., 1.;

		for (int dim = 0; dim < 3; dim++)
		{
			glColor3f(Color(dim, 0), Color(dim, 1), Color(dim, 2));
			Vector3 to(
				T3d(dim, 0),
				T3d(dim, 1),
				T3d(dim, 2));
			to = from + to*handle_radius*diameter*length;
			if (d==2)
			{

				if (dim<2)
				{
					// only draw for x and y dim
					draw_arrow_2d(from, to, Color.row(dim).transpose(), 0.4*handle_radius*diameter);
				}

				//paintArrow(from, to, 20);
				//paintArrow(from, to, 3*handle_radius);
			}
			else
			{
				/*
				glLineWidth(3);
				glBegin(GL_LINES);
				glVertex(from);
				glVertex(to);
				glEnd();
				*/
				draw_arrow_3d(from, to, Color.row(dim).transpose(), 0.4*handle_radius*diameter);
			}

		}
	}

}

void HandlePluginBase::drawPointsSet()
{
	// Push Depth
	GLboolean old_depth_test;
	glGetBooleanv(GL_DEPTH_TEST, &old_depth_test);
	glDisable(GL_DEPTH_TEST);

	float diameter = std::min(m_preview->GetMainMesh().diameter*0.004, m_preview->GetMainMesh().avg_edge_length*0.3);

	Eigen::MatrixXd PS = pointsSet.toMat();
	for (int i = 0; i < PS.rows(); i++)
	{
		draw_point(
			PS(i, 0),
			PS(i, 1),
			PS(i, 2),
			handle_radius,
			true);
	}

	if (try_skeleton_volume || try_skeleton)
	{
		if (PS.rows() >= 2)
		{
			
			// Only used for with_symmetric case
			Eigen::MatrixXd PS2 = plane_symmetric(PS);
			
			if (try_skeleton_volume)
			{
				draw_directed_line_segment2(
					PS(0, 0), PS(0, 1), PS(0, 2),
					PS(1, 0), PS(1, 1), PS(1, 2),
					ske_radius*diameter, 1, 1, 1, false);
				Eigen::VectorXd s(3);
				s(0) = PS(0, 0);
				s(1) = PS(0, 1);
				s(2) = PS(0, 2);
				Eigen::VectorXd e(3);
				e(0) = PS(1, 0);
				e(1) = PS(1, 1);
				e(2) = PS(1, 2);
				Eigen::MatrixXd ske;
				volumetric_skeleton_sampling(
					s, e, ske_dim,
					num_seg_try_skeleton,
					num_per_seg_try_skeleton,
					ske_ring,
					ske_radius*diameter,
					ske,
					true,
					ske_cylinder_or_ellipsoid);
				for (int i = 0; i < ske.rows(); i++)
				{
					draw_point(
						ske(i, 0),
						ske(i, 1),
						ske(i, 2),
						handle_radius,
						false,
						1, 1, 0);
				}

				if (with_symmetric)
				{
					Eigen::MatrixXd ske2 = plane_symmetric(ske);
					draw_directed_line_segment2(
						PS2(0, 0), PS2(0, 1), PS2(0, 2),
						PS2(1, 0), PS2(1, 1), PS2(1, 2),
						ske_radius*diameter, 1, 1, 1, false);
					for (int i = 0; i < ske.rows(); i++)
					{
						draw_point(
							ske2(i, 0),
							ske2(i, 1),
							ske2(i, 2),
							handle_radius,
							false,
							1, 1, 0);
					}
				}
			}

			if (try_skeleton)
			{
				Eigen::MatrixXi BE(1, 2);
				BE << 0, 1;
				igl::draw_skeleton_vector_graphics(PS, BE, igl::BBW_POINT_COLOR, igl::BBW_LINE_COLOR, handle_radius / DEFAULT_HANDLE_RADIUS);
				if (with_symmetric)
				{
					igl::draw_skeleton_vector_graphics(PS2, BE, igl::BBW_POINT_COLOR, igl::BBW_LINE_COLOR, handle_radius / DEFAULT_HANDLE_RADIUS);
				}
			}
		}
	}

	// Pop Depth
	if (old_depth_test)
		glEnable(GL_DEPTH_TEST);
	else
		glDisable(GL_DEPTH_TEST);
}

/******************************************************************/
/*******************      Transformation         *********************/
/******************************************************************/

float* HandlePluginBase::GetRotation() const
{
	//if(!is3DRotation) 
	//	h_Rotation[0] = h_Rotation[1] = 0.;
	return h_Rotation;
}

void HandlePluginBase::SetRotation(const float* new_rotation)
{
	h_Rotation[0] = new_rotation[0];
	h_Rotation[1] = new_rotation[1];
	h_Rotation[2] = new_rotation[2];
	h_Rotation[3] = new_rotation[3];

	//if(!is3DRotation) 
	//	h_Rotation[0] = h_Rotation[1] = 0.;

	if (only_positional_rotate)
	{
		positional_rotate_selected_handles(h_Rotation);
	}
	else
	{
		rotate_selected_handles(h_Rotation, handle_scale);
	}
}

void HandlePluginBase::SetScale(const double s)
{
	handle_scale = s;
	if (only_positional_rotate)
	{
		positional_rotate_selected_handles(h_Rotation);
	}
	else
	{
		rotate_selected_handles(h_Rotation, handle_scale);
	}
}

double HandlePluginBase::GetScale() const
{
	return handle_scale;
}

const float* HandlePluginBase::GetRotation2D() const
{
	return &h_Rotation_2D;
}

void HandlePluginBase::SetRotation2D(const float* new_rotation)
{
	h_Rotation_2D = *new_rotation;

	float rotation_3d[4] = { 0. };

	if (is3DRotation)
	{
		;// do nothing
	}
	else
	{
		if (false)
		{
			rotation_3d[0] = 0;
			rotation_3d[1] = 0;
			rotation_3d[2] = sin(h_Rotation_2D / 2);
			rotation_3d[3] = cos(h_Rotation_2D / 2);
		}
		else
		{
			//double n = m_preview->g_Rotation[0] * m_preview->g_Rotation[0]
			//	+ m_preview->g_Rotation[1] * m_preview->g_Rotation[1]
			//	+ m_preview->g_Rotation[2] * m_preview->g_Rotation[2];
			//n = sqrt(n);
			//double v[3];

			//v[0] = m_preview->center[0] - m_preview->eye[0];
			//v[1] = m_preview->center[1] - m_preview->eye[1];
			//v[2] = m_preview->center[2] - m_preview->eye[2];

			//Eigen::Vector4d V_world(v[0], v[1], v[2], 1);
			//Eigen::Matrix4d ModelViewMatrix(m_preview->m_modelview_matrix);
			//ModelViewMatrix(0, 3) = 0;
			//ModelViewMatrix(1, 3) = 0;
			//ModelViewMatrix(2, 3) = 0;
			//ModelViewMatrix(3, 3) = 1;
			//Eigen::Vector4d V_object = ModelViewMatrix.inverse()*V_world;
			//v[0] = V_object[0];
			//v[1] = V_object[1];
			//v[2] = V_object[2];
			//double normalizer = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
			//normalizer = sqrt(normalizer);
			//v[0] = v[0] / normalizer;
			//v[1] = v[1] / normalizer;
			//v[2] = v[2] / normalizer;
			//rotation_3d[0] = v[0] * sin(h_Rotation_2D / 2);
			//rotation_3d[1] = v[1] * sin(h_Rotation_2D / 2);
			//rotation_3d[2] = v[2] * sin(h_Rotation_2D / 2);
			//rotation_3d[3] = cos(h_Rotation_2D / 2);

		}
	}

	SetRotation(rotation_3d);

}


void HandlePluginBase::translate_selected_handles(float * translation, bool write_constraint)
{
	for (int i = 0; i<select_handle_list.size(); i++)
	{
		if (!all_handle_list[select_handle_list[i]].active)
		{
			continue;//skip passive handles
		}
		all_handle_list[select_handle_list[i]].translate(translation);
	}

	if (write_constraint)
	{
		// Write Constraints Immediately, not quite necessary, might slow down the program. But keep the code simple.
		selected_vertices_moved();
	}
}

void HandlePluginBase::rotate_selected_handles(const float * rotation, const float scale)
{
	Eigen::MatrixXd Rot;
	ConvertQuaternionToMatrix3x3(rotation, Rot);

	Rot = Rot * scale;

	for (int i = 0; i<select_handle_list.size(); i++)
	{
		if (!all_handle_list[select_handle_list[i]].active)
		{
			continue;// skip passive handles
		}
		if (all_handle_list[select_handle_list[i]].type() == HandleTypePoint)
		{
			continue;// skip pure point type
		}

		switch (rotate_type)
		{
		case RotateAroundAvg:
		{
			all_handle_list[select_handle_list[i]].setRotationAroundCenter(Rot);
		}
		break;
		case RotateAroundFirstPoint:
		{
			all_handle_list[select_handle_list[i]].setRotationAroundFristP(Rot);
		}
		break;
		case RotateAroundCursor:
		{
			Eigen::MatrixXd C(1, 3);
			C << ray_traced_point.x, ray_traced_point.y, ray_traced_point.z;
			all_handle_list[select_handle_list[i]].setRotation(Rot, C);
		}
		break;
		}
	}

	update_handles_from_list();// TODO: should replace this with some lower cost function

	// Write Constraints Immediately, not quite necessary, might slow down the program. But keep the code simple.
	selected_vertices_moved();
}

bool HandlePluginBase::rotate_some_handles(const Eigen::MatrixXd& QuatMat, const std::vector<int> IndexMat)
{

	if (QuatMat.rows() != IndexMat.size() || QuatMat.cols() != 4)
	{
		printf("Error: the Quaternion Matrix (%d,%d) does not have the right dimension!\n", QuatMat.rows(), QuatMat.cols());
		return false;
	}

	for (int i = 0; i < IndexMat.size(); i++)
	{
		if (false)
		{
			if (!all_handle_list[IndexMat[i]].active)
			{
				continue;// skip passive handles
			}
		}


		if (all_handle_list[IndexMat[i]].type() == HandleTypePoint)
		{
			continue;// skip pure point type
		}

		float rotation[4];
		rotation[0] = QuatMat(i, 0);
		rotation[1] = QuatMat(i, 1);
		rotation[2] = QuatMat(i, 2);
		rotation[3] = QuatMat(i, 3);
		Eigen::MatrixXd Rot;
		ConvertQuaternionToMatrix3x3(rotation, Rot);

		// Do not set scale here: Rot = Rot * handle_scale;

		switch (rotate_type)
		{
		case RotateAroundAvg:
		{
			all_handle_list[IndexMat[i]].setRotationAroundCenter(Rot);
		}
		break;
		case RotateAroundFirstPoint:
		{
			all_handle_list[IndexMat[i]].setRotationAroundFristP(Rot);
		}
		break;
		case RotateAroundCursor:
		{
			Eigen::MatrixXd C(1, 3);
			C << ray_traced_point.x, ray_traced_point.y, ray_traced_point.z;
			all_handle_list[IndexMat[i]].setRotation(Rot, C);
		}
		break;
		}
	}

	update_handles_from_list();// TODO: should replace this with some lower cost function

	// Write Constraints Immediately, not quite necessary, might slow down the program. But keep the code simple.
	selected_vertices_moved();
}

void HandlePluginBase::set_selected_handles_trans(const std::vector<Eigen::MatrixXd>& trans)
{
	if (trans.size() != select_handle_list.size())
	{
		// This needs to be true because External functions should have set all selected handles to be type with transfromation.
		printf("Error: the transformation list size does not equals to that of selected handle list.\n");
		return;
	}


	for (int i = 0; i<select_handle_list.size(); i++)
	{
		if (!all_handle_list[select_handle_list[i]].active)
		{
			printf("Error: this might happen, but we should make it never happen, doing sth.\n");
			continue;// skip passive handles
		}
		if (all_handle_list[select_handle_list[i]].type() == HandleTypePoint)
		{
			Eigen::MatrixXd Ti = Eigen::MatrixXd::Identity(4, 4);
			Ti.block(0, 0, 3, 4) = trans[i].block(0, 0, 3, 4);
			Eigen::MatrixXd P = Eigen::MatrixXd::Ones(1, 4);
			P.block(0, 0, 1, 3) = all_handle_list[select_handle_list[i]].CenterRestPos();
			P = P * Ti.transpose();
			all_handle_list[select_handle_list[i]].setPointPos(P);
		}
		else
		{
			all_handle_list[select_handle_list[i]].updateTrans(trans[i].transpose());
		}
	}

	update_handles_from_list();// TODO: should replace this with some lower cost function

	// Write Constraints Immediately, not quite necessary, might slow down the program. But keep the code simple.
	selected_vertices_moved();
}

void HandlePluginBase::positional_rotate_selected_handles(const float * rotation)
{
	Eigen::MatrixXd Rot;
	ConvertQuaternionToMatrix3x3(rotation, Rot);

	Rot = Rot * handle_scale;

	Eigen::MatrixXd center = Eigen::MatrixXd::Zero(1, 3);
	int num_to_rotate = 0;

	for (int i = 0; i<select_handle_list.size(); i++)
	{
		if (!all_handle_list[select_handle_list[i]].active)
		{
			continue;// skip passive handles
		}
		center += all_handle_list[select_handle_list[i]].CenterPos();
		num_to_rotate++;
	}
	if (num_to_rotate == 0)
		return;
	center *= 1. / num_to_rotate;
	for (int i = 0; i<select_handle_list.size(); i++)
	{
		if (!all_handle_list[select_handle_list[i]].active)
		{
			continue;// skip passive handles
		}
		// Rotate around
		// P is the transpose of coordinate
		Eigen::MatrixXd P = all_handle_list[select_handle_list[i]].CenterPos();
		Eigen::MatrixXd new_P = (P - center)*Rot.transpose() + center;
		all_handle_list[select_handle_list[i]].translate(new_P - P);
	}

	// Write Constraints Immediately, not quite necessary, might slow down the program. But keep the code simple.
	selected_vertices_moved();
}

/******************************************************************/
/*******************      on-fly edit           *******************/
/******************************************************************/

void HandlePluginBase::add_purepoint_handles_on_fly(const Eigen::MatrixXd& PH)
{
	clear_points_set();
	pointsSet.fromMat(PH);

	//BeginChangeHandlesOnFly();

	int start_index = handleStructure.all_handle_list.size();
	add_some_purepoint_handle_from_PS();
	int end_index = handleStructure.all_handle_list.size();
	move_handle_onto_mesh();
	for (int i = start_index; i < end_index; i++)
		handleStructure.set_one_active(i, true); // set the last one to be active


	//EndChangeHandlesOnFly();

	constraint_changed();
	clear_points_set();
}

#include <move_point_onto_mesh.h>
void HandlePluginBase::add_region_handle_on_fly(const Eigen::MatrixXd& PH, bool add_on_deformed_shape)// right now supports only add one region handle on fly
{

	bool without_poping = add_on_deformed_shape;

	Eigen::MatrixXd P;
	Eigen::VectorXi I;
#if 1

	{
		std::vector<Point3D> sampled_point_list;

		I.resize(PH.rows());
		P.resize(PH.rows(), 3);

		for (int j = 0; j < PH.rows(); j++)
		{
			sampled_point_list.push_back(Point3D(PH(j, 0), PH(j, 1), PH(j, 2)));
		}
		const Eigen::MatrixXd * vertices_to_search = m_preview->GetMainMesh().vertices;
		const Eigen::MatrixXd * rest_vertices = &m_preview->GetMainMesh().rest_vertices;
		attach_all_handles_to_nearest_vertices(vertices_to_search, sampled_point_list);

		for (int j = 0; j < P.rows(); j++)
		{
			I(j) = sampled_point_list[j].index;

			//P(j, 0) = sampled_point_list[j].x;
			//P(j, 1) = sampled_point_list[j].y;
			//P(j, 2) = sampled_point_list[j].z;

			P(j, 0) = (*rest_vertices)(I[j], 0);
			P(j, 1) = (*rest_vertices)(I[j], 1);
			P(j, 2) = (*rest_vertices)(I[j], 2);
		}
	}

#else
	P = PH;
#endif

	clear_points_set();
	pointsSet.fromMat(P);

	//BeginChangeHandlesOnFly();

	int start_index = handleStructure.all_handle_list.size();
	add_region_handle_from_pointset();

	int end_index = handleStructure.all_handle_list.size();

#if 1
	all_handle_list.back().setIndices(I);
	update_handles_from_list();
#else
	move_handle_onto_mesh(true);// without_poping);
#endif

	for (int i = start_index; i < end_index; i++)
	{
		//if (!without_poping)
		//	all_handle_list[i].resumeToRest();
		handleStructure.set_one_active(i, true); // set the last one to be active
	}



	//EndChangeHandlesOnFly();

	constraint_changed();
	clear_points_set();
}

void HandlePluginBase::slide_existing_handles_on_fly(const Eigen::MatrixXd& PH, const Eigen::VectorXi& IH)
{
	assert(PH.rows() == IH.rows() && PH.cols() == 3);

	//BeginChangeHandlesOnFly();

	for (int i = 0; i < IH.rows(); i++)
	{
		if (handleStructure.all_handle_list[IH(i)].type() == HandleTypePoint)
		{
			//only slide pure point handle
			handleStructure.all_handle_list[IH(i)].setPointPos(PH.row(i));
		}
	}

	move_handle_onto_mesh();
	for (int i = 0; i < IH.rows(); i++)
		handleStructure.set_one_active(IH(i), true); // set the last one to be active
	constraint_changed();

	//EndChangeHandlesOnFly();
}

/******************************************************************/
/*******************         cursor           *********************/
/******************************************************************/

bool HandlePluginBase::cursor_update(double x, double y, double z, int index, char key)
{
	if (key == 'r')
	{
		//cursorUsageType = AsRotationCenter;
		bDrawRayTracedCursor = true;

		cursor_rotation_center[0] = ray_traced_point.x;//PickingPlugin::GetReference().ray_traced_point.x;
		cursor_rotation_center[1] = ray_traced_point.y;//PickingPlugin::GetReference().ray_traced_point.y;
		cursor_rotation_center[2] = ray_traced_point.z;//PickingPlugin::GetReference().ray_traced_point.z;
	}
	else
	{
		switch (cursorUsageType)
		{
			//case 'r'://
		case AsInsertedHandle:
		{
			Eigen::MatrixXd PH(1, 3);
			PH << ray_traced_point.x, ray_traced_point.y, ray_traced_point.z;
			BeginChangeHandlesOnFly();
			add_purepoint_handles_on_fly(PH);
			EndChangeHandlesOnFly(true);
		}
		break;
		//case 'l'://
		case AsInsertedPoint:
		{
			pointsSet.add(ray_traced_point.x, ray_traced_point.y, ray_traced_point.z);
		}
		//case 'm'://
		case AsRotationCenter:
		{
			cursor_rotation_center[0] = ray_traced_point.x;//PickingPlugin::GetReference().ray_traced_point.x;
			cursor_rotation_center[1] = ray_traced_point.y;//PickingPlugin::GetReference().ray_traced_point.y;
			cursor_rotation_center[2] = ray_traced_point.z;//PickingPlugin::GetReference().ray_traced_point.z;
		}
		break;
		default:
			;
		}
	}

	return true;
}

bool HandlePluginBase::rayTraceMesh(const int mouse_x, const int mouse_y)
{//, double& x, double& y, double& z, int& hit_index

	float diameter = std::max(m_preview->GetMainMesh().diameter*0.004, m_preview->GetMainMesh().avg_edge_length * 2);

	int hit_index = -1;
	std::vector< std::pair<double, IndexType> > H;
	int minZ_index = -1;
	int maxZ_index = -1;

	bool hit = ray_trace_mesh(
		minZ_index,
		maxZ_index,
		mouse_x,
		mouse_y,
		m_preview->camera.m_viewport,
		m_preview->camera.m_projection_matrix,
		m_preview->camera.m_modelview_matrix,
		import_vertices,
		import_faces,
		H,
		m_preview->height);
	if (!hit)
		return false;
	//choice 1: average center
	//x = ((*import_vertices)(minZ_index,0)+(*import_vertices)(maxZ_index,0))/2;
	//y = ((*import_vertices)(minZ_index,1)+(*import_vertices)(maxZ_index,1))/2;
	//z = ((*import_vertices)(minZ_index,2)+(*import_vertices)(maxZ_index,2))/2;
	//choice 2: nearest one
	hit_index = minZ_index;
	ray_traced_point.x = (*import_vertices)(hit_index, 0);
	ray_traced_point.y = (*import_vertices)(hit_index, 1);
	ray_traced_point.z = (*import_vertices)(hit_index, 2);


	cursor_update(ray_traced_point.x, ray_traced_point.y, ray_traced_point.x, hit_index, 'l');

	return true;
}

void HandlePluginBase::cursorDown(int mouse_x, int mouse_y)
{
	if (!enable_cursor)
		return;

	rayTraceMesh(mouse_x, mouse_y);
	selection_changed();
}

void HandlePluginBase::set_cursor_from_PS()
{
	Eigen::MatrixXd PS = pointsSet.toMat();
	if (PS.rows()>0 && PS.cols()>=3)
	{
		ray_traced_point.x = PS(0, 0);
		ray_traced_point.y = PS(0, 1);
		ray_traced_point.z = PS(0, 2);
	}
}

/******************************************************************/
/*******************         Points Set        ********************/
/******************************************************************/

void HandlePluginBase::add_cursor_to_points_set()
{
	pointsSet.add(
		ray_traced_point.x,
		ray_traced_point.y,
		ray_traced_point.z);
}

void HandlePluginBase::clear_points_set()
{
	current_selected_index = -1;
	pointsSet.clear();
}

bool HandlePluginBase::load_points_set_from_file(const char *fname)
{
	Eigen::MatrixXd PS;
	if (!igl::readDMAT(fname, PS))
	{
		printf("Errors: Fails to load point set %s.\n",fname);
		return false;
	}
		
	clear_points_set();

	for (int i = 0; i < PS.rows(); i++)
	{
		pointsSet.add(PS(i, 0), PS(i, 1), PS(i, 2));
	}

	return true;
}

bool HandlePluginBase::save_points_set_to_file(const char* fname)
{
	Eigen::MatrixXd PS = pointsSet.toMat();
	return igl::writeDMAT(fname, PS);
}

/******************************************************************/
/*******************      Key Framing         *********************/
/******************************************************************/

bool HandlePluginBase::load_keyframing_config(const char * config_file_name)
{
	FILE * config_file = fopen(config_file_name, "r");
	if (NULL == config_file)
	{
		fprintf(stderr, "IOError: %s could not be opened...\n", config_file_name);
		return false;
	}
	handleKeyFrame.clear();

#ifndef LINE_MAX
#  define LINE_MAX 2048
#endif

	char line[LINE_MAX];
	int line_no = 1;
	while (fgets(line, LINE_MAX, config_file) != NULL)
	{
		char name[LINE_MAX];
		double duration;
		// Read first word containing type
		if (sscanf(line, "%s %lf\n", name, &duration) == 2)
		{
			Eigen::MatrixXd keyframe_Handles;
			if (!igl::readDMAT(name, keyframe_Handles))
			{
				fprintf(stderr,
					"Error: load_keyframing_config() cannot open %s\n",
					name);
				fclose(config_file);
				return false;
			};
			printf("KeyFrame Handle Matrix:(%d,%d)\n", keyframe_Handles.rows(), keyframe_Handles.cols());
			assert(keyframe_Handles.cols() == 3);
			handleKeyFrame.push_back(KeyFrame<HandleFrame>(HandleFrame(name, keyframe_Handles), duration, LINEAR_TRANSITION));
		}
		else
		{
			fprintf(stderr,
				"Error: load_keyframing_config() on line %d should have name and duration\n",
				line_no);
			fclose(config_file);
			return false;
		}
		line_no++;
	}
	fclose(config_file);
	key_frame_timer.Tic();
	if (!turn_on_handle_key_frame)
		key_frame_timer.Pause();
	return true;
}
void HandlePluginBase::update_keyframing_skinning()
{
	if (handleKeyFrame.size() == 0)
	{
		return;
	}
	double time = key_frame_timer.Time();
	size_t a = 0;
	size_t b = 0;
	double f = 0;
	handleKeyFrame.get_frame(time, a, b, f);
	//double ff=-2.0*f*f*f+3.0*f*f;
	double ff = f;
	//interpolate between two frames:
	Handles = (1 - ff)*handleKeyFrame[a].state.Handles + ff*handleKeyFrame[b].state.Handles;

	set_pose_pos_or_disp(Handles, false, true);
	//do_this_when_handle_moved();
}
void HandlePluginBase::set_turn_on_key_framing(bool t)
{
	turn_on_handle_key_frame = t;
	if (turn_on_handle_key_frame)
	{
		if (handleKeyFrame.size()>0)
			key_frame_timer.Resume();
	}
	else
	{
		if (handleKeyFrame.size()>0)
			key_frame_timer.Pause();
	}
}
bool HandlePluginBase::get_turn_on_key_framing() const
{
	return turn_on_handle_key_frame;
}

/******************************************************************/
/*******************      Constraints         *********************/
/******************************************************************/
void HandlePluginBase::set_handle_trans_constrained()
{
	for (int i = 0; i < all_handle_list.size(); i++)
	{
		all_handle_list[i].b_constrain_affine_trans = true;
	}
	vertices_moved(false);
}

void HandlePluginBase::set_handle_pos_constrained()
{
	for (int i = 0; i < all_handle_list.size(); i++)
	{
		all_handle_list[i].b_constrain_affine_trans = false;
	}
	vertices_moved(false);
}

bool HandlePluginBase::load_handle_constrained_type_from_file(const char* fname)
{
	Eigen::MatrixXi CTM;// constrained type matrix
	if (!igl::readDMAT(fname, CTM))
	{
		printf("Fail to load constrained type matrix!\n");
		return false;
	}

	if (CTM.rows() != all_handle_list.size() || CTM.cols() != 1)
	{
		printf("Incorrect constrained type matrix size (%d, %d)", CTM.rows(), CTM.cols());
		return false;
	}

	printf("Set constrained type matrix:\n");
	for (int i = 0; i < CTM.rows(); i++)
	{
		bool bcat = (CTM(i, 0) != 0);
		printf("%d: %d\n", i, bcat ? 1 : 0);
		all_handle_list[i].b_constrain_affine_trans = bcat;
	}
	return true;
}

/******************************************************************/
/************      Constraint Updating Output        **************/
/******************************************************************/

void HandlePluginBase::selection_changed()
{
	if (auto_marked_when_selected)
		mark_region_handle_in_mesh();

	// Nothing need to be done if only selected indices changed by now for constrained updating.
	return;

	//	update_handle_trans();
	//
	//	std::vector<IndexType> m_constrained_vertices;
	//	Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic>  m_constrained_vertex_positions;
	//
	//	Eigen::MatrixXd constrained_trans;
	//
	//	write_to_constraint_format(all_handle_list, false, 
	//		m_constrained_vertices, 
	//		m_constrained_vertex_positions,
	//		HandleTrans(),
	//		constrained_trans);
	//
	//#ifdef ADD_LIM_DEFORM
	//	DeformLocallyInjective::GetReference().UpdatePositionalConstraints(m_constrained_vertices);
	//#endif
	//#ifdef ADD_SKINNING_DEFORM
	//	DeformSkinning::GetReference().UpdatePositionalConstraints(m_constrained_vertices, !CONNECT_PICKING_TO_SKINNING);
	//	
	//	HandlePluginBase::GetReference().commit_temp_handles();
	//#endif
	//#ifdef ADD_VEGA_DEFORM
	//	DeformVega::GetReference().UpdatePositionalConstraints(m_constrained_vertices);
	//#endif
	//#ifdef ADD_PHYS_DEFORM
	//	//DeformPhys::GetReference().UpdatePositionalConstraints(m_constrained_vertices);
	//	if (true)//keep same in selection_changed and selected_vertices_moved
	//	{
	//		std::vector<IndexType> m_active_handles;
	//		Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic>  m_active_handle_positions;
	//		Eigen::MatrixXd constrained_trans;
	//
	//		write_to_constraint_format(all_handle_list, true, 
	//			m_active_handles, 
	//			m_active_handle_positions,
	//			HandleTrans(),
	//			constrained_trans);
	//		//DeformPhys::GetReference().UpdateConstraintVertexPositions(m_active_handles, m_active_handle_positions);
	//		DeformPhys::GetReference().UpdateConstraint(m_active_handles, m_active_handle_positions, constrained_trans);
	//	} 
	//	//DeformPhys::GetReference().UpdateConstraintVertexPositions(m_constrained_vertices, m_constrained_vertex_positions);
	//#endif
}

void HandlePluginBase::constraint_changed()
{
	//DeformPhysUI::GetReference().ConstraintChanged();
}

void extract_2d_m_from_3d_copy(const Eigen::MatrixXd& M0, Eigen::MatrixXd& M)
{
	assert(M0.cols() % 4 == 0);

	int num = M0.cols() / 4;
	M.resize(M0.rows(), 3 * num);

	for (int j = 0; j<num; j++)
	{
		M.col(3 * j + 0) = M0.col(4 * j + 0);
		M.col(3 * j + 1) = M0.col(4 * j + 1);
		M.col(3 * j + 2) = M0.col(4 * j + 3);
	}
}

void HandlePluginBase::selected_vertices_moved()
{
	vertices_moved(true);
}

void HandlePluginBase::active_vertices_moved()
{
	vertices_moved(false);
}

bool HandlePluginBase::GetConstraint(LinearConstraint23d& lc)
{

	//const Eigen::MatrixXd& M = DeformSkinning::GetReference().GetM(dim);

	if (!handleStructure.has_build)
	{
		return false;
	}

	{
		Eigen::VectorXi known;
		Eigen::MatrixXd knownValue;
		Eigen::MatrixXd Meq;
		Eigen::MatrixXd Peq;

		handleStructure.get_constraint(
			2,
			lc.LC2d.known,
			lc.LC2d.knownValue,
			lc.LC2d.Meq,
			lc.LC2d.Peq);

		handleStructure.get_constraint(
			3,
			lc.LC3d.known,
			lc.LC3d.knownValue,
			lc.LC3d.Meq,
			lc.LC3d.Peq);
	}



	//known.resize(m_active_handles.size());
	//for (int i=0; i<known.rows(); i++)
	//{
	//	known(i) = m_active_handles[i];
	//}
	//if(dim==2)
	//{
	//	knownValue = m_active_handle_positions.leftCols<2>();
	//} 
	//else
	//{
	//	knownValue = m_active_handle_positions;
	//}

	//switch(DeformSkinning::GetReference().getSkinningType()){
	//case LBS:
	//	{
	//		int nc = m_active_handles.size();

	//		//DeformSkinning::GetReference().set_M_matrix();
	//		const Eigen::MatrixXd& M = DeformSkinning::GetReference().GetM(dim);//= DeformSkinning::GetReference().M;
	//		Meq.resize(nc,M.cols());
	//		Eigen::VectorXi& I = HandlePluginBase::GetReference().HandleIndices;
	//		for (int i=0; i<nc; i++)
	//		{
	//			Meq.row(i) = M.row( I(m_active_handles[i]) );
	//		}
	//		Peq = m_active_handle_positions.cast<double>();

	//		if (dim==2)
	//		{
	//			Eigen::MatrixXd Meq_eff;
	//			Eigen::MatrixXd Peq_eff;
	//			extract_2d_m_from_3d_copy(Meq, Meq_eff);
	//			Peq_eff = Peq.leftCols<2>();
	//			Meq = Meq_eff;
	//			Peq = Peq_eff;
	//		}
	//		
	//		known.resize(0);
	//		knownValue.resize(0,2);

	//	}
	//	break;
	//default: //PBS
	//	break;
	//}	


	//print_matlab(Meq,"Meq");
	//print_matlab(Peq,"Peq");


	return true;
}

void HandlePluginBase::set_active_handles()
{
	//for (int i=0; i<all_handle_list.size(); i++)
	//{
	//	if (i==63||i==138||i==61||i==8)
	//	{
	//		all_handle_list[i].active = true;
	//	}
	//	else
	//	{
	//		all_handle_list[i].active = false;
	//	}
	//}

	selection_changed();
	constraint_changed();
	selected_vertices_moved();
}