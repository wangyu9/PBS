#include "MeshUI.h"
#include "FileDialog.h"


MeshDisplayUI::MeshDisplayUI() : MeshDisplay()
{
	bar = NULL;
}

void MeshDisplayUI::init(const std::string& name)
{
	if (bar == NULL)
	{
		// Create a tweak bar
		bar = new igl::ReTwBar;
		bar->TwNewBar(name.c_str());
		TwDefine( (" "+name+" color='76 76 127' ").c_str() ); // change default tweak bar size and color
		// size='250 500' color='76 76 127' position='500 300'
		bar->TwAddVarRO("NumberOfVertices", TW_TYPE_UINT32, &number_of_vertices,
			" label='Number of vertices' help='Displays number of vertices in mesh.'"
			" group='Mesh statistics'");
		bar->TwAddVarRO("NumberOfFaces", TW_TYPE_UINT32, &number_of_faces,
			" label='Number of faces' help='Displays number of faces in mesh.'"
			" group='Mesh statistics'");
		bar->TwAddVarRO("NumberOfTets", TW_TYPE_UINT32, &number_of_tets,
			" label='Number of tets' help='Displays number of tets in mesh.'"
			" group='Mesh statistics'");


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
		bar->TwAddButton("Load Property", open_dialog_property, this,
			" group='Load & Save'"
			" label='Open property'");
		bar->TwAddButton("Compile Mesh", compile_dialog_mesh, this,
			" group='Load & Save'"
			" label='Compile mesh' key=o help='Compile a mesh.'");//added by wangyu
		bar->TwAddButton("Load Pose Mesh", open_dialog_pose_mesh, this,
			" group='Load & Save'"
			" label='Open pose mesh'");
		//bar->TwAddVarRW( "Flip Y Coord", TW_TYPE_BOOLCPP, &bFlipYCoord,
		// " group='Load & Save'"
		// " label='Flip Y Coord'");//added by wangyu
		bar->TwAddVarCB("Flip Y Coord", TW_TYPE_BOOLCPP, SetFlipYCoordCB, GetFlipYCoordCB, this, " group='Load & Save'");
		bar->TwAddButton("Save Tets", save_dialog_tets, this,
			" group='Load & Save'"
			" label='Save tets'");// added by wangyu
		bar->TwAddButton("Send Status to Matlab", SendStatusToMatlabCB, this, " group='Load & Save'");//added by wangyu
																									  // ---------------------- Overlays ----------------------
		bar->TwAddVarRW("Wireframe", TW_TYPE_BOOLCPP, &show_lines,
			" group='Overlays'"
			" label='Wireframe' key=l help='Toggle wire frame of mesh'");
		bar->TwAddVarRW("Fill", TW_TYPE_BOOLCPP, &show_faces,
			" group='Overlays'"
			" label='Fill' key=t help='Display filled polygons of mesh'");
		bar->TwAddVarRW("ShowVertexId", TW_TYPE_BOOLCPP, &show_vertid,
			" group='Overlays'"
			" label='Show Vertex Labels' key=';' help='Toggle vertex indices'");
		bar->TwAddVarRW("ShowFaceId", TW_TYPE_BOOLCPP, &show_faceid,
			" group='Overlays'"
			" label='Show Faces Labels' key='CTRL+;' help='Toggle face"
			" indices'");

		bar->TwAddVarRW("Show Isolines", TW_TYPE_BOOLCPP, &show_isolines,
			" group='Overlays'"
			" label='Show Isolines' help='Toggle display of isolines of scalar property'");

		bar->TwAddVarCB("Isolines #", TW_TYPE_INT32, set_numIsoLevelsCB, get_numIsoLevelsCB, this,
			" group='Overlays'"
			" label='Isolines #' help='Number of Isolines of the scalar field to display'");

		// Shaders


#ifndef PREVIEW3D_NO_SHADERS

		// added by wangyu:
		TwEnumVal meshDrawingEV[NUM_MESH_DRAWING_TYPE] = {
			{ MESH_DRAWING_DEFAULT,"DEFAULT" },
			{ MESH_DRAWING_PBS,"PBS" }
		};
		TwType meshDrawingT = TwDefineEnum("Mesh Drawing Type", meshDrawingEV, NUM_MESH_DRAWING_TYPE);
		bar->TwAddVarCB("Mesh Drawing Type", meshDrawingT, SetMeshDrawingTypeCB, GetMeshDrawingTypeCB, this, " group='Scene'");


		bar->TwAddVarRW("Update Shader Attribs Every Frame", TW_TYPE_BOOLCPP, &update_shader_attribs_every_frame,
			" group='Scene'");
		bar->TwAddButton("Reload Shader", ReLoadShader, this, " group='Scene'");
#endif

		bar->TwAddVarCB("LineWidth", TW_TYPE_INT32, SetLineWidthCB, GetLineWidthCB, this,
			" group='Scene'");

		bar->TwAddVarCB("Show Texture", TW_TYPE_BOOLCPP, SetShowTextureCB, GetShowTextureCB, this,
			" label='Show Texture'");


		// ---------------------- DRAW OPTIONS ----------------------

		bar->TwAddVarRW("Mesh Alpha", TW_TYPE_FLOAT, &mesh_alpha, " group='Draw options' max=1.0 min=0.0 step=0.1");

		// added by wangyu:
		TwEnumVal colorBarTypeEV[NUM_OF_COLORBAR_TYPE] = {
			{ COLORBAR_IGL_DEFAULT, "DEFAULT" },
			{ COLORBAR_ZERO_ONE, "0-1" },
			{ COLORBAR_HIGHLIGHT_NEG, "0-1 (hightlight neg)" },
			{ COLORBAR_ZERO_ONE_GREY_RED, "0-1 grey-red" },
			{ COLORBAR_ZERO_ONE_NORMAL, "0-1" },
			{ COLORBAR_HIGHLIGHT_NEG_NORMAL, "0-1 (hightlight neg)" },
			{ COLORBAR_ZERO_ONE_GREY_RED_NORMAL, "0-1 grey-red" }
		};
		TwType colorBarT = TwDefineEnum("Color Bar Type", colorBarTypeEV, NUM_OF_COLORBAR_TYPE);
		bar->TwAddVarCB("Color Bar Type", colorBarT, SetColorBarTypeCB, GetColorBarTypeCB, this, " group='Draw options'");


		bar->TwAddButton("Send Color to MATALB", SendColorToMatlabCB, this, " group='Draw options'");


		bar->TwAddVarRW("Trackball", TW_TYPE_BOOLCPP, &show_trackball,
			" group='Scene'"
			" label='Show trackball' key='B' help='Show the trackball in the scene.'");



		bar->TwAddVarCB("Corner Threshold", TW_TYPE_DOUBLE, set_corner_thresholdCB, get_corner_thresholdCB, this,
			" group='Draw options'"
			" label='Corner Threshold' help='Angle difference (cosine) above which there is a sharp feature'");
		TwEnumVal NormalsTypeEV[NUM_COLOR_PRESET_IDS] = {
			{ PER_FACE,  "PER_FACE" },
			{ PER_VERTEX,"PER_VERTEX" },
			{ PER_CORNER,"PER_CORNER" }
		};
		TwType NormalsTypeTW =
			igl::ReTwDefineEnum("NormalsType", NormalsTypeEV, NUM_NORMALS_TYPE);
		bar->TwAddVarRW("NormalsType", NormalsTypeTW, &normals_type,
			" group='Draw options'"
			" label='Normals Type' key=T help='Toggle per face shading or per"
			" vertex shading or per corner shading.' ");

#define NUM_PER_VERTEX_NORMAL_TYPE 3
		TwEnumVal PerVertexNormalsTypeEV[NUM_PER_VERTEX_NORMAL_TYPE] = {
			{ igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_UNIFORM,  "UNIFORM_WEIGHTING" },
			{ igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_AREA,"AREA_WEIGHTING" },
			{ igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_ANGLE,"ANGLE_WEIGHTING" }
		};
		TwType PerVertexNormalsTypeTW =
			igl::ReTwDefineEnum("PerVertexNormalsType", PerVertexNormalsTypeEV, NUM_PER_VERTEX_NORMAL_TYPE);
		bar->TwAddVarRW("PerVertexNormalsType", PerVertexNormalsTypeTW, &per_vertex_normal_type,
			" group='Draw options'");

		bar->TwAddButton("InvertNormals", invert_normalsCB, this,
			" group='Draw options'"
			" label='Invert normals' key=i help='Invert normal directions for inside out meshes.' ");
		bar->TwAddVarRW("Draw Grid", TW_TYPE_BOOLCPP, &show_grid,
			" group='Draw options'"
			" label='Draw Grid' key=g help='Draw a grid below the mesh.' ");

		bar->TwAddVarRW("ShowOverlay", TW_TYPE_BOOLCPP, &show_overlay,
			" group='Draw options'"
			" label='Show overlay' key=o help='Show the overlay layers.' ");
		bar->TwAddVarRW("ShowOverlayDepth", TW_TYPE_BOOLCPP, &show_overlay_depth,
			" group='Draw options'"
			" label='Show overlay depth test' help='Enable the depth test for overlay layer.' ");


		bar->TwAddVarRW("LineColor", TW_TYPE_COLOR3F,
			&line_color,
			" label='Line color' group='Color' help='Select a outline color' ");

		bar->TwAddVarRW("Displacement", TW_TYPE_DIR3F, &draw_displacement, " group='Transformation'");
		bar->TwAddVarRW("Rotation", TW_TYPE_QUAT4F, &draw_rotation," group='Transformation'");
	}
}

#define CLASS_NAME MeshDisplayUI

void TW_CALL CLASS_NAME::open_dialog_mesh(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_open_file_path(fname);

	if (fname[0] == 0)
		return;

	bool bLoad = static_cast<CLASS_NAME *>(clientData)->load_mesh_from_file(fname);

	//if (bLoad)//wangyu
	//{
	//	static_cast<CLASS_NAME *>(clientData)->copy_working_folder(fname);
	//}
}

void TW_CALL CLASS_NAME::open_dialog_property(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_open_file_path(fname);

	if (fname[0] == 0)
		return;

	bool bLoad = static_cast<CLASS_NAME *>(clientData)->load_property_from_file(fname);

}

void TW_CALL CLASS_NAME::SendStatusToMatlabCB(void *clientData)
{
	static_cast<CLASS_NAME *>(clientData)->send_status_to_matlab();
}

void TW_CALL CLASS_NAME::SetShowTextureCB(const void *value, void *clientData)
{
	static_cast<CLASS_NAME *>(clientData)->SetShowTexture(*static_cast<const bool *>(value));
}

void TW_CALL CLASS_NAME::GetShowTextureCB(void *value, void *clientData)
{
	*static_cast<bool *>(value) = static_cast<CLASS_NAME*>(clientData)->GetShowTexture();
}

void TW_CALL CLASS_NAME::invert_normalsCB(void *clientData)
{
	CLASS_NAME *p3d = static_cast<CLASS_NAME *>(clientData);
	p3d->invert_normals = !p3d->invert_normals;
	p3d->is_compiled = false;
}

void TW_CALL CLASS_NAME::get_corner_thresholdCB(void *value, void *clientData)
{
	*static_cast<double *>(value) = static_cast<CLASS_NAME *>(clientData)->get_corner_threshold();
}

void TW_CALL CLASS_NAME::set_corner_thresholdCB(const void *value, void *clientData)
{
	static_cast<CLASS_NAME *>(clientData)->set_corner_threshold(*static_cast<const double *>(value));
}

void TW_CALL CLASS_NAME::get_numIsoLevelsCB(void *value, void *clientData)
{
	*static_cast<int *>(value) = static_cast<CLASS_NAME *>(clientData)->get_numIsoLevels();
}

void TW_CALL CLASS_NAME::set_numIsoLevelsCB(const void *value, void *clientData)
{
	static_cast<CLASS_NAME *>(clientData)->set_numIsoLevels(*static_cast<const int *>(value));
}

void TW_CALL CLASS_NAME::SetLineWidthCB(const void *value, void *clientData)
{
	static_cast<CLASS_NAME *>(clientData)->set_linewidth(*static_cast<const int *>(value));
}

void TW_CALL CLASS_NAME::GetLineWidthCB(void *value, void *clientData)
{
	*static_cast<int *>(value) = static_cast<CLASS_NAME *>(clientData)->linewidth;
}

void TW_CALL CLASS_NAME::open_dialog_texture(void *clientData)
{
	char fname[2048];

	fname[0] = 0;

	get_open_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<CLASS_NAME *>(clientData)->SetTexFilename(fname);

}

void TW_CALL CLASS_NAME::save_dialog_mesh(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_save_file_path(fname);

	if (fname[0] == 0)
		return;
	static_cast<CLASS_NAME *>(clientData)->save_mesh_to_file(fname);
}

void TW_CALL CLASS_NAME::save_dialog_tets(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_save_file_path(fname);

	if (fname[0] == 0)
		return;
	static_cast<CLASS_NAME *>(clientData)->save_tets_to_file(fname);
}


void TW_CALL CLASS_NAME::open_dialog_pose_mesh(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_open_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<CLASS_NAME *>(clientData)->load_pose_mesh_from_file(fname);

}

void TW_CALL CLASS_NAME::compile_dialog_mesh(void *clientData)
{
	static_cast<CLASS_NAME *>(clientData)->compile_mesh();
}

void TW_CALL CLASS_NAME::ReLoadShader(void *clientData)
{
	static_cast<CLASS_NAME *>(clientData)->reload_shader();
}