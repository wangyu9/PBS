//This file has been modified by wangyu

#ifdef __APPLE__
#   include <OpenGL/gl.h>
#   include <OpenGL/glu.h>
#   include <GLUT/glut.h>
#else
#   ifdef _WIN32
#       define NOMINMAX // disables min/max marco of WinDef.h
#       include <windows.h>
#       include <GL/glew.h>
#       include <GL/glut.h>
#   endif
#   include <GL/gl.h>
#   include <GL/glu.h>
#endif

#include "PickingPluginBase.h"
#include "PluginManager.h"
#include "FileDialog.h"
#include "utils/ViewerTrackball.h"


#include <viewer/path_anttweak.h>

#ifdef USING_IGL_HEADER_ONLY_MODE
#define IGL_HEADER_ONLY 
#endif

#include "igl/rgb_to_hsv.h"
#include "igl/hsv_to_rgb.h"

#include "igl/readOBJ.h"
#include "igl/adjacency_list.h"
#include <igl/writeDMAT.h>

// IGL ADDIN
#include <draw_frame.h>
#include <picking_util.h>
#include <transform.h>
#include <math_helper.h>

#ifdef _WIN32
#include <unordered_set>
#else
#include <tr1/unordered_set>
#endif

#define MAXNUMREGIONS 100


Eigen::VectorXi std_vector_to_eigen(const std::vector<int>& v)
{
	Eigen::VectorXi I(v.size());
	for (auto i = 0; i < v.size(); i++)
	{
		I(i) = v[i];
	}
	return I;
}

void PickingPluginBase::GetConstraint( Eigen::VectorXi& I, Eigen::MatrixXd& V)
{
	const int n = m_constrained_vertices.size();
	assert(n==m_constrained_vertex_positions.rows());

	I = std_vector_to_eigen(m_constrained_vertices);
	V = m_constrained_vertex_positions;
}


void TW_CALL SaveSelectionCB(void *clientData)
{
	PickingPluginBase* pnd = (PickingPluginBase*)clientData;
	pnd->saveSelection();
}

void TW_CALL LoadSelectionCB(void *clientData)
{
	PickingPluginBase* pnd = (PickingPluginBase*)clientData;

	char fname[FILE_DIALOG_MAX_BUFFER];
	fname[0] = 0;
	get_open_file_path(fname);

	if (fname[0] == 0)
		return;

	pnd->loadSelection(fname);
}

void TW_CALL SaveSelectedPosCB(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_save_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<PickingPluginBase *>(clientData)->saveSelectedPos(fname);
}

void TW_CALL SaveSelectedIndicesCB(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_save_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<PickingPluginBase *>(clientData)->saveSelectedIndices(fname);
}

void TW_CALL ResetCB(void *clientData)
{
	PickingPluginBase* pnd = (PickingPluginBase*)clientData;
	pnd->reset_mesh();
}


PickingPluginBase::PickingPluginBase()
//temp code, should be removed later 
//all_handle_list (HandlePlugin::GetReference().all_handle_list),
//select_handle_list (HandlePlugin::GetReference().select_handle_list),
//ray_traced_point( HandlePlugin::GetReference().ray_traced_point)
{
	//check in with the manager
	PluginManager().register_plugin(this);

	bar = NULL;

	selFilename = std::string("");
	selection_id.clear();


	m_enable_mouse_response = false;//Added by wangyu
	selectViaFace = true;//added by wangyu

	cursor_search_radius = 4.0;//default search radius added by wangyu
	handle_radius = 1.0;//default handle drawing radius factor added by wangyu

						// compute arbitary colors based on hue space not conflicting with material color
	region_colors.resize(MAXNUMREGIONS, 3);
	random_region_colors.resize(MAXNUMREGIONS, 3);

}

PickingPluginBase::~PickingPluginBase()
{
	delete bar;

};

// initialization (runs every time a mesh is loaded or cleared)
void PickingPluginBase::init(Preview3D *preview)
{
	PreviewPlugin::init(preview);

	//if(IMPORT_HANDLES_IN_PICKING)//2014.10.6
	//{
	// //import_vertices = &DeformSkinning::GetReference().Handles;
	// //import_faces = &DeformSkinning::GetReference().HandleFaces;
	// //import_vertex_to_vertices = &DeformSkinning::GetReference().vertex_to_vertices;
	// import_vertices = &HandlePlugin::GetReference().Handles;
	// import_faces = &HandlePlugin::GetReference().HandleFaces;
	// import_vertex_to_vertices = &HandlePlugin::GetReference().vertex_to_vertices;
	//}
	//else
	{
		import_vertices = m_preview->GetMainMesh().vertices;
		import_faces = m_preview->GetMainMesh().faces;
		import_vertex_to_vertices = &m_preview->GetMainMesh().vertex_to_vertices;
	}


	if (bar == NULL)
	{
		// Create a tweak bar
		bar = new igl::ReTwBar;
		bar->TwNewBar("Pick&Drag");
		bar->TwAddVarRW("Enable Selection", TW_TYPE_BOOLCPP, &m_enable_mouse_response, "group='Enable'");//added by wangyu 
		TwDefine(" Pick&Drag size='250 265' color='76 76 127' position='235 16' refresh=0.5"); // change default tweak bar size and color   
		bar->TwAddButton("Load Selection", LoadSelectionCB, this,
			" group='Save & Load' help='Loads current selection to file.'");
		bar->TwAddButton("Save Selection", SaveSelectionCB, this,
			" group='Save & Load' help='Saves current selection to file.'");
		bar->TwAddButton("Save Selected Position", SaveSelectedPosCB, this, "group='Save & Load' help='Save Only Position of Selected Vertices.'");
		bar->TwAddButton("Save Selected Indices", SaveSelectedIndicesCB, this, "group='Save & Load' help='Save Only Indices of Selected Vertices.'");
		bar->TwAddButton("Reset", ResetCB, this, "group='Save & Load'");


		TwEnumVal MouseModeEV[NUM_MOUSE_MODE] =
		{
			{ TRANSLATE_HANDLE,"TRANSLATE" },
			{ ROTATE_HANDLE, "ROTATE" },
			{ DELETE_SELECTION,"DESELECT" }
		};
		TwType MouseModeTW = igl::ReTwDefineEnum("MouseMode", MouseModeEV, NUM_MOUSE_MODE);
		bar->TwAddVarRW("Mouse Mode", MouseModeTW, &mouse_mode, "group='Options'");
		bar->TwAddVarRW("Search Radius", TW_TYPE_DOUBLE, &cursor_search_radius, " group='Options'");

		//bar->TwAddVarRW("Use Axes", TW_TYPE_BOOLCPP, &m_USE_DISPLAYED_AXES, " group='Options'");
		//bar->TwAddVarRW("Update in Motion", TW_TYPE_BOOLCPP, &m_UPDATE_WHILE_MOVING, " group='Options'");

		bar->TwAddVarRW("Random Colors", TW_TYPE_BOOLCPP, &random_colors, " group='Color'");
		bar->TwAddVarRW("Handle Color", TW_TYPE_COLOR3F, &handle_color, " group='Color' opened=true help='Select next handle color' colormode=hls");
		bar->TwAddVarRW("Handle Radius", TW_TYPE_DOUBLE, &handle_radius, "");

		bar->TwAddVarCB("Turn on Keyframing", TW_TYPE_BOOLCPP, SetTurnOnKeyFramingCB, GetTurnOnKeyFramingCB, this, " group='KeyFraming'");
		bar->TwAddButton("Load Keyframing Config", load_dialog_keyframing_config, this, " group='KeyFraming'");
	}


	selection_id.clear();
	selection_id.resize(import_vertices->rows(), -1);
	while (!m_free_regions.empty())
		m_free_regions.pop();
	for (int i = MAXNUMREGIONS; i >= 0; i--)
		m_free_regions.push(i);

	m_currentRegion = -1;
	mouse_mode = TRANSLATE_HANDLE;//wangyu Set Default here//TRANSLATE_HANDLE;

	m_meshIsLoaded = import_vertices->rows() >0;


	if (m_meshIsLoaded)
	{
		adjacency_list(*(import_faces), (*import_vertex_to_vertices));
	}


	// compute random selection color
	// find hue for current material color
	double hsv[3], rgb[3];
	rgb[0] = m_preview->material.g_MatDiffuse[0];
	rgb[1] = m_preview->material.g_MatDiffuse[1];
	rgb[2] = m_preview->material.g_MatDiffuse[2];
	igl::rgb_to_hsv(rgb, hsv);

	double golden_ratio_conjugate = 0.618033988749895;
	double curHue = 0;
	for (int i = 0; i<MAXNUMREGIONS; i++)
	{
		double r, g, b;
		double hue = std::fmod(curHue, 1.0) * 360;

		// find color not similar to yellow
		while (std::abs(hue - hsv[0]) < 15)
		{
			curHue += golden_ratio_conjugate;
			hue = std::fmod(curHue, 1.0) * 360;
		}

		igl::hsv_to_rgb(hue, 0.8, 1.0, r, g, b);
		random_region_colors.row(i) << r, g, b;
		curHue += golden_ratio_conjugate;
	}

	random_colors = true;

	handle_color[0] = random_region_colors.coeff(0, 0);
	handle_color[1] = random_region_colors.coeff(0, 1);
	handle_color[2] = random_region_colors.coeff(0, 2);

}

bool PickingPluginBase::Serialize(tinyxml2::XMLDocument* doc, tinyxml2::XMLElement*)
{
	return igl::save_ReAntTweakBar(bar, doc);
}

bool PickingPluginBase::Deserialize(tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement*)
{
	return igl::load_ReAntTweakBar(bar, doc);
}


//saves the current selection to a file
bool PickingPluginBase::saveSelection()
{
	char fname[FILE_DIALOG_MAX_BUFFER];
	fname[0] = 0;
	get_save_file_path(fname);

	if (fname[0] == 0)
		return false;

	selFilename = std::string(fname);
	FILE *f = fopen(fname, "w");
	if (NULL == f)
	{
		printf("IOError: %s could not be opened for writing...", fname);
		return false;
	}
#if 0
	for (long i = 0; i<import_vertices->rows(); i++)
		fprintf(f, "%d\n", selection_id[i]);
#else
	fprintf(f, "%lu\n", m_constrained_vertices.size());
	for (unsigned long i = 0; i<m_constrained_vertices.size(); i++)
		fprintf(f, "%d %d %.10f %.10f %.10f\n",
			m_constrained_vertices[i],
			selection_id[m_constrained_vertices[i]],
			m_constrained_vertex_positions(i, 0),
			m_constrained_vertex_positions(i, 1),
			m_constrained_vertex_positions(i, 2));
#endif
	fclose(f);

	return true;
}

//loads a selection to a file
bool PickingPluginBase::loadSelection(const char* fname)
{

	selFilename = std::string(fname);
	FILE *f = fopen(fname, "r");
	if (NULL == f)
	{
		printf("IOError: %s could not be opened for writing...", fname);
		return false;
	}

	selection_id.assign(import_vertices->rows(), -1);
	long unsigned int num_constrained = 0;
	fscanf(f, "%lu", &num_constrained);

	m_constrained_vertices.resize(num_constrained);
	m_constrained_vertex_positions.resize(num_constrained, 3);

	float x, y, z;
	for (unsigned long i = 0; i<m_constrained_vertices.size(); i++)
	{
		int sel_id, vid;
		fscanf(f, "%d %d %f %f %f\n",
			&vid,
			&sel_id,
			&x, &y, &z);
		m_constrained_vertex_positions.row(i) << x, y, z;
		m_constrained_vertices[i] = vid;
		selection_id[vid] = sel_id;
	}

	selection_changed();
	constraint_changed();//added by wangyu
	selected_vertices_moved();//added by wangyu

	fclose(f);
	colorSelection();

	return true;
}

//saves only the position of selected vertices
bool PickingPluginBase::saveSelectedPos(const char* fname)
{
	return igl::writeDMAT(fname, m_constrained_vertex_positions);
}

bool PickingPluginBase::saveSelectedIndices(const char* fname)
{
	Eigen::MatrixXi I(m_constrained_vertices.size(), 1);

	for (int i = 0; i < I.rows(); i++)
	{
		I(i, 0) = m_constrained_vertices[i];
	}

	return igl::writeDMAT(fname, I);
}

// keyboard callback
bool PickingPluginBase::keyDownEvent(unsigned char key, int modifiers, int, int)
{
	if (m_meshIsLoaded)
	{
		switch (key)
		{
		case 24: // ctrl + x
		{
			if (modifiers & Preview3D::CTRL)//CTRL + X: delete all selections
			{
				selection_id.clear();
				selection_id.resize(import_vertices->rows(), -1);
				while (!m_free_regions.empty())
					m_free_regions.pop();
				for (int i = MAXNUMREGIONS; i >= 0; i--)
					m_free_regions.push(i);

				colorSelection();
				return true;
			}
			break;
		}
		case 26: // ctrl + z
		{
			if (modifiers & Preview3D::CTRL)//CTRL + Z: reset the mesh to its initial coordinates
			{
				reset_mesh();
				return true;
			}
			break;
		}
		case 20: // ctrl + t
		{
			if (modifiers == (Preview3D::CTRL))//CTRL + T: set mouse mode to translation
			{
				mouse_mode = TRANSLATE_HANDLE;
				return true;
			}
			break;
		}
		case 18: // ctrl + r
		{
			if (modifiers == (Preview3D::CTRL))//CTRL + R: set mouse mode to rotation
			{
				mouse_mode = ROTATE_HANDLE;
				return true;
			}
			break;
		}
		case 4: // ctrl + d
		{
			if (modifiers == (Preview3D::CTRL))//CTRL + D: set mouse mode to deletion
			{
				mouse_mode = DELETE_SELECTION;
				return true;
			}
			break;
		}
		default:
			break;
		}
	}
	return false;
}

void PickingPluginBase::reset_mesh()
{

	for (int i = 0; i<selection_id.size(); i++)
		selection_id[i] = -1;

	m_constrained_vertices.clear();

	Eigen::Vector3d matColor = Eigen::Vector3d(m_preview->material.g_MatDiffuse[0], m_preview->material.g_MatDiffuse[1], m_preview->material.g_MatDiffuse[2]);
	for (int i = 0; i<m_preview->GetMainMesh().vertex_colors->rows(); i++)
	{
		m_preview->GetMainMesh().vertex_colors->row(i) = matColor;
	}
	for (int i = 0; i < m_preview->GetMainMesh().face_colors->rows(); i++)
	{
		m_preview->GetMainMesh().face_colors->row(i) = matColor;
	}

	m_preview->GetMainMesh().is_compiled = false;

	selection_changed();
}

void PickingPluginBase::lassoDownAlt(int mouse_x, int mouse_y, int selected_index)
{
	m_currentRegion = selection_id[selected_index];
	selection_changed();
}
void PickingPluginBase::lassoDownShift(int mouse_x, int mouse_y, int selected_index)
{
	int best = selected_index;

	if (mouse_mode == DELETE_SELECTION)
		// deletion of a selected region: check which region is under the mouse pointer
	{

		if (best >= 0)
		{
			if (selection_id[best] >= 0) //if a region was found, unselect it
			{
				m_currentRegion = selection_id[best];
				//directly delete the selected region and update the display
				for (int vi = 0; vi < import_vertices->rows(); ++vi)
				{
					if (selection_id[vi] == m_currentRegion)
						selection_id[vi] = -1;
				}

				m_free_regions.push(m_currentRegion);
				colorSelection();

				if (random_colors)
				{
					int id = m_free_regions.top();
					handle_color[0] = random_region_colors.coeff(id, 0);
					handle_color[1] = random_region_colors.coeff(id, 1);
					handle_color[2] = random_region_colors.coeff(id, 2);
				}

				get_constrained_positions(m_currentRegion, m_constrained_vertices, m_constrained_vertex_positions, true);
				selection_changed();
				constraint_changed();

				return;
			}
		}
	}
	else if (mouse_mode == TRANSLATE_HANDLE || mouse_mode == ROTATE_HANDLE)
		// translation, rotation of a selected region: check which region is under the mouse pointer
	{

		if (best >= 0 && selection_id[best] >= 0)  //if a region was found, mark it for translation/rotation
		{
			m_currentRegion = selection_id[best];
			return;
		}
	}
	return;
}
void PickingPluginBase::lassoDownNone(int mouse_x, int mouse_y, int selected_index)
{
	m_currentRegion = selection_id[selected_index];
	selection_changed();
}
void PickingPluginBase::lassoUp(const std::vector<int>& pickedVertices, int modifiers)
{
	//ok now divide picked vertices into connected components. Each selected region will be a single component.
	std::vector<int> component(pickedVertices.size(), 0);
	int nComp = connected_components(pickedVertices, (*import_vertex_to_vertices), component);

	if (1)//wangyu changed temply modifiers == Preview3D::CTRL)
	{
		//mark/unmark the picked vertices
		std::vector<int> regionIds;
		for (int i = 0; i < nComp; i++)
		{
			regionIds.push_back(m_free_regions.top());
			m_free_regions.pop();
		}

		//for marking, mark all possible regions (visible or not)
		bool changed = false;
		for (unsigned long vi = 0; vi < component.size(); vi++)
		{
			if (selection_id[pickedVertices[vi]] == -1)
			{
				int id = regionIds[component[vi] - 1];
				selection_id[pickedVertices[vi]] = id;

				region_colors.row(id) = Eigen::Vector3d(handle_color[0], handle_color[1], handle_color[2]);

				changed = true;
			}
		}

		if (random_colors)
		{
			int id = m_free_regions.top();
			handle_color[0] = random_region_colors.coeff(id, 0);
			handle_color[1] = random_region_colors.coeff(id, 1);
			handle_color[2] = random_region_colors.coeff(id, 2);
		}

		//constrained vertex index changes
		if (changed)
		{
			///vertices in constrained_vertices have to be ORDERED !!!
			int num_constrained_vertices = import_vertices->rows() - std::count(selection_id.begin(), selection_id.end(), -1);

			m_constrained_vertex_positions = Eigen::MatrixXd(num_constrained_vertices, 3);
			m_constrained_vertices.resize(num_constrained_vertices);

			int count = 0;
			for (long vi = 0; vi < import_vertices->rows(); ++vi)
				if (selection_id[vi] >= 0)
				{
					m_constrained_vertices[count] = vi;
					m_constrained_vertex_positions.row(count) = import_vertices->row(vi);
					count++;
				}

			get_constrained_positions(m_currentRegion, m_constrained_vertices, m_constrained_vertex_positions, true);
			selection_changed();
			constraint_changed();
		}
	}
	else
	{
		if (modifiers == Preview3D::SHIFT)
		{
			//for unmarking, sort component and only delete top one
			Eigen::MatrixXd pv(pickedVertices.size(), 3);
			for (unsigned long vi = 0; vi < pickedVertices.size(); vi++)
				pv.row(vi) = import_vertices->row(pickedVertices[vi]);

			double min_z = 1e10;
			int closest_component = -1;
			for (int c = 0; c < nComp; ++c)
			{
				RowVector3 centroid;
				computeRegionCentroid(&pv, component, c + 1, centroid);
				double x, y, z;
				gluProject(centroid(0),
					centroid(1),
					centroid(2),
					m_preview->camera.m_modelview_matrix,
					m_preview->camera.m_projection_matrix,
					m_preview->camera.m_viewport, &x, &y, &z);
				if (z < min_z)
				{
					closest_component = c;
					min_z = z;
				}
			}
			bool changed = false;
			int regionId = 0;
			for (unsigned long vi = 0; vi < component.size(); vi++)
			{
				if (component[vi] == closest_component + 1)
				{
					regionId = selection_id[pickedVertices[vi]];
					selection_id[pickedVertices[vi]] = -1;
					changed = true;
				}
			}

			if (changed)
			{
				m_free_regions.push(regionId);
				selection_changed();
			}
		}
	}

	colorSelection();

	return;
}
void PickingPluginBase::lassoMove(float translation[3])
{
	switch (mouse_mode)
	{
	case TRANSLATE_HANDLE: // translation: calculate the current translation
	{
		if (m_currentRegion < 0)//(!activate || m_currentRegion < 0)
			break;

		translate_vertices(m_currentRegion, translation);
		//update the constrained vertices (selected regions and handle), calculate the deformation and update the vertices
		get_constrained_positions(m_currentRegion, m_constrained_vertices, m_constrained_vertex_positions, true);
		selected_vertices_moved();

		return;
		break;
	}
	case ROTATE_HANDLE:
	{
		// TODO: implement the rotation part.
#if 0
		if (!activate || m_currentRegion < 0)
			break;

		get_rotation(mouse_x,
			from_x,
			mouse_y,
			from_y,
			depth_at_mouse_pos,
			m_preview->camera.m_modelview_matrix,
			m_preview->camera.m_projection_matrix,
			m_preview->camera.m_viewport,
			rotation,
			(m_USE_DISPLAYED_AXES) ? axis_is_selected : 0);

		if (m_UPDATE_WHILE_MOVING)
		{
			//rotation by the centroid: center of the coordinate frame (if applicable) does not move
			//update the constrained vertices (selected regions and handle), calculate the deformation and update the vertices
			get_constrained_positions(m_currentRegion, m_constrained_vertices, m_constrained_vertex_positions, true);
			selected_vertices_moved();
		}
#endif
		return;

		break;
	}
	case DELETE_SELECTION:
	{
		break;
	}
	}
}

#include <GL_helper.h>
void PickingPluginBase::push_gl_settings()
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

void PickingPluginBase::pop_gl_settings()
{
	bool enable_depth = true;
	pop_settings(enable_depth);
}

//mouse callback
bool PickingPluginBase::mouseDownEvent(int mouse_x, int mouse_y, int button, int modifiers)
{
	if (!m_enable_mouse_response)
	{
		return false;
	}

	if (m_meshIsLoaded)
	{

#if 1
		push_gl_settings();

		bool any_response = false;
		switch (button)
		{
		case 0:
			m_currentRegion = -1;
			break;
		}

		{// lasso 
			bool callLassoDown = false;
			int inserted_index = -1;
			any_response |= lasso.mouseDown(
				mouse_x, mouse_y,
				modifiers,
				*import_vertices,
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
					lassoDownShift(mouse_x, mouse_y, inserted_index);
					break;
				case Preview3D::CTRL:
					break;
				default:
					lassoDownNone(mouse_x, mouse_y, inserted_index);
				}
			}
		}

		pop_gl_settings();

		return any_response;

#else
		switch (button)
		{
		case 0: // left click
		{

			// save state before translation/rotation
			from_x = mouse_x;
			from_y = mouse_y;

			previous_m_cpoint3 << m_cpoint3;
			*previous_vertices << *import_vertices;

			glReadPixels(mouse_x, m_preview->camera.m_viewport[3] - mouse_y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth_at_mouse_pos);

			m_currentRegion = -1;

			activate = false;

			if (modifiers == Preview3D::ALT)
			{
				selection_changed();
				return true;
			}

			if ((modifiers == (Preview3D::CTRL | Preview3D::SHIFT) || modifiers == Preview3D::CTRL))
				// selection/undo selection mode : start tracking mouse path for deselecting enclosed vertices (lasso tool)
			{
				mousePoints.clear();
				//insert the first point
				mousePoints.push_back(std::make_pair(mouse_x, mouse_y));
				selecting = true;
				return true;
			}
			else if (mouse_mode == DELETE_SELECTION)
				// deletion of a selected region: check which region is under the mouse pointer
			{

				std::vector< std::pair<double, IndexType> > H;

				int best;
				double minsqdist;
				if (selectViaFace)
				{
					best = pick_face_and_vertex(mouse_x,
						mouse_y,
						m_preview->camera,
						import_vertices,
						import_faces,
						H);
				}
				else
				{
					best = pick_nearest_vertex(mouse_x,
						mouse_y,
						minsqdist,
						m_preview->camera,
						import_vertices);
				}


				if (best >= 0)
				{
					if (selection_id[best] >= 0) //if a region was found, unselect it
					{
						m_currentRegion = selection_id[best];
						//directly delete the selected region and update the display
						for (int vi = 0; vi<import_vertices->rows(); ++vi)
						{
							if (selection_id[vi] == m_currentRegion)
								selection_id[vi] = -1;
						}

						m_free_regions.push(m_currentRegion);
						colorSelection();

						if (random_colors)
						{
							int id = m_free_regions.top();
							handle_color[0] = random_region_colors.coeff(id, 0);
							handle_color[1] = random_region_colors.coeff(id, 1);
							handle_color[2] = random_region_colors.coeff(id, 2);
						}

						get_constrained_positions(m_currentRegion, m_constrained_vertices, m_constrained_vertex_positions, true);
						selection_changed();
						constraint_changed();

						return true;
					}
				}


			}
			else if (mouse_mode == TRANSLATE_HANDLE || mouse_mode == ROTATE_HANDLE)
				// translation, rotation of a selected region: check which region is under the mouse pointer
			{
				std::vector< std::pair<double, IndexType> > H;

				int best;
				if (selectViaFace)
				{
					best = pick_face_and_vertex(mouse_x,
						mouse_y,
						m_preview->camera,
						import_vertices,
						import_faces,
						H);
				}
				else
				{
					double minsqdist;
					best = pick_nearest_vertex(mouse_x,
						mouse_y,
						minsqdist,
						m_preview->camera,
						import_vertices);
				}

				if (best >= 0 && selection_id[best] >= 0)  //if a region was found, mark it for translation/rotation
				{
					m_currentRegion = selection_id[best];
					activate = true;
					return true;
				}
			}

			break;
		}

		case 2://right click
		{
			//if(modifiers == Preview3D::ALT)
			//{
			// int hit_index;
			// if(HandlePlugin::GetReference().ray_trace_mesh(mouse_x,mouse_y,ray_traced_point.x,ray_traced_point.y,ray_traced_point.z,hit_index))
			//  HandlePlugin::GetReference().cursor_update(ray_traced_point.x,ray_traced_point.y,ray_traced_point.z,hit_index,'r');
			// return true;
			//}
			break;
		}
		// case 2://middle click // this seems not to happen
		//{
		//	  //if(modifiers == Preview3D::ALT)
		//	  //{
		//		 // int hit_index;
		//		 // if(ray_trace_mesh(mouse_x,mouse_y,ray_traced_point.x,ray_traced_point.y,ray_traced_point.z,hit_index))
		//			//  DeformSkinning::GetReference().cursor_update(ray_traced_point.x,ray_traced_point.y,ray_traced_point.z,hit_index,'m');
		//		 // return true;
		//	  //}
		//	  //break;
		//}
		}
#endif

	}
	return false;
};


//colors selection with different colors per region
void PickingPluginBase::colorSelection()
{
	//if (IMPORT_HANDLES_IN_PICKING)//2014.10.6
	//{
	//	return;
	//}

	if (false)
	{
		//default
		if (m_preview->GetMainMesh().vertex_colors->rows() != import_vertices->rows())
		{
			m_preview->GetMainMesh().vertex_colors->resize(import_vertices->rows(), 3);
			m_preview->GetMainMesh().vertex_colors->col(0).setConstant(m_preview->material.g_MatDiffuse[0]);
			m_preview->GetMainMesh().vertex_colors->col(1).setConstant(m_preview->material.g_MatDiffuse[1]);
			m_preview->GetMainMesh().vertex_colors->col(2).setConstant(m_preview->material.g_MatDiffuse[2]);
		}


		double golden_ratio_conjugate = 0.618033988749895;
		for (int i = 0; i < import_vertices->rows(); i++)
		{
			int region_id = selection_id[i];
			if (region_id != -1)
				m_preview->GetMainMesh().vertex_colors->row(i) = region_colors.row(region_id);
		}

		m_preview->GetMainMesh().is_compiled = false;
	}
	else
	{


		//m_preview->set_vertex_colors(,)

		int num_to_set = 0;

		for (int i = 0; i < import_vertices->rows(); i++)
		{
			int region_id = selection_id[i];
			if (region_id != -1)
				num_to_set++;//m_preview->vertex_colors->row(i) = region_colors.row(region_id);
		}

		Eigen::VectorXi I(num_to_set);
		Eigen::MatrixXd C(num_to_set, 3);

		int k = 0;
		for (int i = 0; i < import_vertices->rows(); i++)
		{
			int region_id = selection_id[i];
			if (region_id != -1)
			{
				I(k) = i;
				C.row(k) = region_colors.row(region_id);
				k++;
			}
		}

		m_preview->GetMainMesh().set_face_colors_from_vertex(I, C);
	}



}

bool PickingPluginBase::mouseUpEvent(int mouse_x, int mouse_y, int, int modifiers)
{
	if (!m_enable_mouse_response)
	{
		return false;
	}

	if (m_meshIsLoaded)
	{

#if 1
		push_gl_settings();

		bool any_response = false;
		{//lasso
			bool callLassoUp = false;
			bool callLassoMove = false;
			std::vector<int> indics_inside_lasso;
			bool shouldMove = lasso.is_activate();
			any_response |= lasso.mouseUp(
				mouse_x, mouse_y,
				*import_vertices,
				callLassoUp,
				callLassoMove,
				indics_inside_lasso
				);
			if (shouldMove) lassoMove(lasso.translation);
			if (callLassoUp) lassoUp(indics_inside_lasso, modifiers);
		}

		pop_gl_settings();
		return any_response;
#else
		if (selecting) // if we are in selection mode: gather up any faces inside the bounding box of the lasso, and then check separately which of the vertices are inside the lasso polyline
		{
			if (mouse_mode == TRANSLATE_HANDLE || mouse_mode == ROTATE_HANDLE || mouse_mode == DELETE_SELECTION)
			{
				//for selection and deselection: find vertices inside the polyline the user has drawn. Then mark/unmark them.
				//step 1: use the bounding box of the polyline and OpenGL picking to isolate potentially selected faces
				int minx = 1e5, maxx = -1, miny = 1e5, maxy = -1;
				for (unsigned long i = 0; i<mousePoints.size(); ++i)
				{
					minx = std::min(minx, mousePoints[i].first);
					maxx = std::max((float)maxx, (float)mousePoints[i].first);
					miny = std::min(miny, mousePoints[i].second);
					maxy = std::max(maxy, mousePoints[i].second);
				}
				int width = maxx - minx;
				int height = maxy - miny;
				int cx = minx + 0.5*width;
				int cy = miny + 0.5*height;

				std::vector< std::pair<double, IndexType> > H;
				if (selectViaFace)
				{
					pick_face_and_vertex(mouse_x,
						mouse_y,
						m_preview->camera,
						import_vertices,
						import_faces,
						H,
						std::vector<int>(0),
						std::vector<int>(0),
						maxx - minx,
						maxy - miny,
						&cx,
						&cy);
				}
				else
				{
					double minsqdist;
					pick_nearest_vertex(mouse_x,
						mouse_y,
						minsqdist,
						m_preview->camera,
						import_vertices);
				}

				//step2: check all vertices of the selected faces and see if they fall inside the polyline or not
				std::vector<std::vector<IndexType > >poly;
				poly.resize(mousePoints.size());
				for (unsigned long i = 0; i<mousePoints.size(); ++i)
				{
					poly[i].resize(2);
					poly[i][0] = mousePoints[i].first;
					poly[i][1] = m_preview->camera.m_viewport[3] - mousePoints[i].second;
				}

				std::tr1::unordered_set<int> doneVertices;
				std::tr1::unordered_set<int> pv;
				std::vector<int > pickedVertices(0);

				for (unsigned long ii = 0; ii<H.size(); ii++)
				{
					IndexType fi = H[ii].second;
					double x, y, z;
					for (int vi = 0; vi<import_faces->cols(); vi++)
					{
						int vertex = (*(import_faces))(fi, vi);
						if (doneVertices.insert(vertex).second)
						{
							gluProject((*import_vertices)(vertex, 0),
								(*import_vertices)(vertex, 1),
								(*import_vertices)(vertex, 2),
								m_preview->camera.m_modelview_matrix,
								m_preview->camera.m_projection_matrix,
								m_preview->camera.m_viewport, &x, &y, &z);

							if (inpoly(poly, (unsigned int)x, (unsigned int)y))
							{
								if (pv.insert(vertex).second)
									pickedVertices.push_back(vertex);

								if (vertexSelection == false)
								{
									for (int jj = 0; jj<(*import_vertex_to_vertices)[vertex].size(); jj++)
									{
										int vertex_ = (*import_vertex_to_vertices)[vertex][jj];
										if (pv.insert(vertex_).second)
											pickedVertices.push_back(vertex_);
									}
								}
								break;
							}
						}
					}
				}

				//ok now divide picked vertices into connected components. Each selected region will be a single component.
				std::vector<int> component(pickedVertices.size(), 0);
				int nComp = connected_components(pickedVertices, (*import_vertex_to_vertices), component);

				if (modifiers == Preview3D::CTRL)
				{
					//mark/unmark the picked vertices
					std::vector<int> regionIds;
					for (int i = 0; i<nComp; i++)
					{
						regionIds.push_back(m_free_regions.top());
						m_free_regions.pop();
					}

					//for marking, mark all possible regions (visible or not)
					bool changed = false;
					for (unsigned long vi = 0; vi<component.size(); vi++)
					{
						if (selection_id[pickedVertices[vi]] == -1)
						{
							int id = regionIds[component[vi] - 1];
							selection_id[pickedVertices[vi]] = id;

							region_colors.row(id) = Eigen::Vector3d(handle_color[0], handle_color[1], handle_color[2]);

							changed = true;
						}
					}

					if (random_colors)
					{
						int id = m_free_regions.top();
						handle_color[0] = random_region_colors.coeff(id, 0);
						handle_color[1] = random_region_colors.coeff(id, 1);
						handle_color[2] = random_region_colors.coeff(id, 2);
					}

					//constrained vertex index changes
					if (changed)
					{
						get_constrained_positions(m_currentRegion, m_constrained_vertices, m_constrained_vertex_positions, true);
						selection_changed();
						constraint_changed();
					}
				}
				else
				{
					if (modifiers == Preview3D::SHIFT)
					{
						//for unmarking, sort component and only delete top one
						Eigen::MatrixXd pv(pickedVertices.size(), 3);
						for (unsigned long vi = 0; vi<pickedVertices.size(); vi++)
							pv.row(vi) = import_vertices->row(pickedVertices[vi]);

						double min_z = 1e10;
						int closest_component = -1;
						for (int c = 0; c<nComp; ++c)
						{
							RowVector3 centroid;
							computeRegionCentroid(&pv, component, c + 1, centroid);
							double x, y, z;
							gluProject(centroid(0),
								centroid(1),
								centroid(2),
								m_preview->camera.m_modelview_matrix,
								m_preview->camera.m_projection_matrix,
								m_preview->camera.m_viewport, &x, &y, &z);
							if (z <min_z)
							{
								closest_component = c;
								min_z = z;
							}
						}
						bool changed = false;
						int regionId = 0;
						for (unsigned long vi = 0; vi<component.size(); vi++)
						{
							if (component[vi] == closest_component + 1)
							{
								regionId = selection_id[pickedVertices[vi]];
								selection_id[pickedVertices[vi]] = -1;
								changed = true;
							}
						}

						if (changed)
						{
							m_free_regions.push(regionId);
							selection_changed();
						}
					}
				}

				mousePoints.clear();
				colorSelection();
				selecting = false;
				return true;
			}

			selection_changed();
			mousePoints.clear();
			colorSelection();
			selecting = false;
			return true;
			//}


		}

		switch (mouse_mode)
		{
		case TRANSLATE_HANDLE:
		case ROTATE_HANDLE:
		{
			if (!m_UPDATE_WHILE_MOVING && m_currentRegion >= 0)
			{
				get_constrained_positions(m_currentRegion, m_constrained_vertices, m_constrained_vertex_positions, true);
				selected_vertices_moved();
			}
			break;
		}
		default:
			break;

		}

		axis_is_selected[0] = false;
		axis_is_selected[1] = false;
		axis_is_selected[2] = false;
		m_currentRegion = -1;
		translation[0] = 0.;
		translation[1] = 0.;
		translation[2] = 0.;
		rotation[0] = 0.;
		rotation[1] = 0.;
		rotation[2] = 0.;
		rotation[3] = 1.;
		m_currentRegion = -1;
		activate = false;
#endif   
	}
	return false;
}

bool PickingPluginBase::mouseMoveEvent(int mouse_x, int mouse_y)
{
	if (!m_enable_mouse_response)
	{
		return false;
	}

	if (m_meshIsLoaded)
	{
#if 1
		push_gl_settings();

		bool any_response = false;
		{//lasso
			bool callLassoMove = false;
			bool shouldMove = lasso.is_activate();
			any_response |= lasso.mouseMove(mouse_x, mouse_y, callLassoMove, false);
			if (shouldMove) lassoMove(lasso.translation);
		}

		pop_gl_settings();
		return any_response;
#else

		if (selecting)//if in selection mode: lasso tool (add mouse location to the lasso polyline)
		{
			//keep tracking the mouse: gather up the points
			mousePoints.push_back(std::make_pair(mouse_x, mouse_y));
			return true;
		}
		switch (mouse_mode)
		{
		case TRANSLATE_HANDLE: // translation: calculate the current translation
		{
			if (!activate || m_currentRegion <0)
				break;

			get_translation(mouse_x,
				from_x,
				mouse_y,
				from_y,
				depth_at_mouse_pos,
				m_preview->camera.m_modelview_matrix,
				m_preview->camera.m_projection_matrix,
				m_preview->camera.m_viewport,
				translation,
				(m_USE_DISPLAYED_AXES) ? axis_is_selected : 0);

			if (m_UPDATE_WHILE_MOVING)
			{
				if (m_USE_DISPLAYED_AXES)
				{
					//move the center of the coordinate frame (required for the coordinate frame to be displayed correctly)
					m_cpoint3[0] = previous_m_cpoint3[0] + translation[0];
					m_cpoint3[1] = previous_m_cpoint3[1] + translation[1];
					m_cpoint3[2] = previous_m_cpoint3[2] + translation[2];
				}

				//update the constrained vertices (selected regions and handle), calculate the deformation and update the vertices
				get_constrained_positions(m_currentRegion, m_constrained_vertices, m_constrained_vertex_positions, true);
				selected_vertices_moved();

			}
			return true;
			break;
		}
		case ROTATE_HANDLE:
		{
			if (!activate || m_currentRegion <0)
				break;

			get_rotation(mouse_x,
				from_x,
				mouse_y,
				from_y,
				depth_at_mouse_pos,
				m_preview->camera.m_modelview_matrix,
				m_preview->camera.m_projection_matrix,
				m_preview->camera.m_viewport,
				rotation,
				(m_USE_DISPLAYED_AXES) ? axis_is_selected : 0);

			if (m_UPDATE_WHILE_MOVING)
			{
				//rotation by the centroid: center of the coordinate frame (if applicable) does not move
				//update the constrained vertices (selected regions and handle), calculate the deformation and update the vertices
				get_constrained_positions(m_currentRegion, m_constrained_vertices, m_constrained_vertex_positions, true);
				selected_vertices_moved();
			}
			return true;

			break;
		}
		case DELETE_SELECTION:
		{
			break;
		}
		}

		//if we are in axes selection mode, we need to check if the mouse is on a mesh region that is selected, and only then should we pick an axes
		if (m_USE_DISPLAYED_AXES && (!m_preview->down || !activate))
		{
			axis_is_selected[0] = false; axis_is_selected[1] = false; axis_is_selected[2] = false;


			std::vector< std::pair<double, IndexType> > H;
			int vertex_under_mouse = pick_face_and_vertex(mouse_x,
				mouse_y,
				m_preview->camera,
				import_vertices,
				import_faces,
				H);
			//if on a selected region, pick the axis
			if (vertex_under_mouse >= 0 && selection_id[vertex_under_mouse] >= 0)
			{
				//
				m_currentRegion = selection_id[vertex_under_mouse];

				computeRegionCentroid(import_vertices, selection_id, m_currentRegion, m_cpoint3);
				double x1, y1, z1;
				double x2, y2, z2;
				double xc1, yc1, zc1;
				double xc2, yc2, zc2;

				gluProject(m_cpoint3(0),
					m_cpoint3(1),
					m_cpoint3(2),
					m_preview->camera.m_modelview_matrix,
					m_preview->camera.m_projection_matrix,
					m_preview->camera.m_viewport,
					&xc1, &yc1, &zc1);

				RowVector3 aabb, AABB;
				computeRegionBoundingBox(import_vertices, selection_id, m_currentRegion, aabb, AABB);
				//move the coordinate frame to the front of the scene
				gluProject(aabb(0),
					aabb(1),
					aabb(2),
					m_preview->camera.m_modelview_matrix,
					m_preview->camera.m_projection_matrix,
					m_preview->camera.m_viewport,
					&x1, &y1, &z1);

				gluProject(AABB(0),
					AABB(1),
					AABB(2),
					m_preview->camera.m_modelview_matrix,
					m_preview->camera.m_projection_matrix,
					m_preview->camera.m_viewport,
					&x2, &y2, &z2);

				gluUnProject(xc1, yc1, std::min(z1, z2) - 0.01,
					m_preview->camera.m_modelview_matrix,
					m_preview->camera.m_projection_matrix,
					m_preview->camera.m_viewport,
					&xc2, &yc2, &zc2);
				m_cpoint3 << xc2, yc2, zc2;

				int ax = pick_axes(m_cpoint3,
					axes_length,
					axes_radius,
					mouse_x,
					mouse_y,
					m_preview->camera,
					10,
					10);

				axis_is_selected[ax] = true;
			}
		}
#endif
	}
	return false;
}

bool PickingPluginBase::mouseScrollEvent(int /*mouse_x*/, int /*mouse_y*/, float /*delta*/) { return false; };

void PickingPluginBase::preDraw(int currentTime)
{
	if (turn_on_handle_key_frame)
	{
		update_keyframing_selection();
	}
};


//stuff that is drawn by the plugin after the previewer has displayed the mesh
//first draw 3d, then 2d
void PickingPluginBase::postDraw(int currentTime)
{

	if (!m_meshIsLoaded)
	{
		return;
	}

	glDisable(GL_LIGHTING);

	double diameter = std::min(m_preview->GetMainMesh().diameter*0.004, m_preview->GetMainMesh().avg_edge_length*0.3);

#if 1
	//push_gl_settings();

	
	for (int i = 0; i < m_constrained_vertices.size(); i++)
	{
		int region_id = selection_id[m_constrained_vertices[i]];

		if (region_id >= 0)
		{
			glPushMatrix();

			GLUquadricObj *quadric;
			quadric = gluNewQuadric();
			gluQuadricDrawStyle(quadric, GLU_FILL);

			glTranslatef(
				m_constrained_vertex_positions(i,0),
				m_constrained_vertex_positions(i,1),
				m_constrained_vertex_positions(i,2)
				);

			Eigen::Vector3d color = region_colors.row(region_id);
			glColor3d(color[0], color[1], color[2]);
			gluSphere(quadric, diameter*handle_radius, 7, 3);

			gluDeleteQuadric(quadric);

			glPopMatrix();
		}
	}

	//pop_gl_settings();

#else
	for (int i = 0; i<import_vertices->rows(); i++)
	{
		int region_id = selection_id[i];

		if (region_id >= 0)
		{
			glPushMatrix();

			GLUquadricObj *quadric;
			quadric = gluNewQuadric();
			gluQuadricDrawStyle(quadric, GLU_FILL);

			glTranslatef(import_vertices->coeff(i, 0), import_vertices->coeff(i, 1), import_vertices->coeff(i, 2));
			Eigen::Vector3d color = region_colors.row(region_id);
			glColor3d(color[0], color[1], color[2]);
			gluSphere(quadric, diameter*handle_radius, 7, 3);

			gluDeleteQuadric(quadric);

			glPopMatrix();
		}
	}
#endif

	if (m_preview->light.use_lighting)
		glEnable(GL_LIGHTING);
	else
		glDisable(GL_LIGHTING);
	
#if 1
	return lasso.draw();
#else
	//if in the process of selection, draw a dashed line with the lasso (2-d drawing)
	if (selecting)
	{
		if (mousePoints.size())
		{
			GLint viewport[4];
			glGetIntegerv(GL_VIEWPORT, viewport);

			glMatrixMode(GL_PROJECTION);
			glPushMatrix();
			glLoadIdentity();
			gluOrtho2D((GLdouble)viewport[0], (GLdouble)viewport[2], (GLdouble)viewport[1], (GLdouble)viewport[3]);
			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glLoadIdentity();

			glDisable(GL_DEPTH_TEST);
			glDisable(GL_CULL_FACE);
			glDisable(GL_BLEND);
			glDisable(GL_TEXTURE_2D);
			glDisable(GL_LIGHTING);
			glDisable(GL_LIGHT0);

			float linewidth;
			glGetFloatv(GL_LINE_WIDTH, &linewidth);
			glLineWidth(2.);
			glLineStipple(2, 0xAAAA);
			glEnable(GL_LINE_STIPPLE);
			glColor3d(0.5, 0.5, 0.5);
			glBegin(GL_LINE_STRIP);
			for (unsigned long i = 0; i <mousePoints.size(); ++i)
				glVertex2d(mousePoints[i].first, viewport[3] - mousePoints[i].second);
			glEnd();
			glDisable(GL_LINE_STIPPLE);
			glLineWidth(linewidth);

			//        glutSwapBuffers();

			glEnable(GL_DEPTH_TEST);

			glMatrixMode(GL_MODELVIEW);
			glPopMatrix();
			glMatrixMode(GL_PROJECTION);
			glPopMatrix();
			glMatrixMode(GL_MODELVIEW);
		}
	}
	return;//for connected skinnning mode return here.
#endif

#if 0
	if (m_meshIsLoaded)
	{
		axes_length = 0.6 / m_preview->camera.zoom;
		axes_radius = 0.006 / m_preview->camera.zoom;

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		glMultMatrixd(m_preview->camera.m_projection_matrix);
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();
		glMultMatrixd(m_preview->camera.m_modelview_matrix);

		if (!m_UPDATE_WHILE_MOVING && m_currentRegion >= 0 && activate)
		{
			get_constrained_positions(m_currentRegion, m_constrained_vertices, m_constrained_vertex_positions, true);

			glEnable(GL_BLEND);
			glEnable(GL_POLYGON_OFFSET_FILL); // Avoid Stitching!
			glPolygonOffset(1.0, 1.0);

			if (m_preview->light.use_lighting)
				glEnable(GL_LIGHTING);
			else
				glDisable(GL_LIGHTING);

			// paint faces that have at least one picked vertex, with shading
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			glEnable(GL_COLOR_MATERIAL);
			float linewidth;
			glGetFloatv(GL_LINE_WIDTH, &linewidth);
			glLineWidth(2.);
			glLineStipple(1, 0xAAAA);
			glEnable(GL_LINE_STIPPLE);
			glColor3d(region_colors.coeff(m_currentRegion, 0), region_colors.coeff(m_currentRegion, 1), region_colors.coeff(m_currentRegion, 2));

			glBegin(GL_TRIANGLES);
			for (int fi = 0; fi<import_faces->rows(); ++fi)
			{
				int firstPickedVertex = -1;
				for (int vi = 0; vi<import_faces->cols(); ++vi)
					if (selection_id[(*(import_faces))(fi, vi)] == m_currentRegion)
					{
						firstPickedVertex = vi;
						break;
					}
				if (firstPickedVertex == -1)
					continue;


				for (int vi = 0; vi<import_faces->cols(); ++vi)
				{
					RowVector3 p0, p1;
					int vertex_id = (*(import_faces))(fi, vi);
					if (selection_id[vertex_id] == m_currentRegion)
					{
						int index = std::find(m_constrained_vertices.begin(),
							m_constrained_vertices.end(),
							vertex_id) - m_constrained_vertices.begin();
						p0 = m_constrained_vertex_positions.row(index);
					}
					else
						p0 = import_vertices->row(vertex_id);

					glVertex3f(p0[0], p0[1], p0[2]);
				}

			}
			glEnd();
			glDisable(GL_LINE_STIPPLE);
			glLineWidth(linewidth);
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

			glDisable(GL_POLYGON_OFFSET_FILL);
		}

		if (m_currentRegion >= 0)
		{
			if (m_USE_DISPLAYED_AXES)
			{
				glEnable(GL_COLOR_MATERIAL);
				paintCoordinateFrame(m_cpoint3, axes_length, axes_radius, axis_is_selected);
			}
		}

		glPopMatrix();
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);


		//if in the process of selection, draw a dashed line with the lasso (2-d drawing)
		if (selecting)
		{
			if (mousePoints.size())
			{
				GLint viewport[4];
				glGetIntegerv(GL_VIEWPORT, viewport);

				glMatrixMode(GL_PROJECTION);
				glPushMatrix();
				glLoadIdentity();
				gluOrtho2D((GLdouble)viewport[0], (GLdouble)viewport[2], (GLdouble)viewport[1], (GLdouble)viewport[3]);
				glMatrixMode(GL_MODELVIEW);
				glPushMatrix();
				glLoadIdentity();

				glDisable(GL_DEPTH_TEST);
				glDisable(GL_CULL_FACE);
				glDisable(GL_BLEND);
				glDisable(GL_TEXTURE_2D);
				glDisable(GL_LIGHTING);
				glDisable(GL_LIGHT0);

				float linewidth;
				glGetFloatv(GL_LINE_WIDTH, &linewidth);
				glLineWidth(2.);
				glLineStipple(2, 0xAAAA);
				glEnable(GL_LINE_STIPPLE);
				glColor3d(0.5, 0.5, 0.5);
				glBegin(GL_LINE_STRIP);
				for (unsigned long i = 0; i <mousePoints.size(); ++i)
					glVertex2d(mousePoints[i].first, viewport[3] - mousePoints[i].second);
				glEnd();
				glDisable(GL_LINE_STIPPLE);
				glLineWidth(linewidth);

				//        glutSwapBuffers();

				glEnable(GL_DEPTH_TEST);

				glMatrixMode(GL_MODELVIEW);
				glPopMatrix();
				glMatrixMode(GL_PROJECTION);
				glPopMatrix();
				glMatrixMode(GL_MODELVIEW);
			}
		}
	}
	return;
#endif
}

void PickingPluginBase::translate_vertices(int region_index, float translation[3])
{
	///vertices in constrained_vertices have to be ORDERED !!!
	//int num_constrained_vertices = import_vertices->rows() - std::count(selection_id.begin(), selection_id.end(), -1);

	//m_constrained_vertex_positions = Eigen::MatrixXd(num_constrained_vertices, 3);
	//m_constrained_vertices.resize(num_constrained_vertices);

	RowVector3 t;
	t[0] = translation[0];
	t[1] = translation[1];
	t[2] = translation[2];
	int count = 0;
	for (long vi = 0; vi < import_vertices->rows(); ++vi)
		if (selection_id[vi]>=0)
		{
			if (selection_id[vi] == region_index)
			{
				assert(m_constrained_vertices[count] == vi);
				m_constrained_vertex_positions.row(count) += t;
			}			
			count++;
		}
}

void PickingPluginBase::get_constrained_positions(int r,
	std::vector<int> &constrained_vertices,
	Eigen::MatrixXd &constrained_positions,
	bool overwrite_original_positions)
{


#if 0
	///vertices in constrained_vertices have to be ORDERED !!!
	int num_constrained_vertices = import_vertices->rows() - std::count(selection_id.begin(), selection_id.end(), -1);
	constrained_positions = Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic>(num_constrained_vertices, 3);
	constrained_vertices.resize(num_constrained_vertices);

	RowVector3 centroid;
	computeRegionCentroid(previous_vertices, selection_id, r, centroid);

	int count = 0;
	for (long vi = 0; vi<import_vertices->rows(); ++vi)
		if (selection_id[vi] >= 0)
		{

			constrained_vertices[count] = vi;
			RowVector3 goalPosition = previous_vertices->row(vi);
			if (overwrite_original_positions && selection_id[vi] == r)
			{
				if (mouse_mode == TRANSLATE_HANDLE)
				{
					goalPosition[0] += translation[0];
					goalPosition[1] += translation[1];
					goalPosition[2] += translation[2];

				}
				else if (mouse_mode == ROTATE_HANDLE)
				{
					goalPosition -= centroid;
					float pt[3] = { goalPosition[0], goalPosition[1], goalPosition[2] };
					quat_rotate_point(rotation, pt, pt);
					goalPosition[0] = pt[0] + centroid[0];
					goalPosition[1] = pt[1] + centroid[1];
					goalPosition[2] = pt[2] + centroid[2];

				}
			}
			constrained_positions.row(count) = goalPosition;
			count++;
		}

#endif
}

void PickingPluginBase::constraint_changed()
{
	return;
}

void PickingPluginBase::selection_changed()
{
	return;


#ifdef ADD_LIM_DEFORM
	DeformLocallyInjective::GetReference().UpdatePositionalConstraints(m_constrained_vertices);
#endif
#ifdef ADD_SKINNING_DEFORM
	DeformSkinning::GetReference().UpdatePositionalConstraints(m_constrained_vertices, !CONNECT_PICKING_TO_SKINNING);
	HandlePlugin::GetReference().commit_temp_handles();
#endif
#ifdef ADD_VEGA_DEFORM
	DeformVega::GetReference().UpdatePositionalConstraints(m_constrained_vertices);
#endif
#ifdef ADD_PHYS_DEFORM
	//DeformPhysBase::GetReference().UpdatePositionalConstraints(m_constrained_vertices);
	if (true)//keep same in selection_changed and selected_vertices_moved
	{
		//DeformPhysBase::GetReference().UpdateConstraintVertexPositions(m_constrained_vertices, m_constrained_vertex_positions);
	}
#ifdef I_DO_NOT_REMEMBER_WHAT_THIS_IS_FOR
	else
	{
		int num_selected = 0;
		for (int i = 0; i<all_handle_list.size(); i++)
		{
			all_handle_list[i].selected = true;
			if (all_handle_list[i].selected)	num_selected++;
		}
		m_constrained_vertex_positions.resize(num_selected, 3);//assume dim is 3
		int k = 0;
		m_constrained_vertices.clear();
		for (int i = 0; i<all_handle_list.size(); i++)
		{
			bool is_constrained = all_handle_list[i].selected;
			if (is_constrained)
			{
				assert(all_handle_list[i].index >= 0);
				m_constrained_vertices.push_back(all_handle_list[i].index);
				m_constrained_vertex_positions(k, 0) = all_handle_list[i].x;
				m_constrained_vertex_positions(k, 1) = all_handle_list[i].y;
				m_constrained_vertex_positions(k, 2) = all_handle_list[i].z;
				k++;
			}
		}
		DeformPhysBase::GetReference().UpdateConstraintVertexPositions(m_constrained_vertices, m_constrained_vertex_positions);
	}
#endif
#endif
}

void PickingPluginBase::selected_vertices_moved()
{
	return;

#ifdef ADD_LIM_DEFORM
	DeformLocallyInjective::GetReference().UpdateConstraintVertexPositions(m_constrained_vertices, m_constrained_vertex_positions);
#endif
#ifdef ADD_SKINNING_DEFORM
	DeformSkinning::GetReference().UpdateConstraintVertexPositions(m_constrained_vertices, m_constrained_vertex_positions, !CONNECT_PICKING_TO_SKINNING);
#endif
#ifdef ADD_VEGA_DEFORM
	DeformVega::GetReference().UpdateConstraintVertexPositions(m_constrained_vertices, m_constrained_vertex_positions);
#endif
#ifdef ADD_PHYS_DEFORM
	if (true)//keep same in selection_changed and selected_vertices_moved
	{
		//DeformPhysBase::GetReference().UpdateConstraintVertexPositions(m_constrained_vertices, m_constrained_vertex_positions);
	}
	else
	{
#ifdef I_DO_NOT_REMEMBER_WHAT_THIS_IS_FOR
		int num_selected = 0;
		for (int i = 0; i<all_handle_list.size(); i++)
		{
			all_handle_list[i].selected = true;
			if (all_handle_list[i].selected)	num_selected++;
		}
		m_constrained_vertex_positions.resize(num_selected, 3);//assume dim is 3
		int k = 0;
		m_constrained_vertices.clear();
		for (int i = 0; i<all_handle_list.size(); i++)
		{
			bool is_constrained = all_handle_list[i].selected;
			if (is_constrained)
			{
				assert(all_handle_list[i].index >= 0);
				m_constrained_vertices.push_back(all_handle_list[i].index);
				m_constrained_vertex_positions(k, 0) = all_handle_list[i].x;
				m_constrained_vertex_positions(k, 1) = all_handle_list[i].y;
				m_constrained_vertex_positions(k, 2) = all_handle_list[i].z;
				k++;
			}
		}
		DeformPhysBase::GetReference().UpdateConstraintVertexPositions(m_constrained_vertices, m_constrained_vertex_positions);
#endif
	}
#endif
	/**** Assignment 5: Add your code for deformation here ****/

	// Simple deformation: the constrained vertices are moved to their target positions
	/*Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic> new_vertex_positions = *(import_vertices);
	for (unsigned long i = 0; i<m_constrained_vertices.size(); ++i)
	{
	int vid = m_constrained_vertices[i];
	new_vertex_positions.row(vid) = m_constrained_vertex_positions.row(i);
	}

	*(import_vertices) = new_vertex_positions;
	*/
	//m_preview->is_compiled = false;
}

void PickingPluginBase::init_Picking()
{//used for connecting deformers to skinning deformer

	selection_id.clear();
	selection_id.resize(import_vertices->rows(), -1);
	while (!m_free_regions.empty())
		m_free_regions.pop();
	for (int i = MAXNUMREGIONS; i >= 0; i--)
		m_free_regions.push(i);

	m_currentRegion = -1;
	mouse_mode = TRANSLATE_HANDLE;

	//if(IMPORT_HANDLES_IN_PICKING)//2014.10.6
	//{
	//	mouse_mode = TRANSLATE_HANDLE;//wangyu Set Default here//TRANSLATE_HANDLE;
	//}
	//else
	//{
	//	mouse_mode = HANDLE_TRANSLATE;//wangyu Set Default here//TRANSLATE_HANDLE;
	//}

	m_meshIsLoaded = import_vertices->rows() >0;

	if (m_meshIsLoaded)
	{
		adjacency_list(*(import_faces), (*import_vertex_to_vertices));
	}



	// compute random selection color
	// find hue for current material color
	double hsv[3], rgb[3];
	rgb[0] = m_preview->material.g_MatDiffuse[0];
	rgb[1] = m_preview->material.g_MatDiffuse[1];
	rgb[2] = m_preview->material.g_MatDiffuse[2];
	igl::rgb_to_hsv(rgb, hsv);

	double golden_ratio_conjugate = 0.618033988749895;
	double curHue = 0;
	for (int i = 0; i<MAXNUMREGIONS; i++)
	{
		double r, g, b;
		double hue = std::fmod(curHue, 1.0) * 360;

		// find color not similar to yellow
		while (std::abs(hue - hsv[0]) < 15)
		{
			curHue += golden_ratio_conjugate;
			hue = std::fmod(curHue, 1.0) * 360;
		}

		igl::hsv_to_rgb(hue, 0.8, 1.0, r, g, b);
		random_region_colors.row(i) << r, g, b;
		curHue += golden_ratio_conjugate;
	}

	random_colors = true;

	handle_color[0] = random_region_colors.coeff(0, 0);
	handle_color[1] = random_region_colors.coeff(0, 1);
	handle_color[2] = random_region_colors.coeff(0, 2);

}

//Key framing:

bool PickingPluginBase::loadSelectionFile(const char * fname, Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic>& S)
{
	std::string selFilename = std::string(fname);
	FILE *f = fopen(fname, "r");
	if (NULL == f)
	{
		printf("IOError: %s could not be opened for writing...", fname);
		return false;
	}

	//selection_id.assign(import_vertices->rows(), -1);
	long unsigned int num_constrained = 0;
	fscanf(f, "%lu", &num_constrained);
	//m_constrained_vertices.resize(num_constrained);
	//m_constrained_vertex_positions.resize(num_constrained,3);
	S.resize(num_constrained, 3);
	float x, y, z;
	for (unsigned long i = 0; i<num_constrained; i++)//m_constrained_vertices.size()
	{
		int sel_id, vid;
		fscanf(f, "%d %d %f %f %f\n",
			&vid,
			&sel_id,
			&x, &y, &z);
		//m_constrained_vertex_positions.row(i)<<x,y,z;
		S.row(i) << x, y, z;
		//m_constrained_vertices[i] = vid;
		//selection_id[vid] = sel_id;
	}

	fclose(f);

	return true;
}

void TW_CALL PickingPluginBase::load_dialog_keyframing_config(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_open_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<PickingPluginBase*>(clientData)->load_keyframing_config(fname);
}

void PickingPluginBase::set_turn_on_key_framing(bool t)
{
	turn_on_handle_key_frame = t;
	if (turn_on_handle_key_frame)
	{
		if (selectionKeyFrame.size()>0)
			key_frame_timer.Resume();
	}
	else
	{
		if (selectionKeyFrame.size()>0)
			key_frame_timer.Pause();
	}
}

bool PickingPluginBase::get_turn_on_key_framing() const
{
	return turn_on_handle_key_frame;
}

bool PickingPluginBase::load_keyframing_config(const char * config_file_name)
{
	FILE * config_file = fopen(config_file_name, "r");
	if (NULL == config_file)
	{
		fprintf(stderr, "IOError: %s could not be opened...\n", config_file_name);
		return false;
	}
	selectionKeyFrame.clear();

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
			Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic> keyframe_Selection;
			if (!loadSelectionFile(name, keyframe_Selection))
			{
				fprintf(stderr,
					"Error: load_keyframing_config() cannot open %s\n",
					name);
				fclose(config_file);
				return false;
			};
			printf("KeyFrame Selection Matrix:(%d,%d)\n", keyframe_Selection.rows(), keyframe_Selection.cols());
			assert(keyframe_Selection.cols() == 3);
			selectionKeyFrame.push_back(KeyFrame<SelectionFrame>(SelectionFrame(name, keyframe_Selection), duration, LINEAR_TRANSITION));
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

void PickingPluginBase::update_keyframing_selection()
{
	if (selectionKeyFrame.size() == 0)
	{
		return;
	}
	double time = key_frame_timer.Time();
	size_t a = 0;
	size_t b = 0;
	double f = 0;
	selectionKeyFrame.get_frame(time, a, b, f);
	//double ff=-2.0*f*f*f+3.0*f*f;
	double ff = f;
	//interpolate between two frames:
	m_constrained_vertex_positions = (1 - ff)*selectionKeyFrame[a].state.Selection + selectionKeyFrame[b].state.Selection;
	for (int i = 0; i<selectionKeyFrame[a].state.Selection.rows(); i++)
	{
		for (int d = 0; d<3; d++)
		{
			m_constrained_vertex_positions(i, d) = (1 - ff)*selectionKeyFrame[a].state.Selection(i, d) + ff*selectionKeyFrame[b].state.Selection(i, d);
		}
		//printf("f:%f v1:(%f,%f,%f) v2:(%f,%f,%f)\n",ff,
		//	selectionKeyFrame[a].state.Selection(i,0),
		//	selectionKeyFrame[a].state.Selection(i,1),
		//	selectionKeyFrame[a].state.Selection(i,2),
		//	selectionKeyFrame[b].state.Selection(i,0),
		//	selectionKeyFrame[b].state.Selection(i,1),
		//	selectionKeyFrame[b].state.Selection(i,2)
		//	);
	}
	selection_changed();
	selected_vertices_moved();//added by wangyu
							  //selected_vertices_moved();
}

void local_print_out_argv(const std::vector<std::string> &cl)
{
	for (size_t i = 0; i < cl.size(); i++)
	{
		printf("  %s", cl[i]);
	}
}

#include <viewer/CommandLineBase.h>
bool CommandLine(PickingPluginBase& plugin, std::vector<std::string> cl)
{
	if (cl.size() < 1)
	{
		printf("Error: No Command Line for PickingPlugin.\n");
		return false;
	}
	else
	{
		printf("PickingPlugin");
		local_print_out_argv(cl); // todo, replace this with print_out_argv in CommandLineBase.h later
		printf("\n");
	}

	if (cl[0] == std::string("load_selection"))
	{
		if (cl.size() < 2)
		{
			printf("Error: No mesh file name for load_selection.\n");
			return false;
		}
		return plugin.loadSelection(cl[1].c_str());
	}
	else
	{
		printf("Error: Unknown Command Line Type for Viewer.\n");
		return false;
	}
}

bool PickingPluginBase::commandLine(std::string c, std::vector<std::string> cl)
{
	if (c == std::string("Pick&Drag"))
	{
		return CommandLine(*this, cl);
	}
	return false;
}