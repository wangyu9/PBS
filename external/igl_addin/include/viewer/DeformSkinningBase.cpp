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

#include "PluginManager.h"

//#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include <unsupported/Eigen/OpenGLSupport>

#include "DeformSkinningBase.h"

//#include "PickingPlugin.h"

#include "FileDialog.h"

#include "path_anttweak.h"

#include <sort_weights.h>

#ifdef USING_IGL_HEADER_ONLY_MODE
#define IGL_HEADER_ONLY 
#endif

#define SUPPORT_HDF5_MAT

//#include "igl/boundary_faces.h"
#include <igl/boundary_facets.h>

#ifdef USE_MATLAB_ENGINE
#include <igl/matlab/matlabinterface.h>
#endif

#include <Eigen/Core>
#include <igl/readDMAT.h>
#include <igl/writeDMAT.h>
#include <cstdio>
#ifdef USE_MATLAB_ENGINE
#include <igl/matlab/MatlabWorkspace.h>
#endif
#include <igl/on_boundary.h>

#include "igl/readOBJ.h"
#include "igl/readMESH.h"
#include "igl/adjacency_list.h"

#include <igl/lbs_matrix.h>

//#include <FAST/skinning/lbs_matrix.h>
//#include <FAST/skinning/region_colors_from_weights.h>
//#include <FAST/skinning/sort_weights.h>

// IGL ADDIN:
#include <reproject_coordinate.h>
#include <lbs.h>
#include <extension_from_name.h>
#include <readH5MAT.h>
#include <writeH5MAT.h>
#ifdef USE_MATLAB_ENGINE
#include <MATLAB/matlab.h>
#endif
#include "utils/TimerWrapper.h"
//#include <FAST/skinning.h>
//#include <skinning_interface.h>
//#include <igl/boundary_conditions.h>
//#include "igl/mosek/bbw.h"
#include <normal_deform_factors.h>
#include "math_helper.h"
//#define ENABLE_PICKING_PLUGIN



using namespace std;
using namespace Eigen;


WeightsType str2_weights_type(const std::string wt)
{
	if (wt == "BilaplacianCoordinate"){
		return BilaplacianCoordinate;
	} 
	else if (wt == "ThinPlateSpline")
	{
		return ThinPlateSpline;
	}
	else if (wt == "StandardBilaplacian")
	{
		return StandardBilaplacian;
	}
	else if (wt == "LeastSquareMesh")
	{
		return LeastSquareMesh;
	}
	else if (wt == "BoundedBiharmonicWeights")
	{
		return BoundedBiharmonicWeights;
	}
	else if (wt == "BiharmonicWeights")
	{
		return BiharmonicWeights;
	}
	else if (wt == "FacetBiharmonicWeights")
	{
		return FacetBiharmonicWeights;
	}
	else if (wt == "ConstraintBilaplacianCoordinate")
	{
		return ConstraintBilaplacianCoordinate;
	}
	else if (wt == "TrilaplacianCoordinate")
	{
		return TrilaplacianCoordinate;
	}
	else if (wt == "TestSlot2")
	{
		return TestSlot2;
	}
	else if (wt == "TestSlot3")
	{
		return TestSlot3;
	}
	//else if (wt == "")
	//{
	//	return;
	//} 
	else
	{
		printf("Error: no matched weights type, set default one.");
		return BilaplacianCoordinate;
	}
}

SkinningType str2_skinning_type(const std::string st)
{
	if (st=="LBS")
	{
		return LBS;
	}
	else if (st == "DQLBS")
	{
		return DQLBS;
	}
	else if (st == "PBS")
	{
		return PBS;
	}
	else if (st == "HS")
	{
		return HS;
	}
	else if (st == "None")
	{
		return None;
	}
	//else if (st == )
	//{
	//	return;
	//}
	else
	{
		printf("Error: no matched skinning type, set default one.");
		return PBS;
	}
}

bool DeformSkinningBase::set_skinning_type(const std::string stn)
{
	SkinningType st = str2_skinning_type(stn);
	SetSkinning(st);
	return true;
}

bool DeformSkinningBase::set_weights_type(const std::string wtn)
{
	WeightsType wt = str2_weights_type(wtn);
	SetWeightsType(wt);
	return true;
}

void extract_2d_m_from_3d(const Eigen::MatrixXd& M0, Eigen::MatrixXd& M)
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

/************* Commuication Functions *************/

void DeformSkinningBase::SendVarToDeformPhys(const int dim, Eigen::MatrixXd& Var)
{
	Var = GetVar(dim);

	return;

	//if (dim==2&&skinningType==LBS)
	//{
	//	assert(varibles->rows()%4==0);
	//	int num = varibles->rows()/4;

	//	Var.resize(3*num,2);
	//	for (int i=0; i<num; i++)
	//	{
	//		Var(3*i+0,0) = varibles->coeff(4*i+0,0);
	//		Var(3*i+0,1) = varibles->coeff(4*i+0,1);
	//		Var(3*i+1,0) = varibles->coeff(4*i+1,0);
	//		Var(3*i+1,1) = varibles->coeff(4*i+1,1);

	//		Var(3*i+2,0) = varibles->coeff(4*i+3,0);
	//		Var(3*i+2,1) = varibles->coeff(4*i+3,1);
	//	}
	//} 
	//else
	//{
	//	Var = (*varibles).leftCols(dim);//P.leftCols(cols) is P(:, 1:cols) in MATLAB
	//}
}

void DeformSkinningBase::SendBasisToDeformPhys(const int dim, Eigen::MatrixXd& ARAP_M)
{
	switch (skinningType){
	case LBS:
		ARAP_M = (dim == 3) ? M3d_lbs : M2d_lbs;
		break;
	case PBS:
		ARAP_M = Weights;
		break;
	case HS:
		ARAP_M = (dim == 3) ? M3d_hs : M2d_hs;
	}

}

/********* End of Commuication Functions **********/

DeformSkinningBaseUI::DeformSkinningBaseUI()
{
	//check in with the manager
	PluginManager().register_plugin(this);
	bar = NULL;
}

DeformSkinningBaseUI::~DeformSkinningBaseUI()
{


	DeformSkinningBase::~DeformSkinningBase();
}

void DeformSkinningBaseUI::init(Preview3D *preview)
{
	PreviewPlugin::init(preview);

	// init menu bar
	if (bar == NULL)
	{
		// Create a tweak bar
		bar = new igl::ReTwBar;
		bar->TwNewBar("SkinningDeformation");
		TwDefine(" SkinningDeformation size='250 500' color='76 76 127' position='500 300' label='Skinning Deformation' "); // change default tweak bar size and color
		bar->TwAddVarRW("Enable Deformer", TW_TYPE_BOOLCPP, &enable_deform_skinning, "");
		bar->TwAddVarRO("#Elements", TW_TYPE_INT32, &numElements, "");

#define SkinningCount 5
		TwEnumVal skinningEV[SkinningCount] = { { LBS, "LBS" }, { DQLBS, "DQLBS" }, { PBS, "PBS" }, { HS, "HS" }, { None, "None" } };
		TwType skinningT = TwDefineEnum("Skinning", skinningEV, SkinningCount);
		bar->TwAddVarCB("Skinning", skinningT, SetSkinningCB, GetSkinningCB, this, "");
		//#define EnergyCount 8 
		//TwEnumVal energyEV[EnergyCount] = {{IDENTITY, "Identity"}, {DIRICHLET, "Dirichlet"}, {UNILAP, "Uniform Laplacian"}, {COTLAP, "Cotan Laplacian"}, {GREEN, "Green Strain"}, {ARAP, "ARAP"}, {LSC, "LSC (2D only)"}, {POISSON, "Poisson (2D only)"}};
		//TwType energy = TwDefineEnum("Energy", energyEV, EnergyCount);
		//bar->TwAddVarCB("Energy", energy, SetEnergyCB, GetEnergyCB, this, "");
		bar->TwAddVarRW("Show Invertions", TW_TYPE_BOOLCPP, &showInvertedElements, " group='Draw' label='Show Invertions'");
		bar->TwAddVarRW("Draw Tangents", TW_TYPE_BOOLCPP, &drawVertexTangents, " group='Draw'");
		bar->TwAddVarRW("ApplySkinning", TW_TYPE_BOOLCPP, &runSolver, "label='Apply Skinning'");
		//bar->TwAddVarRW("Barriers", TW_TYPE_BOOLCPP, &enableBarriers, "group='Solver Options'");
		//bar->TwAddVarRW("BarrierWeights", TW_TYPE_DOUBLE, &barrierWeight, "group='Solver Options'");
		//bar->TwAddVarRW("SubStepping", TW_TYPE_BOOLCPP, &enableSubStepping, "group='Solver Options'");
		//bar->TwAddVarRW("AlphaUpdate", TW_TYPE_BOOLCPP, &enableAlphaUpdate, "group='Solver Options'");
		//bar->TwAddVarRW("Alpha/Ratio", TW_TYPE_DOUBLE, &alpha, "group='Solver Options'");
		//bar->TwAddVarRW("Output", TW_TYPE_BOOLCPP, &enableOutput, "group='Solver Options'");

		//bar->TwAddVarRO("PCError", TW_TYPE_DOUBLE, &error, "group='Solver Options'");


		//Mesh
		bar->TwAddVarRW(" Tetrahedralize Mesh", TW_TYPE_BOOLCPP, &wantTetMesh,
			" group='Mesh'"
			" label='Tetrahedralize Mesh'");
		bar->TwAddVarRW(" Recompute Normal when Updated", TW_TYPE_BOOLCPP, &recompute_normal_updated,
			" group='Mesh'"
			" label='Recompute Normal'");

		//Weights
		bar->TwAddButton("Load Weights", open_dialog_weights, this,
			" group='Weights'"
			" label='Load Weights' help='Load weights.'");

		bar->TwAddButton("Load Sparse Weights and Indices", open_dialog_sparse_weights, this,
			" group='Weights'"
			" label='Load Sparse Weights'");

#define WeightsTypeCount 11
		TwEnumVal weightsTypeEV[WeightsTypeCount] = { { BilaplacianCoordinate, "BilaplacianCoordinate" },
		{ ThinPlateSpline, "ThinPlateSpline" },
		{ StandardBilaplacian, "StandardBilaplacian" },
		{ LeastSquareMesh, "LeastSquareMesh" },
		{ BoundedBiharmonicWeights, "BoundedBiharmonicWeights" },
		{ BiharmonicWeights, "BiharmonicWeights" },
		{ FacetBiharmonicWeights, "FacetBiharmonicWeights" },
		{ ConstraintBilaplacianCoordinate, "ConstraintBilaplacianCoordinate" },
		{ TrilaplacianCoordinate, "TrilaplacianCoordinate" },
		{ TestSlot2, "TestSlot2" },
		{ TestSlot3, "TestSlot3" }
		};
		TwType weightsTypeT = TwDefineEnum("Weights Type", weightsTypeEV, WeightsTypeCount);
		bar->TwAddVarCB("Weights Type", weightsTypeT, SetWeightsTypeCB, GetWeightsTypeCB, this, " group='Weights'");


		bar->TwAddVarRW("Current Weights", TW_TYPE_STDSTRING, &currentWeightsFileName, " group='Weights'");
		//bar->TwAddButton("Draw Weights", draw_dialog_weights, this,
		//	" group='Weights'"
		//	" label='Draw Weights'");
		bar->TwAddVarRW("Visualize", TW_TYPE_BOOLCPP, &enable_draw_weights_on_mesh, "label='Visualize' group='Weights'");

		bar->TwAddButton("Save Weights", save_dialog_weights, this,
			" group='Weights'"
			" label='Save Weights'");

		bar->TwAddButton("Reproject Sparse Weights", ReprojectSparseWeights, this, " group='Weights'");
		bar->TwAddButton("Reproject Deform Factors", ReprojectDeformFactors, this, " group='Weights'");

		bar->TwAddVarRW("Sparse Weights Slots to Use", TW_TYPE_BOOLCPP, &num_sparse_weight_slots, " group='Weights'");
		bar->TwAddVarRW("Sparse Factors Slots to Use", TW_TYPE_BOOLCPP, &num_sparse_factor_slots, " group='Weights'");

		bar->TwAddButton("Load Deform Factors", open_dialog_deform_factors, this,
			" group='Deform Factor'");
		bar->TwAddButton("Save Deform Factors", save_dialog_deform_factors, this,
			" group='Deform Factor'");

		/******************To Rendering*******************/
#define NormalUpdateTypeCount 3
		TwEnumVal normalUpdateTypeEV[NormalUpdateTypeCount] = { { NaiveNormalUpdate, "NaiveNormalUpdate" }, { SkinningNormalUpdate, "SkinningNormalUpdate" }, { NoNormalUpdate, "NoNormalUpdate" } };
		TwType normalUpdateTypeT = TwDefineEnum("Normal Update Type", normalUpdateTypeEV, NormalUpdateTypeCount);

		bar->TwAddVarCB("Normal Update Type", normalUpdateTypeT, SetNormalUpdateTypeCB, GetNormalUpdateTypeCB, this, "group='To Rendering'");
		bar->TwAddVarRW("Full Weights for Skinning", TW_TYPE_BOOLCPP, &use_full_weights, "group='To Rendering'");
		bar->TwAddVarRW("Full Factors for Normal Updating", TW_TYPE_BOOLCPP, &use_full_factors, "group='To Rendering'");

	}


	DeformSkinningBase::init(preview);
}

bool DeformSkinningBaseUI::Serialize(tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element)
{
	return igl::save_ReAntTweakBar(bar, doc);
}

bool DeformSkinningBaseUI::Deserialize(tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element)
{
	return igl::load_ReAntTweakBar(bar, doc);
}

// keyboard callback
//LIMData* data_skinning = NULL;
bool DeformSkinningBaseUI::keyDownEvent(unsigned char key, int modifiers, int mouse_x, int mouse_y)
{
	if (!enable_deform_skinning)
		return false;

	switch (key)
	{

	case '<':
	{
		if (weightsFrames.size()>0)
		{
			current_weights_frame_index = (current_weights_frame_index + 1) % weightsFrames.size();
			currentWeightsFileName = weightsFrames[current_weights_frame_index].name;
			Weights = weightsFrames[current_weights_frame_index].Weights;
			//applySkinning();
			update_skinning = true;
		}
		return true;
	}
	break;

	case '>':
	{
		if (weightsFrames.size()>0)
		{
			current_weights_frame_index = (current_weights_frame_index - 1) % weightsFrames.size();
			currentWeightsFileName = weightsFrames[current_weights_frame_index].name;
			Weights = weightsFrames[current_weights_frame_index].Weights;
			//applySkinning();
			update_skinning = true;
		}
		return true;
	}
	break;

	}

	return false;
}

//mouse callback
bool DeformSkinningBaseUI::mouseDownEvent(int mouse_x, int mouse_y, int button, int modifiers)
{
	return false;
}

bool DeformSkinningBaseUI::mouseUpEvent(int mouse_x, int mouse_y, int button, int modifiers)
{
	return false;
}

bool DeformSkinningBaseUI::mouseMoveEvent(int mouse_x, int mouse_y)
{


	return false;
}

bool DeformSkinningBaseUI::mouseScrollEvent(int mouse_x, int mouse_y, float delta)
{
	return false;
}

//stuff that is drawn by the plugin before the previewer has displayed the mesh
//first draw 3d, then 2d
void DeformSkinningBaseUI::preDraw(int currentTime)
{
	if (!enable_deform_skinning)
		return;


	if (isMeshLoaded && runSolver)
	{
		if (update_skinning)
		{

#ifdef DEBUG_PLUGINS_SEQUENTIAL_ORDER
			printf("DEBUG_PLUGINS_SEQUENTIAL_ORDER: DeformSkinningBase ApplySkinning\n");
#endif

			applySkinning();
			copy_skinned_mesh_back();
		}

#define COMPILE_MESH_EVERY_TIME
#ifndef COMPILE_MESH_EVERY_TIME
		m_preview->is_compiled = false;//VERY SLOW for large mesh, this will cause the mesh to be compiled in Viewer.cpp line 1414 
#endif
	}
}

//stuff that is drawn by the plugin after the previewer has displayed the mesh
//first draw 3d, then 2d
void DeformSkinningBaseUI::postDraw(int currentTime)
{
	if (!enable_deform_skinning)
		return;

	if (isMeshLoaded)
	{
		if (showInvertedElements)
		{

		}
		if (drawVertexTangents)//draw tangent plane: tangents and binormals
		{
			draw_tangents();
		}
	}

}


void TW_CALL DeformSkinningBaseUI::open_dialog_weights(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_open_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<DeformSkinningBaseUI *>(clientData)->load_weights_from_file(fname);

}

void TW_CALL DeformSkinningBaseUI::open_dialog_sparse_weights(void *clientData)
{
	char weights_fname[2048];

	weights_fname[0] = 0;
	get_open_file_path(weights_fname);

	if (weights_fname[0] == 0)
		return;

	char indices_fname[2048];

	indices_fname[0] = 0;
	get_open_file_path(indices_fname);

	if (indices_fname[0] == 0)
		return;

	static_cast<DeformSkinningBaseUI *>(clientData)->load_sparse_weights_from_file(weights_fname, indices_fname);

}

void TW_CALL DeformSkinningBaseUI::open_dialog_deform_factors(void *clientData)
{
	char fname1[2048];

	fname1[0] = 0;
	get_open_file_path(fname1);

	if (fname1[0] == 0)
		return;

	char fname2[2048];

	fname2[0] = 0;
	get_open_file_path(fname2);

	if (fname2[0] == 0)
		return;

	static_cast<DeformSkinningBaseUI *>(clientData)->load_deform_factors_from_file(fname1, fname2);

}

void TW_CALL DeformSkinningBaseUI::save_dialog_deform_factors(void *clientData)
{
	char fname1[2048];

	fname1[0] = 0;
	get_save_file_path(fname1);

	if (fname1[0] == 0)
		return;

	char fname2[2048];

	fname2[0] = 0;
	get_save_file_path(fname2);

	if (fname2[0] == 0)
		return;

	static_cast<DeformSkinningBaseUI *>(clientData)->save_deform_factors_to_file(fname1, fname2);

}

void TW_CALL DeformSkinningBaseUI::save_dialog_weights(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_save_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<DeformSkinningBaseUI *>(clientData)->save_weights_to_file(fname);
}

//void TW_CALL DeformSkinningBaseUI::draw_dialog_weights(void *clientData)
//{
//	static_cast<DeformSkinningBaseUI *>(clientData)->draw_weights_to_mesh();
//}



DeformSkinningBase::DeformSkinningBase()
	:enable_deform_skinning(true),
	hasLoadHandles(false), hasLoadWeights(false)
{
	isMeshLoaded = false;
	showInvertedElements = false;
	drawVertexTangents = false;
	runSolver = true;
	isTetMesh = false;

	recompute_normal_updated = false;

	wantTetMesh = false;
	skinningType = HS;
	enable_draw_weights_on_mesh = false;
	workSpaceClearedBeforeUse = false;
	weightsType = BilaplacianCoordinate;

	use_full_weights = true;
	use_full_factors = false;

	M3d_lbs.resize(0, 0);
	M2d_lbs.resize(0, 0);
	M3d_hs.resize(0, 0);
	M2d_hs.resize(0, 0);
	M3d_hs_2nd.resize(0, 0);
	M2d_hs_2nd.resize(0, 0);

	Weights.resize(0, 0);
	W.resize(0, 0);
	WI.resize(0, 0);


	alphaFactors.resize(0, 0);
	betaFactors.resize(0, 0);

	t_dir.resize(0, 3);
	b_dir.resize(0, 3);
	tang_dir.resize(0, 3);
	binormal_dir.resize(0, 3);

	sorted_alphaFactors.resize(0, 0);
	sorted_betaFactors.resize(0, 0);
#ifdef USE_MATLAB_ENGINE
	matlabEngine = new (Engine*);
	igl::matlab::mlinit(matlabEngine);
#endif
	currentWeightsFileName = "";
	current_weights_frame_index = 0;

	update_skinning = false;

	num_sparse_weight_slots = 16;
	num_sparse_factor_slots = 16;

	numElements = 0;
};

DeformSkinningBase::~DeformSkinningBase()
{
#ifdef USE_MATLAB_ENGINE
	using namespace igl::matlab;
	if (*matlabEngine != NULL)
	{
		mlclose(matlabEngine);
		delete *matlabEngine;
	}
#endif
};

// initialization (runs every time a mesh is loaded or cleared)
void DeformSkinningBase::init(Preview3D *preview)
{
	m_preview3d = preview;

	pV = m_preview3d->GetMainMesh().vertices;
	pF = m_preview3d->GetMainMesh().faces;

	isMeshLoaded = pV->rows() > 0;

	workSpaceClearedBeforeUse = false;


	if (!enable_deform_skinning)
		return;



	//bool wantTetMesh = true;//set whether do we want tet mesh
	if (isMeshLoaded)
	{
		if (wantTetMesh)
		{
			isTetMesh = true;

			//m_preview3d->camera.set_toggle_ortho(false, m_preview3d->width, m_preview3d->height);
			//m_preview3d->camera.enable_rotation = true;
		}
		else
		{
			isTetMesh = false;

			//m_preview3d->camera.set_toggle_ortho(true, m_preview3d->width, m_preview3d->height);
			//m_preview3d->camera.view_xy_plane();
			//if (wantTetMesh)
			//{
			//	m_preview3d->camera.enable_rotation = false;
			//}
			//else
			//{
			//	m_preview3d->camera.enable_rotation = true;//wangyu: we only want to see the surface of 3D model
			//}
			//createTriMesh(true);
		}

		int dim = isTetMesh ? 3 : 2;

		//mesh->ConstraintMatrix->resize(mesh->InitalVertices->rows()*dim,mesh->InitalVertices->rows()*dim);
		//mesh->ConstraintTargets->resize(mesh->InitalVertices->rows()*dim);
		//mesh->ConstraintTargets->setZero();

		//// init with identity matrix in order to reserve single vertex constraints
		//vector<Eigen::Triplet<double> > triplets;
		//for(int i=0;i<mesh->InitalVertices->rows()*dim;i++)
		//	triplets.push_back(Triplet<double>(i,i,1));
		//mesh->ConstraintMatrix->setFromTriplets(triplets.begin(),triplets.end());

		////initEnergy();

		//// free all constraint vertices as now hessian structure is already reserved
		//for(int i=0;i<mesh->InitalVertices->rows()*dim;i++)
		//	mesh->ConstraintMatrix->coeffRef(i,i) = 0;

		////solver->UpdatePositionalConstraintMatrix();
	}
}

//CALLED when moved
void DeformSkinningBase::UpdateConstraintVertexPositions(const std::vector<IndexType>& constraintVertices, const Eigen::MatrixXd& positions, bool update_skinning)
{
	if (!enable_deform_skinning)
		return;

	int dim = isTetMesh ? 3 : 2;

	int N = pV->rows();

	//HandlePlugin::GetReference().commit_temp_handles_only();

	//solver->Restart();

	if (update_skinning && hasLoadHandles && hasLoadWeights && skinningType != None)
	{
		//applySkinning();
		update_skinning = true;
	}
	if (skinningType == None)
	{
		for (int i = 0; i<constraintVertices.size(); i++)
		{
			int idx = constraintVertices[i];
			for (int d = 0; d<dim; d++)
			{
				//mesh->ConstraintTargets->coeffRef(idx*dim+d) = positions.coeff(i,d);
				if (1)
				{
					//printf("%d %f %f\n",d, pV->coeffRef(idx+dim*d), positions.coeff(i,d));
					pV->coeffRef(idx + N*d) = positions.coeff(i, d);
					//mesh->
				}
				//if(d==1)
				//	pV->coeffRef(idx+dim*d) = positions.coeff(i,2);
				//mesh->PredictedVertices->coeffRef(idx*dim+d) = positions.coeff(i,d);
			}
		}
	}
}

void DeformSkinningBase::applyPointBasedSkinning(Eigen::Matrix<double, Eigen::Dynamic, 3>* verticesMatrix)
{//assume verticesMatrix and Weights and Handles are all column major
	assert(verticesMatrix->rows() == Weights.rows());
	assert(Weights.cols() == Handles().rows());
	assert(verticesMatrix->cols() == Handles().cols());
	for (int i = 0; i<Weights.rows(); i++)
	{
		for (int j = 0; j<Handles().cols(); j++)
		{
			double c_ij = 0;
			for (int k = 0; k<Weights.cols(); k++)
			{
				c_ij += Weights(i, k)*Handles()(k, j);
				//if(COL_MAJOR_HANDLES)
				//{
				//	c_ij += Weights.coeffRef(i+k*Weights.rows())*Handles.coeffRef(k+j*Handles.rows());
				//}
				//else
				//{
				//	//c_ij += Weights.coeffRef(i+k*Weights.rows())*Handles.coeffRef(k+j*Handles.rows());
				//}
			}
			verticesMatrix->coeffRef(i + j*verticesMatrix->rows()) = c_ij;
		}
	}
}

//#include <print_matlab.h>
void DeformSkinningBase::set_M_lbs_matrix(int dim)
{
	// Set M matrix
	igl::lbs_matrix(m_preview3d->GetMainMesh().rest_vertices.cast<double>(), Weights, M3d_lbs);
	igl::lbs_matrix(m_preview3d->GetMainMesh().rest_vertices.cast<double>().leftCols<2>(), Weights, M2d_lbs);
	//print_matlab(M,std::string("M_from_vs") );
}

void DeformSkinningBase::applyLinearBlendSkinning(Eigen::MatrixXd* verticesMatrix)
{
	//if(M3d_lbs.cols()!=HandleTrans(3).rows())
	//{
	//	printf("Weights or HandlesTrans Matrix Dimension Incorrect!\n");
	//	return;
	//}
	//printf("Apply Linear Blend Skinning: M(%d,%d)*T(%d,%d)\n",M3d_lbs.rows(),M3d_lbs.cols(),HandleTrans(3).rows(),HandleTrans(3).cols());

	if (M3d_lbs.cols() != HandleVars(3).rows())
	{
		printf("Weights or HandleVars Matrix Dimension Incorrect!\n");
		return;
	}
	printf("Apply Linear Blend Skinning: M(%d,%d)*T(%d,%d)\n", M3d_lbs.rows(), M3d_lbs.cols(), HandleVars(3).rows(), HandleVars(3).cols());


	(*verticesMatrix) = M3d_lbs*HandleVars(3);
}

void DeformSkinningBase::applyHybridSkinning(Eigen::MatrixXd* verticesMatrix)
{
	if (M3d_hs.cols() != HandleVars(3).rows())
	{
		printf("Weights or HandleVars Matrix Dimension Incorrect!\n");
		return;
	}

	//printf("Apply Hybrid Skinning: M(%d,%d)*T(%d,%d)\n",M3d_hs.rows(),M3d_hs.cols(),HandleVars(3).rows(),HandleVars(3).cols());

	(*verticesMatrix) = M3d_hs*HandleVars(3);

	//print_matlab(M3d_hs,"M3d_hs");
	//print_matlab(HandleVars(3),"HandleVars");
}

void DeformSkinningBase::applySkinning()
{//only call this function when weights and handels has been load

	//Eigen::SparseMatrix
	// m_storage.m_rows;

	//for PBS
	//*mesh->DeformedVertices = W *(*mesh->DeformedVertices);
	//apply skinning here:
	if (hasLoadWeights && hasLoadHandles)
	{
		int dim = isTetMesh ? 3 : 2;
		int N = pV->rows();


		//TimerWrapper timerWrapper;
		//timerWrapper.Tic();
		switch (skinningType){
		case LBS:
			//set_M_lbs_matrix();
			applyLinearBlendSkinning(pV);
			break;
		case HS:
			applyHybridSkinning(pV);
			break;
		case DQLBS:
			break;
		case PBS:
		{
			if (use_full_weights)
			{
				if (Weights.cols() != Handles().rows())
				{
					printf("Weights or Handles Matrix Dimension Incorrect!\n");
					return;
				}

				// Full Weights Skinning
				*pV = Weights * Handles();
			}
			else
			{
				// Sparse Weights Skinning
				for (int i = 0; i<WI.rows(); i++)
				{
					int dim = 3;
					for (int c = 0; c<dim; c++)
					{
						pV->coeffRef(i, c) = 0;
						for (int j = 0; j<WI.cols(); j++)
						{
							int k = WI(i, j);
							pV->coeffRef(i, c) +=
								(float)W(i, j) * (float)Handles()(k, c);
						}
					}
				}
			}
		}
		break;
		case None:
			break;
		}
		//timerWrapper.Toc();
		//printf("Skinning time: %f sec.\n",timerWrapper.Duration());


	}

	// update shading
	switch (normalUpdateType)
	{
	case NaiveNormalUpdate:
	{
		m_preview3d->GetMainMesh().normals_changed = true;
	}
	break;
	case SkinningNormalUpdate:
	{
		switch (skinningType){
		case LBS:
			break;
		case DQLBS:
			break;
		case HS:
			break;
		case PBS:
		{
			Eigen::MatrixXd* pVertex_normals = m_preview3d->GetMainMesh().vertex_normals;
			Eigen::MatrixXd H = Handles();
			Eigen::VectorXi HI = HandleIndices();
			Eigen::MatrixXd tangents(H.rows(), 3);
			Eigen::MatrixXd binormals(H.rows(), 3);
			for (int i = 0; i<H.rows(); i++)
			{
				int index = HI(i);//HandlePlugin::GetReference().HandleIndices(i);
				if (index >= t_dir.rows() || index<0)
				{
					printf("Invalid handle index value of %d, when we should have 0<=index<%d is \n", index, t_dir);
				}
				tangents.row(i) = t_dir.row(index);
				binormals.row(i) = b_dir.row(index);
			}

			//Eigen::MatrixXd tang_dir;
			//Eigen::MatrixXd binormal_dir;
			if (use_full_factors)
			{
				// Full Factor
				Eigen::MatrixXd H = Handles();
				tang_dir = alphaFactors * H;// + Weights * tangents;//+ t_dir.transpose();// 
				binormal_dir = betaFactors * H;// + Weights * binormals;//+ b_dir.transpose();// 
			}
			else
			{
				// Sparse Factor
				tang_dir.resize(WI.rows(), Handles().cols());
				binormal_dir.resize(WI.rows(), Handles().cols());
				for (int i = 0; i<WI.rows(); i++)
				{
					int dim = 3;
					assert(dim == Handles().cols());
					for (int c = 0; c<dim; c++)
					{
						tang_dir(i, c) = 0;
						binormal_dir(i, c) = 0;
						for (int j = 0; j<sorted_alphaFactors.cols(); j++)
						{
							int k = WI(i, j);
							tang_dir(i, c) +=
								(double)sorted_alphaFactors(i, j) * (double)Handles()(k, c);
							//+ (float)W(i,j) * (float)tangents(k,c);
							binormal_dir(i, c) +=
								(double)sorted_betaFactors(i, j) * (double)Handles()(k, c);
							//+ (float)W(i,j) * (float)binormals(k,c);
						}
						//for (int j=0; j<alphaFactors.cols(); j++)
						//{
						//	int k = j;
						//	tang_dir(i,c) += 
						//		(double)alphaFactors(i,j) * (double)Handles()(k,c);
						//	//+ (float)W(i,j) * (float)tangents(k,c);
						//	binormal_dir(i,c) +=
						//		(double)betaFactors(i,j) * (double)Handles()(k,c);
						//	//+ (float)W(i,j) * (float)binormals(k,c);
						//}

						//tang_dir(i,c) += (float)t_dir(c,i);
						//binormal_dir(i,c) += (float) b_dir(c,i);
					}
				}
			}
			for (int i = 0; i<pVertex_normals->rows(); i++)
			{
				Eigen::Vector3d t = tang_dir.row(i).transpose();
				Eigen::Vector3d b = binormal_dir.row(i).transpose();
				Eigen::Vector3d new_normal = (t.cross(b)).normalized();
				for (int c = 0; c<3; c++)
				{
					pVertex_normals->coeffRef(i, c) = new_normal(c);
				}
				//double prod = pVertex_normals->coeffRef(i,0) * new_normal(0)
				//			+ pVertex_normals->coeffRef(i,1) * new_normal(1)
				//			+ pVertex_normals->coeffRef(i,2) * new_normal(2);
				//if (prod<0.9)
				//{
				//	printf("Incorrect normal(%d) detected! with inner prod %f.\n", i, prod);
				//}
				//if (prod>0)
				//{
				//	for (int c=0; c<3; c++)
				//	{
				//		pVertex_normals->coeffRef(i,c) = new_normal(c);
				//	}	
				//} 
				//else
				//{
				//	for (int c=0; c<3; c++)
				//	{
				//		pVertex_normals->coeffRef(i,c) = -new_normal(c);
				//	}	
				//}

			}

		}
		break;
		case None:
			break;
		}
	}
	break;
	case NoNormalUpdate:
	{

	}
	break;
	}
}

//CALLED when selection changed
void DeformSkinningBase::UpdatePositionalConstraints(const std::vector<IndexType>& constraintVertices, bool update_skinning)
{
	if (!enable_deform_skinning)
		return;

	int dim = isTetMesh ? 3 : 2;

	//HandlePlugin::GetReference().commit_temp_handles_only();

	//// free all constraint vertices
	//for(int i=0;i<mesh->InitalVertices->rows()*dim;i++)
	//  mesh->ConstraintMatrix->coeffRef(i,i) = 0;
	//mesh->ConstraintTargets->setZero();

	//// set new constraint vertices
	//for(int i=0;i<constraintVertices.size();i++)
	//{
	//  int idx = constraintVertices[i];
	//  for(int c=0;c<dim;c++)
	//  {
	//    mesh->ConstraintMatrix->coeffRef(idx*dim+c,idx*dim+c) = 1;
	//    mesh->ConstraintTargets->coeffRef(idx*dim+c) = pV->coeff(idx,c);
	//  }
	//}

	//solver->UpdatePositionalConstraintMatrix();

	//solver->Restart();

	if (update_skinning && hasLoadHandles && hasLoadWeights)
	{
		//applySkinning();
		update_skinning = true;
	}
}

bool DeformSkinningBase::set_weights(const Eigen::MatrixXd& newW, const std::string name)
{
	Weights = newW;

	// push weight frame
	std::string weights_file_path_name(name);
	size_t last_dir_marker = weights_file_path_name.rfind('\\');
	currentWeightsFileName = weights_file_path_name.substr(last_dir_marker + 1);
	weightsFrames.push_back(WeightsFrame(currentWeightsFileName, Weights));
	current_weights_frame_index = weightsFrames.size() - 1;

	do_this_when_weights_loaded();

	return true;
}

bool DeformSkinningBase::load_weights_from_file(const char* weights_file_name)
{
	std::string extension = extension_from_name(weights_file_name);
	Eigen::MatrixXd newW;

	if (extension == "dmat")
	{
		igl::readDMAT(weights_file_name, newW);
	}
	else if (extension == "h5")
	{
#ifdef SUPPORT_HDF5_MAT
		readH5MAT(weights_file_name, newW);
#else
		printf("Unknown weights file type %s", weights_file_name);
		return false;
#endif
	}
	else
	{
		printf("Unknown weights file type %s", weights_file_name);
		return false;
	}

	printf("Load Weights Matrix:(%d,%d)\n", newW.rows(), newW.cols());

	set_weights(newW, weights_file_name);

	return true;
}

void DeformSkinningBase::do_this_when_weights_loaded()
{

	sortWeights();// position TBD
	computeDeformFactors();

	M3d_hs = Weights;
	M2d_hs = Weights;

	hasLoadWeights = true;
	if (hasLoadHandles&&hasLoadWeights)
	{
		// Set M matrix
		set_M_lbs_matrix();
		update_skinning = true;
	}
}

void DeformSkinningBase::do_this_when_only_M_loaded()
{
	hasLoadWeights = true;
	if (hasLoadHandles&&hasLoadWeights)
	{
		update_skinning = true;
	}
}

void copy_sparse_weights_to_dense(Eigen::MatrixXd& W, Eigen::MatrixXi& WI, Eigen::MatrixXd& Weights)
{
	assert(W.rows() == WI.rows() && W.cols() == WI.cols());
	Weights.resize(W.rows(), WI.maxCoeff() + 1);
	Weights.setZero();

	// copy to Weights
	for (int i = 0; i<WI.rows(); i++)
	{
		for (int j = 0; j<WI.cols(); j++)
		{
			int k = WI(i, j);
			Weights(i, k) = W(i, j);
		}
	}
}

bool DeformSkinningBase::load_sparse_weights_from_file(const char* sparse_weights_file_nameconst, const char* sparse_weight_indices_file_name)
{
	igl::readDMAT(sparse_weights_file_nameconst, W);
	igl::readDMAT(sparse_weight_indices_file_name, WI);

	copy_sparse_weights_to_dense(W, WI, Weights);

	printf("Load Weights Matrix:(%d,%d)\n", Weights.rows(), Weights.cols());

	// Weights is sorted once more which is not necessary.
	do_this_when_weights_loaded();

	return true;
}

bool DeformSkinningBase::load_deform_factors_from_file(const char* alpha_factors_file_nameconst, const char* beta_factors_file_nameconst)
{
	igl::readDMAT(alpha_factors_file_nameconst, alphaFactors);
	igl::readDMAT(beta_factors_file_nameconst, betaFactors);

	printf("Load deform factor Matrix:(%d,%d)\n", alphaFactors.rows(), alphaFactors.cols());

	return true;
}

bool DeformSkinningBase::save_deform_factors_to_file(const char* alpha_factors_file_nameconst, const char* beta_factors_file_nameconst)
{
	igl::writeDMAT(alpha_factors_file_nameconst, alphaFactors);
	igl::writeDMAT(beta_factors_file_nameconst, betaFactors);

	return true;
}

bool DeformSkinningBase::save_weights_to_file(const char* weights_file_name)
{
	//ori_Handles = Handles;


	std::string extension = extension_from_name(weights_file_name);

	if (extension == "dmat")
	{
		igl::writeDMAT(weights_file_name, Weights);
	}
	else if (extension == "h5")
	{
#ifdef SUPPORT_HDF5_MAT
		writeH5MAT(weights_file_name, Weights);
#else
		printf("Unknown weights file type %s", weights_file_name);
		return false;
#endif
	}
	else
	{
		printf("Unknown weights file type %s", weights_file_name);
		return false;
	}

	return true;
}

bool DeformSkinningBase::draw_weights_to_mesh(const std::vector<std::pair<int, Eigen::VectorXd>>& cols_to_draw)
{

	//region_colors_from_weights
	if (!hasLoadWeights)
		return false;

	Eigen::VectorXd color;

	if (!enable_draw_weights_on_mesh)
	{
		return false;
		//m_preview3d->vertex_property->resize(0,3);
	}
	else
	{
		int column = 0;
		const Eigen::MatrixXd& ToDraw = M3d_hs;

		color = MatrixXd::Zero(ToDraw.rows(), 1);
		//*m_preview3d->vertex_property = MatrixXd::Zero(ToDraw.rows(),1);
		printf("Drawing Weights:");
		for (int i = 0; i<cols_to_draw.size(); i++)
		{
			column = cols_to_draw[i].first;
			if (0 <= column && column<ToDraw.cols())
			{
				assert(ToDraw.rows() == cols_to_draw[i].second.rows());
				VectorX new_col = ToDraw.col(column).array() / cols_to_draw[i].second.array();
				//*m_preview3d->vertex_property 
				color += new_col;
				printf("%d ", column);
			}
			else
			{
				printf("Error: the column you want to draw exceeds the Weights Matrix!\n");
				return false;
			}
		}
		printf("\n");
		printf("Load Weights to Draw: (%d,%d)",
			m_preview3d->GetMainMesh().vertex_property->rows(),
			m_preview3d->GetMainMesh().vertex_property->cols());

		m_preview3d->GetMainMesh().draw_colors_on_mesh(color);

		//	m_preview3d->compute_isolevels();
		//	m_preview3d->compute_vertex_colors(
		//	m_preview3d->vertex_property,
		//	m_preview3d->vertex_colors);
		////m_preview3d->is_compiled = false;
		//m_preview3d->update_colors();

		return true;
	}
}

void DeformSkinningBase::computeDeformFactors()
{
	//Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic > F = *m_preview3d->faces;
	const auto md = m_preview3d->GetMainMesh();

	Eigen::MatrixXi F = md.faces->cast<int>();
	Eigen::MatrixXd V_rest = md.rest_vertices.cast<double>();
	Eigen::MatrixXd Normals_face0 = md.face_normals->cast<double>();// the normal at REST pose
	Eigen::MatrixXd Normals_vertex0 = md.vertex_normals->cast<double>(); // the normal at REST pose

	normal_deform_factors(V_rest, F, Normals_vertex0, Normals_face0, Weights, t_dir, b_dir, alphaFactors, betaFactors);

	int factor_cols = 4 * NUM_WEIGHT_GRADIENTS_SLOTS_IN_SHADER;
	if (WI.cols()<factor_cols)
	{
		factor_cols = WI.cols();
		printf("Warning: Sparse weights has less than %d slots per vertex.\n", 4 * NUM_WEIGHT_GRADIENTS_SLOTS_IN_SHADER);
	}
	sorted_alphaFactors.resize(WI.rows(), factor_cols);
	sorted_betaFactors.resize(WI.rows(), factor_cols);
	sorted_alphaFactors.setZero();
	sorted_betaFactors.setZero();
	for (int i = 0; i<WI.rows(); i++)
	{
		for (int j = 0; j<factor_cols; j++)
		{
			int k = WI(i, j);
			if (k >= alphaFactors.cols())
				continue;
			sorted_alphaFactors(i, j) = alphaFactors(i, k);
			sorted_betaFactors(i, j) = betaFactors(i, k);
		}
	}
}

void DeformSkinningBase::sortWeights()
{
	Eigen::MatrixXd EW(Weights.rows(), 0);
	sort_abs_weights(
		Weights, EW,
		4 * NUM_WEIGHTS_SLOTS_IN_SHADER,
		W, WI);
}

void DeformSkinningBase::reproject_sparse_weights()
{
	printf("Reinforce Weights.\n");

	sortWeights();//do not have to do this as long as weights have been sorted.

	Eigen::MatrixXd V_rest = m_preview3d->GetMainMesh().rest_vertices.cast<double>();
	Eigen::MatrixXd H = Handles();
	reproject_coordinate_individually(W, WI, V_rest, H, 3, 1.0);

	copy_sparse_weights_to_dense(W, WI, Weights);

	sortWeights();// position TBD
	computeDeformFactors();

}

void DeformSkinningBase::reproject_deform_factors()
{
	Eigen::MatrixXd H = Handles().cast<double>();

	reproject_coordinate_individually(sorted_alphaFactors, WI, alphaFactors*H, H, 3, 0.0);
	reproject_coordinate_individually(sorted_betaFactors, WI, betaFactors*H, H, 3, 0.0);
}

void DeformSkinningBase::draw_tangents()
{
	if (true)
	{
		glLineWidth(0.8);
		glColor3f(1.0, 0.0, 0.0);
		glBegin(GL_LINES);

		for (int i = 0; i<m_preview3d->GetMainMesh().vertices->rows(); i++)
		{
			Vector3 from((*m_preview3d->GetMainMesh().vertices)(i, 0), (*m_preview3d->GetMainMesh().vertices)(i, 1), (*m_preview3d->GetMainMesh().vertices)(i, 2));
			Vector3 to(t_dir(i, 0), t_dir(i, 1), t_dir(i, 2));//rest pose
			//Vector3 to( tang_dir(i,0), tang_dir(i,1), tang_dir(i,2) );
			to = to*0.01 + from;
			//paintArrow(from,to,2);
			glVertex3f(from(0), from(1), from(2));
			glVertex3f(to(0), to(1), to(2));
		}

		glEnd();
	}

	if (true)
	{
		glLineWidth(0.8);
		glColor3f(0.0, 1.0, 0.0);
		glBegin(GL_LINES);

		for (int i = 0; i<m_preview3d->GetMainMesh().vertices->rows(); i++)
		{
			Vector3 from((*m_preview3d->GetMainMesh().vertices)(i, 0), (*m_preview3d->GetMainMesh().vertices)(i, 1), (*m_preview3d->GetMainMesh().vertices)(i, 2));
			Vector3 to(b_dir(i, 0), b_dir(i, 1), b_dir(i, 2));//rest pose
			//Vector3 to( binormal_dir(i,0), binormal_dir(i,1), binormal_dir(i,2) );
			to = to*0.01 + from;
			//paintArrow(from,to,2);
			glVertex3f(from(0), from(1), from(2));
			glVertex3f(to(0), to(1), to(2));
		}

		glEnd();
	}

	if (true)
	{
		glLineWidth(0.8);
		glColor3f(0.0, 0.0, .0);
		glBegin(GL_LINES);

		for (int i = 0; i<m_preview3d->GetMainMesh().vertices->rows(); i++)
		{
			Vector3 from((*m_preview3d->GetMainMesh().vertices)(i, 0), (*m_preview3d->GetMainMesh().vertices)(i, 1), (*m_preview3d->GetMainMesh().vertices)(i, 2));
			Vector3 to((*m_preview3d->GetMainMesh().vertex_normals)(i, 0), (*m_preview3d->GetMainMesh().vertex_normals)(i, 1), (*m_preview3d->GetMainMesh().vertex_normals)(i, 2));
			//Vector3 to( binormal_dir(i,0), binormal_dir(i,1), binormal_dir(i,2) );
			to = to*0.01 + from;
			//paintArrow(from,to,2);
			glVertex3f(from(0), from(1), from(2));
			glVertex3f(to(0), to(1), to(2));
		}

		glEnd();
	}

	if (false)
	{
		glLineWidth(0.8);
		glColor3f(1.0, 0.0, 0.0);
		glBegin(GL_LINES);

		for (int i = 0; i<tang_dir.rows(); i++)
		{
			Vector3 from((*m_preview3d->GetMainMesh().vertices)(i, 0), (*m_preview3d->GetMainMesh().vertices)(i, 1), (*m_preview3d->GetMainMesh().vertices)(i, 2));
			//Vector3 to( t_dir(0,i), t_dir(1,i), t_dir(2,i) );//rest pose
			Vector3 to(tang_dir(i, 0), tang_dir(i, 1), tang_dir(i, 2));
			to = to*0.01 + from;
			//paintArrow(from,to,2);
			glVertex3f(from(0), from(1), from(2));
			glVertex3f(to(0), to(1), to(2));
		}

		glEnd();
	}

	if (false)
	{
		glLineWidth(0.8);
		glColor3f(0.0, 1.0, 0.0);
		glBegin(GL_LINES);

		for (int i = 0; i<binormal_dir.rows(); i++)
		{
			Vector3 from((*m_preview3d->GetMainMesh().vertices)(i, 0), (*m_preview3d->GetMainMesh().vertices)(i, 1), (*m_preview3d->GetMainMesh().vertices)(i, 2));
			//Vector3 to( b_dir(0,i), b_dir(1,i), b_dir(2,i) );//rest pose
			Vector3 to(binormal_dir(i, 0), binormal_dir(i, 1), binormal_dir(i, 2));
			to = to*0.01 + from;
			//paintArrow(from,to,2);
			glVertex3f(from(0), from(1), from(2));
			glVertex3f(to(0), to(1), to(2));
		}

		glEnd();
	}
}

//void DeformSkinningBase::update_handles_in_picking()
//{
//	switch(skinningType){
//	case LBS:
//		HandlePlugin::GetReference().update_handles_for_lbs();
//		break;
//	default: //PBS
//		HandlePlugin::GetReference().update_handles_in_picking();
//		break;
//	}
//}




