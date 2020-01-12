//#ifdef IGL_STATIC_LIBRARY
//#undef IGL_STATIC_LIBRARY
//#include <igl/biharmonic_coordinates.h>
//#define IGL_STATIC_LIBRARY
//#endif

#include "DeformSkinning.h"
#include "PluginManager.h"
#include "HandlePlugin.h"

// Declare the single entity of the plugin
// This should be in all the plugin headers
static DeformSkinningUI  DeformSkinningUIInstance = DeformSkinningUI();
DeformSkinning&  DeformSkinningInstance_(){ return DeformSkinningUIInstance; }

DeformSkinning& DeformSkinning::GetReference()
{
	return DeformSkinningUIInstance;
}

// Virtual Functions
Eigen::MatrixXd DeformSkinning::Handles(int dim) const
{
	return HandlePlugin::GetReference().Handles.cast<double>().leftCols(dim);
}

Eigen::MatrixXd DeformSkinning::RestHandles(int dim) const
{
	return HandlePlugin::GetReference().ori_Handles.cast<double>().leftCols(dim);
}

Eigen::MatrixXd DeformSkinning::HandleTrans(int dim) const
{
	return HandlePlugin::GetReference().GetTrans(dim); //HandlePlugin::GetReference().HandleTrans();
}

Eigen::MatrixXd DeformSkinning::HandleVars(int dim) const
{
	return HandlePlugin::GetReference().GetVars(dim);
}

Eigen::VectorXi DeformSkinning::HandleIndices() const
{
	return HandlePlugin::GetReference().HandleIndices;
}

void DeformSkinning::copy_skinned_mesh_back() const
{
	// copy vertex state to the main viewer
	for (int r = 0; r < m_preview->GetMainMesh().vertices->rows(); r++)
		for (int c = 0; c < m_preview->GetMainMesh().vertices->cols(); c++)
			m_preview->GetMainMesh().vertices->coeffRef(r, c) = pV->coeff(r, c);
}

// Overwriting Existing Functions
void DeformSkinning::do_this_when_weights_loaded()
{
	DeformSkinningBase::do_this_when_weights_loaded();
	HandlePlugin::GetReference().SetVisFromW(Weights);
}

void DeformSkinning::do_this_when_only_M_loaded()
{
	DeformSkinningBase::do_this_when_only_M_loaded();
	HandlePlugin::GetReference().SetVisFromM(M3d_hs);
}

bool DeformSkinning::load_tet_group_from_MATLAB()
{
	using namespace igl::matlab;

	if (!workSpaceClearedBeforeUse)
	{
		printf("Must load_weights_from_MATLAB first.\n");
		return false;
	}

	Eigen::MatrixXi tet_group;

	mleval(matlabEngine, "ExternalCallList_TetGroup;");
	mlgetmatrix(matlabEngine, "T_cluster", tet_group);

	if (tet_group.cols() != 1)
	{
		printf("load_tet_group_from_MATLAB fails!\n ");
		return false;
	}

	printf("Load Tet group of size (%d,%d) from MATLAB.\n", tet_group.rows(), tet_group.cols());

	DeformPhysUI::GetReference().set_rotation_cluster(tet_group.col(0));

	return true;
}

inline bool is_3d_vertices(const Eigen::MatrixXd& V)
{
	// Judging by the z axis
	return (V.col(2).maxCoeff() - V.col(2).minCoeff())>0.0001;
}

#include "matlab_folder_path.h"
bool DeformSkinning::send_data_to_MATALB()
{
	using namespace igl::matlab;
	std::string str_cd_folder = std::string("cd ") + std::string(MATLAB_FOLDER_PATH);
	mleval(matlabEngine, str_cd_folder);

	if (!workSpaceClearedBeforeUse)
	{
		//make sure this is call at the first time after loading the mesh
		mleval(matlabEngine, "clear;");
		workSpaceClearedBeforeUse = true;
	}

	const Eigen::MatrixXd& V_rest = m_preview3d->GetMainMesh().rest_vertices;//*m_preview3d->vertices;
	Eigen::MatrixXd C = Handles();


	Eigen::MatrixXi F = *m_preview3d->GetMainMesh().faces;

	/** Be careful that mlsetmatrix will add 1 for all interger type!**/

	mlsetscalar(matlabEngine, "weights_type", (double)((int)weightsType));
	mlsetmatrix(matlabEngine, "V", V_rest);
	mlsetmatrix(matlabEngine, "F", F);

	Eigen::MatrixXi T = *m_preview3d->GetMainMesh().tets;
	mlsetmatrix(matlabEngine, "T", T);

	const Eigen::MatrixXi B = HandlePlugin::GetReference().HandleIndices.cast<int>();
	const Eigen::MatrixXd BC = HandlePlugin::GetReference().BoundaryCondition().cast<double>();

	switch (skinningType)
	{
	case LBS:
	case PBS:
	case DQLBS:
	case None:
		break;
	case HS:
	{
		mlsetmatrix(matlabEngine, "M", M3d_hs_2nd);
	}
	}

	//B.resize(Handles().rows(),1);
	//for(int i=0; i<Handles().rows(); i++)
	//{
	//	int index = HandlePlugin::GetReference().HandleIndices(i);
	//	assert(index>=0);
	//	if(index>=0)
	//	{
	//		assert(Handles()(i,0) == (*m_preview3d->vertices)(index,0));
	//		assert(Handles()(i,1) == (*m_preview3d->vertices)(index,1));
	//		assert(Handles()(i,2) == (*m_preview3d->vertices)(index,2));
	//		C(i,0) = m_preview3d->rest_vertices(index,0);
	//		C(i,1) = m_preview3d->rest_vertices(index,1);
	//		C(i,2) = m_preview3d->rest_vertices(index,2);
	//		B(i,0) = index;
	//	}
	//}

	if (true)
	{
		mlsetmatrix(matlabEngine, "B", B);
	}
	else
	{
		mlsetmatrix(matlabEngine, "C", C);
	}
	mlsetmatrix(matlabEngine, "BC", BC);

	return true;
}

#include <compute_weights.h>


bool DeformSkinning::compute_weights()
{
	TimerWrapper timeWrapper;
	
	const MeshDisplay md = m_preview3d->GetMainMesh();

	const Eigen::MatrixXd V = md.rest_vertices;//*m_preview3d->vertices;
	const Eigen::MatrixXi F = md.faces->cast<int>();
	const Eigen::MatrixXi T = md.tets->cast<int>();

	const Eigen::MatrixXi TF = T.rows() ? T : F;

	const Eigen::MatrixXi B = HandlePlugin::GetReference().HandleIndices.cast<int>();
	const Eigen::MatrixXd BC = HandlePlugin::GetReference().BoundaryCondition().cast<double>();

	std::vector<std::vector<int>> S;
	for (int i = 0; i < B.rows(); i++)
	{
		std::vector<int> Si;
		Si.push_back(B(i,0));
		S.push_back(Si);
	}

	timeWrapper.Tic();

	switch (skinningType)
	{
	case LBS:
	case DQLBS:
	case PBS:
	case None:
	{

		Eigen::MatrixXd newW;
		bilaplacian_coordinates(V, TF, S, newW);

		if (newW.cols() == 0)
		{
			printf("Compute Weights fails!\n");
			return false;
		}
		
		set_weights(newW, "From Matlab");

		printf("Compute Weights Matrix:(%d,%d) using %f sec.\n", Weights.rows(), Weights.cols(), timeWrapper.Duration());

	}
	break;
	case HS:
	{
		const Eigen::MatrixXd& V_rest = m_preview3d->GetMainMesh().rest_vertices;
		printf("TF: min:%d, max:%d", TF.minCoeff(), TF.maxCoeff());
		bilaplacian_coordinates(V, TF, S, M3d_hs);
		//mlgetmatrix(matlabEngine, "W", M3d_hs);

		if (M3d_hs.cols() == 0)
		{
			printf("Load Hybrid Skinning Weights Fails!\n");
			return false;
		}

		HandlePlugin::GetReference().recover_M2d_from_M3d(M3d_hs, M2d_hs);
		//print_matlab("M2d_hs",M2d_hs);
		// only calculate and load M3d_hs
		printf("Load Hybrid Skinning Weights Matrix from MATLAB:(%d,%d) using %f sec.\n", M3d_hs.rows(), M3d_hs.cols(), timeWrapper.Duration());

		do_this_when_only_M_loaded();
	}
	break;
	}

	update_skinning = true;

	timeWrapper.Toc();

	return true;
}

//#include <print_matlab.h>
bool DeformSkinning::load_weights_from_MATLAB()
{
	using namespace igl::matlab;

	TimerWrapper timeWrapper;
	timeWrapper.Tic();

	send_data_to_MATALB();

	//igl::readDMAT(weights_file_name,Weights);
	//mleval(matlabEngine,"cd 'C:\\WorkSpace\\Visual Studio 2010\\lim\\data\\armadillo-full';");
	mleval(matlabEngine, "ExternalCallList;");



	switch (skinningType)
	{
	case LBS:
	case DQLBS:
	case PBS:
	case None:
	{
		//Weights.resize(V_rest.rows(),Handles().rows());
		Eigen::MatrixXd newW;
		mlgetmatrix(matlabEngine, "W", newW);

		if (newW.cols() == 0)
		{
			printf("Load Weights fails!\n");
			return false;
		}

		set_weights(newW, "From Matlab");

		printf("Load Weights Matrix from MATLAB:(%d,%d) using %f sec.\n", Weights.rows(), Weights.cols(), timeWrapper.Duration());
	}
	break;
	case HS:
	{
		const Eigen::MatrixXd& V_rest = m_preview3d->GetMainMesh().rest_vertices;
		mlgetmatrix(matlabEngine, "W", M3d_hs);

		if (M3d_hs.cols() == 0)
		{
			printf("Load Hybrid Skinning Weights Fails!\n");
			return false;
		}

		HandlePlugin::GetReference().recover_M2d_from_M3d(M3d_hs, M2d_hs);
		//print_matlab("M2d_hs",M2d_hs);
		// only calculate and load M3d_hs
		printf("Load Hybrid Skinning Weights Matrix from MATLAB:(%d,%d) using %f sec.\n", M3d_hs.rows(), M3d_hs.cols(), timeWrapper.Duration());

		do_this_when_only_M_loaded();
	}
	break;
	}

	update_skinning = true;

	//mlgetmatrix(matlabEngine, "H", (Eigen::MatrixXd) Handles);
	timeWrapper.Toc();
	//applySkinning();

	return true;
}

#include <extension_from_name.h>
#include <igl/readDMAT.h>
bool DeformSkinning::load_weights_from_file2(const char* weights_file_name)
{
	std::string extension = extension_from_name(weights_file_name);
	Eigen::MatrixXd newW;

	switch (skinningType)
	{
	case LBS:
	case DQLBS:
	case PBS:
	case None:
	{

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

		if (newW.cols() == 0)
		{
			printf("Load Weights fails!\n");
			return false;
		}

		set_weights(newW, "From file");

		printf("Load Weights Matrix from file. :(%d,%d).\n", Weights.rows(), Weights.cols());
	}
	break;
	case HS:
	{
		const Eigen::MatrixXd& V_rest = m_preview3d->GetMainMesh().rest_vertices;

		igl::readDMAT(weights_file_name, M3d_hs);

		if (M3d_hs.cols() == 0)
		{
			printf("Load Hybrid Skinning Weights Fails!\n");
			return false;
		}

		HandlePlugin::GetReference().recover_M2d_from_M3d(M3d_hs, M2d_hs);
		//print_matlab("M2d_hs",M2d_hs);
		// only calculate and load M3d_hs
		printf("Load Hybrid Skinning Weights Matrix from MATLAB:(%d,%d).\n", M3d_hs.rows(), M3d_hs.cols());

		do_this_when_only_M_loaded();
	}
	break;
	}

	update_skinning = true;

	return true;
}

void DeformSkinning::init(Preview3D *preview)
{
	bool init_antweakbar = (bar == NULL);

	DeformSkinningBaseUI::init(preview);

	init_antweakbar = init_antweakbar && (bar != NULL);

	if (init_antweakbar)
	{
		bar->TwAddButton("Compute Weights", dialog_compute_weights, this, "group='Weights'");
		bar->TwAddButton("Send Data to MATLAB", send_dialog_data_to_MATLAB, this, " group='Weights'");
		bar->TwAddButton("Load Weights form MATLAB", open_dialog_weights_from_MATLAB, this,
			" group='Weights'"
			" label='Load Weights form MATLAB' key=w");
		bar->TwAddButton("Load Tet Group form MATLAB", open_dialog_tet_group_from_MATLAB, this,
			" group='Weights'");
	}
}

void TW_CALL DeformSkinning::dialog_compute_weights(void *clientData)
{
	static_cast<DeformSkinning *>(clientData)->compute_weights();
}

void TW_CALL DeformSkinning::send_dialog_data_to_MATLAB(void *clientData)
{
	static_cast<DeformSkinning*>(clientData)->send_data_to_MATALB();
}

void TW_CALL DeformSkinning::open_dialog_weights_from_MATLAB(void *clientData)
{
	//move_dialog_handle_onto_mesh(clientData);
	HandlePlugin::GetReference().MoveHandlesOntoMesh();

	static_cast<DeformSkinning *>(clientData)->load_weights_from_MATLAB();
}

void TW_CALL DeformSkinning::open_dialog_tet_group_from_MATLAB(void *clientData)
{
	static_cast<DeformSkinning *>(clientData)->load_tet_group_from_MATLAB();
}



#include <viewer/CommandLineBase.h>
bool CommandLine(DeformSkinning& plugin, std::vector<std::string> cl)
{
	if (cl.size() < 1)
	{
		printf("Error: No Command Line for DeformSkinning.\n");
		return false;
	}
	else
	{
		printf("DeformSkinning");
		print_out_argv(cl);
		printf("\n");
	}

	if (cl[0] == std::string("load_weights_from_MATLAB"))
	{
		return plugin.load_weights_from_MATLAB();
	}
	else if (cl[0] == std::string("compute_weights"))
	{
		return plugin.compute_weights();
	}
	else if (cl[0] == std::string("load_weights_from_file"))
	{
		if (cl.size() < 2)
		{
			printf("Error: No file name for load_weights_from_file.\n");
			return false;
		}
		return plugin.load_weights_from_file(cl[1].c_str());
	}
	else if (cl[0] == std::string("load_weights_from_file2"))
	{
		if (cl.size() < 2)
		{
			printf("Error: No file name for load_weights_from_file2.\n");
			return false;
		}
		return plugin.load_weights_from_file2(cl[1].c_str());
	}
	else if (cl[0] == std::string("set_skinning_type"))
	{
		if (cl.size() < 2)
		{
			printf("Error: No file name for set_skinning_type.\n");
			return false;
		}
		return plugin.set_skinning_type(cl[1].c_str());
	}
	else if (cl[0] == std::string("set_weights_type"))
	{
		if (cl.size() < 2)
		{
			printf("Error: No file name for set_weights_type.\n");
			return false;
		}
		return plugin.set_weights_type(cl[1].c_str());
	}
	else
	{
		printf("Error: Unknown Command Line Type for Viewer.\n");
		return false;
	}
}

bool DeformSkinning::commandLine(std::string c, std::vector<std::string> cl)
{
	if (c == std::string("DeformSkinning"))
	{
		return CommandLine(*this, cl);
	}
	return false;
}