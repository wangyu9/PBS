#include "DeformPhys.h"


// Declare the single entity of the plugin
// This should be in all the plugin headers
DeformPhysUI  DeformPhysBaseUIInstance;
DeformPhysUI&  DeformPhysBaseInstance_(){ return DeformPhysBaseUIInstance; }


DeformPhysUI& DeformPhysUI::GetReference()
{
	return DeformPhysBaseUIInstance;
}


void DeformPhysUI::init(Preview3D *preview)
{
	DeformPhysBaseUI::init(preview);

	if (DeformPhysBase::initEntireMesh(
		m_preview->GetMainMesh().rest_vertices.cast<double>(),
		m_preview->GetMainMesh().tets->cast<int>(),
		m_preview->GetMainMesh().faces->cast<int>()))
	{
		Eigen::MatrixXd Var;
		GetSubspaceVar(DeformPhysBase::get_dim(), Var);
		Eigen::MatrixXd SubBases;
		GetSubspaceBases(DeformPhysBase::get_dim(), SubBases);
		initVariables(Var, SubBases);
	}

	//if (DeformPhysBase::Dim == 2)
	//{
	//	m_preview->set_toggle_ortho(true);
	//	m_preview->view_xy_plane();
	//	m_preview->enable_rotation = false;
	//}
	//else
	//{
	//	m_preview->set_toggle_ortho(false);
	//	m_preview->enable_rotation = true;
	//}

}

void DeformPhysUI::preDraw(int currentTime)
{
	DeformPhysBaseUI::preDraw(currentTime);
	if(DeformPhysBaseUI::isMeshLoaded&&DeformPhysBaseUI::runSolver)
		SetToOutput(DeformPhysBase::get_dim(), DeformPhysBase::get_variables());
}



/************* Commuication Functions *************/

void DeformPhysUI::GetSubspaceVar(const int dim, Eigen::MatrixXd& Var)
{
	GetFromDeformSkinning(dim, Var);
}

#include "DeformSkinning.h"
void DeformPhysUI::SetToOutput(const int dim, const Eigen::MatrixXd& Var)
{
	SetToDeformSkinning(dim, Var);

	//*m_preview->vertices = geometrySolver.V;
	m_preview->GetMainMesh().update_normal(NormalUpdateMethod::NORMAL_UPDATE_PBS);

	DeformSkinning::GetReference().update_skinning = true;
}

#include "HandlePlugin.h"
#include "DeformSkinning.h"

void DeformPhysUI::GetFromDeformSkinning(const int dim, Eigen::MatrixXd& Var)
{

	DeformSkinning::GetReference().SendVarToDeformPhys(dim, Var);

}

void DeformPhysUI::SetToDeformSkinning(const int dim, const Eigen::MatrixXd& Var)
{
	HandlePlugin::GetReference().UpdateForDeformSkinningHS(Var);
	return;
}

void DeformPhysUI::GetSubspaceBases(const int dim, Eigen::MatrixXd& M)
{
	DeformSkinning::GetReference().SendBasisToDeformPhys(dim, M);
}

//#include "PickingPlugin.h"
void DeformPhysUI::prepare_connect()
{
	if (DeformPhysBase::initEntireMesh(
		m_preview->GetMainMesh().rest_vertices.cast<double>(),
		m_preview->GetMainMesh().tets->cast<int>(),
		m_preview->GetMainMesh().faces->cast<int>()))
	{
		Eigen::MatrixXd Var;
		GetSubspaceVar(DeformPhysBase::get_dim(), Var);
		Eigen::MatrixXd SubBases;
		GetSubspaceBases(DeformPhysBase::get_dim(), SubBases);
		initVariables(Var, SubBases);
	}


	//PickingPlugin::GetReference().init_Picking();
	//HandlePlugin::GetReference().init_picking();

	//update();
}

/********* End of Commuication Functions **********/


#include <viewer/CommandLineBase.h>
bool CommandLine(DeformPhysUI& plugin, std::vector<std::string> cl)
{
	if (cl.size() < 1)
	{
		printf("Error: No Command Line for Viewer.\n");
		return false;
	}
	else
	{
		printf("DeformPhys");
		print_out_argv(cl);
		printf("\n");
	}

	if (cl[0] == std::string("load_rotation_cluster_from_file"))
	{
		if (cl.size() < 2)
		{
			printf("Error: No rotation cluster file name for load_mesh_from_file.\n");
			return false;
		}

		plugin.load_rotation_cluster_from_file(cl[1].c_str());
		// should not need this but need to pass the mesh to deformphys
		plugin.prepare_connect();
		return true;
	}
	else if (cl[0] == std::string("load_per_group_weight_from_file"))
	{
		if (cl.size() < 2)
		{
			printf("Error: No file name for load_per_group_weight_from_file.\n");
			return false;
		}
		plugin.load_per_group_weight_from_file(cl[1].c_str());
		return true;
	}
	else if (cl[0] == std::string("set_FPS_rotation_cluster"))
	{
		if (cl.size() < 2)
		{
			plugin.set_FPS_rotation_cluster();
		}
		else
		{
			plugin.set_FPS_rotation_cluster(atoi(cl[1].c_str()));
		}
		plugin.prepare_connect();
		return true;
	}
	else if (cl[0] == std::string("start_solver"))
	{
		//plugin.prepare_connect();
		plugin.runSolver = true;
		return true;
	}
	else if (cl[0] == std::string("set_with_dynamics"))
	{
		bool dynamics = true;
		if (cl.size() < 2)
		{
			printf("Warning: not specified true or false for with_dynamics, default is true.\n");
		}
		dynamics = std::stoi(cl[1].c_str());
		plugin.set_with_dynamics(dynamics);
		return true;
	}
	else if (cl[0] == std::string("set_rho"))
	{
		if (cl.size() < 2)
		{
			printf("Error: No file name for %s.\n", std::string("set_rho"));
			return false;
		}
		plugin.set_rho(std::stod(cl[1].c_str()));
		return true;
	}
	else if (cl[0] == std::string("set_mu"))
	{
		if (cl.size() < 2)
		{
			printf("Error: No file name for %s.\n", std::string("set_mu"));
			return false;
		}
		plugin.set_mu(std::stod(cl[1].c_str()));
		return true;
	}
	else if (cl[0] == std::string("set_time_step"))
	{
		if (cl.size() < 2)
		{
			printf("Error: No file name for %s.\n", std::string("set_time_step"));
			return false;
		}
		plugin.set_time_step(std::stod(cl[1].c_str()));
		return true;
	}
	else
	{
		printf("Error: Unknown Command Line Type for Viewer.\n");
		return false;
	}
}

bool DeformPhysUI::commandLine(std::string c, std::vector<std::string> cl)
{
	if (c == std::string("DeformPhys"))
	{
		return CommandLine(*this, cl);
	}
	return false;
}