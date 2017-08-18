#include "DeformARAP.h"



static DeformARAPUI  DeformARAPUIInstance = DeformARAPUI();

DeformARAPUI& DeformARAPUI::GetReference()
{
	return DeformARAPUIInstance;
}

DeformARAPUI::DeformARAPUI():
	recording_mesh(false),
	recorded_mesh_index(0)
{
}

void DeformARAPUI::init(Preview3D *preview)
{
	DeformARAPBaseUI::init(preview);

	const auto& bar = DeformARAPBaseUI::bar;

	if (bar)
	{
		bar->TwAddButton("Set Mesh From Viewer Main", DIALOG_OF(getMeshFromViewer), this, "");
		bar->TwAddButton("Set Constraint From Picking", DIALOG_OF(getConstraintFromPickingPlugin), this, "");
		bar->TwAddButton("Restart Solver", DIALOG_OF(RestartSolver), this, "");	
		bar->TwAddButton("Step Once", DIALOG_OF(UpdateSolverOnce), this, "");

		bar->TwAddVarCB("Recording Mesh", TW_TYPE_BOOLCPP, DIALOG_OF(set_recording_mesh), DIALOG_OF(get_recording_mesh), this, "");
	}
}

void DeformARAPUI::preDraw(int currentTime)
{
	DeformARAPBaseUI::preDraw(currentTime);
	if (run_solver)
	{
		setMeshToViewer();
	}
}

void DeformARAPUI::UpdateSolverOnce()
{
	DeformARAPBase::UpdateSolverOnce();
	setMeshToViewer();
}

#include <igl/writeMESH.h>
#include <string.h>
void DeformARAPUI::setMeshToViewer()
{
	MeshDisplay& md = m_preview->GetMainMesh();
	md.SetV(DeformARAPBase::GetMeshV());

	if (recording_mesh)
	{
		char anim_filename[256];
		sprintf_s(anim_filename, 256, "frame%04d.mesh", recorded_mesh_index++);
		igl::writeMESH(anim_filename, md.GetV(), md.GetT(), md.GetF());
	}
}

void DeformARAPUI::getMeshFromViewer()
{
	const Eigen::MatrixXd& V = m_preview->GetMainMesh().GetVR();
	const Eigen::MatrixXi& T = m_preview->GetMainMesh().GetT();
	const Eigen::MatrixXi& F = m_preview->GetMainMesh().GetF();

	DeformARAPBase::InitMesh(V,T,F);
}

#include <multigrid/PickingPluginMG.h>
void DeformARAPUI::getConstraintFromPickingPlugin()
{

	Eigen::VectorXi I;
	Eigen::MatrixXd P;
	PickingPluginMG::GetReference().GetConstraint(I, P);
	SetConstraint(I, P);
}

void DeformARAPUI::SetConstraint(const Eigen::VectorXi& I, const Eigen::MatrixXd& P)
{
	DeformARAPBase::SetConstraint(I, P, Eigen::MatrixXd(), Eigen::MatrixXd());
}

void DeformARAPUI::set_recording_mesh(bool r)
{
	recording_mesh = r;
}

bool DeformARAPUI::get_recording_mesh() const
{
	return recording_mesh;
}

#include <viewer/CommandLineBase.h>
bool CommandLine(DeformARAPUI& plugin, std::vector<std::string> cl)
{
	if (cl.size() < 1)
	{
		printf("Error: No Command Line for DeformARAPUI.\n");
		return false;
	}
	else
	{
		printf("DeformARAPUI");
		print_out_argv(cl);
		printf("\n");
	}

	if (cl[0] == std::string("get_mesh_from_viewer"))
	{
		plugin.getMeshFromViewer();
		return true;
	}
	else if (cl[0] == std::string("get_constraint_from_picking"))
	{
		plugin.getConstraintFromPickingPlugin();
		return true;
	}
	else if (cl[0] == std::string("restart_solver"))
	{
		plugin.RestartSolver();
		return true;
	}
	else
	{
		printf("Error: Unknown Command Line Type for Viewer.\n");
		return false;
	}
}

bool DeformARAPUI::commandLine(std::string c, std::vector<std::string> cl)
{
	if (c == std::string("DeformARAP"))
	{
		return CommandLine(*this, cl);
	}
	return false;
}