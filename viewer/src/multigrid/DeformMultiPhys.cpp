#include "DeformMultiPhys.h"



static DeformMultiPhysUI  DeformMultiPhysUIInstance = DeformMultiPhysUI();

DeformMultiPhysUI& DeformMultiPhysUI::GetReference()
{
	return DeformMultiPhysUIInstance;
}

DeformMultiPhysUI::DeformMultiPhysUI():
	recording_mesh(false),
	recorded_mesh_index(0)
{
}

void DeformMultiPhysUI::init(Preview3D *preview)
{
	DeformMultiPhysBaseUI::init(preview);

	const auto& bar = DeformMultiPhysBaseUI::bar;

	if (bar)
	{
		bar->TwAddButton("Set Mesh From Viewer Main", DIALOG_OF(getMeshFromViewer), this, "");
		bar->TwAddButton("Set Constraint From Picking", DIALOG_OF(getConstraintFromPickingPlugin), this, "");
		bar->TwAddButton("Restart Solver", DIALOG_OF(RestartSolver), this, "");	
		
		bar->TwAddButton("Step Once", DIALOG_OF(UpdateSolverOnce), this, "");
		bar->TwAddButton("Temp Command", DIALOG_OF(TempCommand), this, "");

		bar->TwAddVarCB("Recording Mesh", TW_TYPE_BOOLCPP, DIALOG_OF(set_recording_mesh), DIALOG_OF(get_recording_mesh), this, "");


	}
}

void DeformMultiPhysUI::preDraw(int currentTime)
{
	DeformMultiPhysBase::Update();
	//DeformMultiPhysBaseUI::preDraw(currentTime);
	if (run_solver)
	{
		setMeshToViewer();
	}
}

void DeformMultiPhysUI::UpdateSolverOnce()
{
	DeformMultiPhysBase::UpdateSolverOnce();
	setMeshToViewer();
}

VOID DeformMultiPhysUI::TempCommand()
{
			DeformMultiPhysBase::UpdateSolverOnce();
			setMeshToViewer();
}

#include <igl/writeMESH.h>
#include <string.h>
void DeformMultiPhysUI::setMeshToViewer()
{
	MeshDisplay& md = m_preview->GetMainMesh();
	md.SetV(DeformMultiPhysBase::GetMeshV());

	if (recording_mesh)
	{
		char anim_filename[256];
		sprintf_s(anim_filename, 256, "frame%04d.mesh", recorded_mesh_index++);
		igl::writeMESH(anim_filename, md.GetV(), md.GetT(), md.GetF());
	}
}

void DeformMultiPhysUI::getMeshFromViewer()
{
	const Eigen::MatrixXd& V = m_preview->GetMainMesh().GetVR();
	const Eigen::MatrixXi& T = m_preview->GetMainMesh().GetT();
	const Eigen::MatrixXi& F = m_preview->GetMainMesh().GetF();

	DeformMultiPhysBase::InitMesh(V,T,F);
}

#include <multigrid/PickingPluginMG.h>
void DeformMultiPhysUI::getConstraintFromPickingPlugin()
{

	Eigen::VectorXi I;
	Eigen::MatrixXd P;
	PickingPluginMG::GetReference().GetConstraint(I, P);
	SetConstraint(I, P);
}

void DeformMultiPhysUI::SetConstraint(const Eigen::VectorXi& I, const Eigen::MatrixXd& P)
{
	DeformMultiPhysBase::SetConstraint(I, P, Eigen::MatrixXd(), Eigen::MatrixXd());
}

void DeformMultiPhysUI::set_recording_mesh(bool r)
{
	recording_mesh = r;
}

bool DeformMultiPhysUI::get_recording_mesh() const
{
	return recording_mesh;
}


#include <viewer/CommandLineBase.h>
bool CommandLine(DeformMultiPhysUI& plugin, std::vector<std::string> cl)
{
	if (cl.size() < 1)
	{
		printf("Error: No Command Line for DeformMultiPhysUI.\n");
		return false;
	}
	else
	{
		printf("DeformMultiPhysUI");
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

bool DeformMultiPhysUI::commandLine(std::string c, std::vector<std::string> cl)
{
	if (c == std::string("DeformMultiPhys"))
	{
		return CommandLine(*this, cl);
	}
	return false;
}