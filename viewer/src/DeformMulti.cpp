#include "DeformMulti.h"



static DeformMultiUI  DeformMultiUIInstance = DeformMultiUI();

DeformMultiUI& DeformMultiUI::GetReference()
{
	return DeformMultiUIInstance;
}

DeformMultiUI::DeformMultiUI():
	update_id(0), get_id(0),
	recording_mesh(false),
	recorded_mesh_index(0)
{
	get_id = DeformMultiBase::GetMultiSolverNumLayer() - 1;
}

void DeformMultiUI::init(Preview3D *preview)
{
	DeformMultiBaseUI::init(preview);

	const auto& bar = DeformMultiBaseUI::bar;

	if (bar)
	{
		bar->TwAddButton("Set Mesh From Viewer Main", DIALOG_OF(getMeshFromViewer), this, "");
		bar->TwAddButton("Set Constraint From Picking", DIALOG_OF(getConstraintFromPickingPlugin), this, "");
		bar->TwAddButton("Restart Solver", DIALOG_OF(RestartSolver), this, "");	
		
		bar->TwAddButton("Step Once", DIALOG_OF(UpdateSolverOnce), this, "");
		bar->TwAddButton("Temp Command", DIALOG_OF(TempCommand), this, "");

		bar->TwAddVarRW("Output Layer ID", TW_TYPE_INT32, &get_id, "min=0 step=1");
		bar->TwAddVarRW("Update ID", TW_TYPE_INT32, &update_id, "min=0 step=1");
	
		bar->TwAddVarCB("Recording Mesh", TW_TYPE_BOOLCPP, DIALOG_OF(set_recording_mesh), DIALOG_OF(get_recording_mesh), this, "");


	}
}

void DeformMultiUI::preDraw(int currentTime)
{
	DeformMultiBase::Update(update_id,get_id);
	//DeformMultiBaseUI::preDraw(currentTime);
	if (run_solver)
	{
		setMeshToViewer();
	}
}

void DeformMultiUI::UpdateSolverOnce()
{
	DeformMultiBase::UpdateSolverOnce(update_id,get_id);
	setMeshToViewer();
}

VOID DeformMultiUI::TempCommand()
{
	for (int j = 0; j < DeformMultiBase::GetMultiSolverNumLayer(); j++)
	{
		update_id = j;
		for (int i = 0; i < 150; i++)
		{
			DeformMultiBase::UpdateSolverOnce(update_id, get_id);
			setMeshToViewer();
		}
	}
}

#include <igl/writeMESH.h>
#include <string.h>
void DeformMultiUI::setMeshToViewer()
{
	MeshDisplay& md = m_preview->GetMainMesh();
	md.SetV(DeformMultiBase::GetMeshV());

	if (recording_mesh)
	{
		char anim_filename[256];
		sprintf_s(anim_filename, 256, "frame%04d.mesh", recorded_mesh_index++);
		igl::writeMESH(anim_filename, md.GetV(), md.GetT(), md.GetF());
	}
}

void DeformMultiUI::getMeshFromViewer()
{
	const Eigen::MatrixXd& V = m_preview->GetMainMesh().GetVR();
	const Eigen::MatrixXi& T = m_preview->GetMainMesh().GetT();
	const Eigen::MatrixXi& F = m_preview->GetMainMesh().GetF();

	DeformMultiBase::InitMesh(V,T,F);
}

#include <multigrid/PickingPluginMG.h>
void DeformMultiUI::getConstraintFromPickingPlugin()
{

	Eigen::VectorXi I;
	Eigen::MatrixXd P;
	PickingPluginMG::GetReference().GetConstraint(I, P);
	SetConstraint(I, P);
}

void DeformMultiUI::SetConstraint(const Eigen::VectorXi& I, const Eigen::MatrixXd& P)
{
	DeformMultiBase::SetConstraint(I, P, Eigen::MatrixXd(), Eigen::MatrixXd());
}

void DeformMultiUI::set_recording_mesh(bool r)
{
	recording_mesh = r;
}

bool DeformMultiUI::get_recording_mesh() const
{
	return recording_mesh;
}


#include <viewer/CommandLineBase.h>
bool CommandLine(DeformMultiUI& plugin, std::vector<std::string> cl)
{
	if (cl.size() < 1)
	{
		printf("Error: No Command Line for DeformMultiUI.\n");
		return false;
	}
	else
	{
		printf("DeformMultiUI");
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

bool DeformMultiUI::commandLine(std::string c, std::vector<std::string> cl)
{
	if (c == std::string("DeformMulti"))
	{
		return CommandLine(*this, cl);
	}
	return false;
}