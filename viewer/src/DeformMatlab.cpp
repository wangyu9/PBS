#include "DeformMatlab.h"

static DeformMatlabUI  DeformMatlabUIInstance = DeformMatlabUI();

void DeformMatlabUI::init(Preview3D *preview)
{
	DeformMatlabBaseUI::init(preview);

	const auto& bar = DeformMatlabBaseUI::bar;

	if (bar)
	{
		bar->TwAddButton("Set Mesh From Viewer Main", DIALOG_OF(getMeshFromViewer), this, "");
		bar->TwAddButton("Restart Solver", DIALOG_OF(RestartSolver), this, "");
		bar->TwAddButton("Set Constraint From Picking", DIALOG_OF(getConstraintFromPickingPlugin), this, "");
	}
}

void DeformMatlabUI::preDraw(int currentTime)
{
	DeformMatlabBaseUI::preDraw(currentTime);
	setMeshToViewer();
}

void DeformMatlabUI::setMeshToViewer()
{
	MeshDisplay& md = m_preview->GetMainMesh();
	if (run_solver)
	{
		md.SetV(DeformMatlabBase::GetMeshV());
	}
}

void DeformMatlabUI::getMeshFromViewer()
{
	const Eigen::MatrixXd& V = m_preview->GetMainMesh().GetVR();
	const Eigen::MatrixXi& T = m_preview->GetMainMesh().GetT();
	const Eigen::MatrixXi& F = m_preview->GetMainMesh().GetF();

	DeformMatlabBase::InitMesh(V,T,F);
}

#include <plugins\PickingPlugin.h>
void DeformMatlabUI::getConstraintFromPickingPlugin()
{

	Eigen::VectorXi I;
	Eigen::MatrixXd P;
	PickingPlugin::GetReference().GetConstraint(I, P);
	SetConstraint(I, P);
}

void DeformMatlabUI::SetConstraint(const Eigen::VectorXi& I, const Eigen::MatrixXd& P)
{
	DeformMatlabBase::SetConstraint(I, P, Eigen::MatrixXd(), Eigen::MatrixXd());
}