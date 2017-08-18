#ifndef DEFORM_MATLAB_H
#define DEFORM_MATLAB_H

#include <viewer/DeformMatlabBaseUI.h>

#include <viewer/anttweak_helper.h>

#define DIALOG_VOID_FUN_VOID_REGISTER(fname,cname)  static void TW_CALL dialog_##fname (void *clientData){static_cast< cname*>(clientData)->fname();}

class DeformMatlabUI : public DeformMatlabBaseUI
{
public:
	
	// External Call
	void SetConstraint(const Eigen::VectorXi& I, const Eigen::MatrixXd& P);

	// overwitten existing one
	void init(Preview3D *preview);
	void preDraw(int currentTime);

	void getMeshFromViewer();
	void getConstraintFromPickingPlugin();
	void setMeshToViewer();

	DIALOG_VOID_FUN_VOID_REGISTER(getMeshFromViewer, DeformMatlabUI)
	DIALOG_VOID_FUN_VOID_REGISTER(RestartSolver, DeformMatlabUI)
	DIALOG_VOID_FUN_VOID_REGISTER(getConstraintFromPickingPlugin, DeformMatlabUI)
};

#endif /*DEFORM_MATLAB_H*/
