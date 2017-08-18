#ifndef DEFORM_MULTI_PHYS_H
#define DEFORM_MULTI_PHYS_H

#include <viewer/DeformMultiPhysBaseUI.h>

#include <viewer/anttweak_helper.h>

#define DIALOG_VOID_FUN_VOID_REGISTER(fname,cname)  static void TW_CALL dialog_##fname (void *clientData){static_cast< cname*>(clientData)->fname();}

class DeformMultiPhysUI : public DeformMultiPhysBaseUI
{
public:
	
	DeformMultiPhysUI();

	static DeformMultiPhysUI& GetReference();

	// External Call
	void SetConstraint(const Eigen::VectorXi& I, const Eigen::MatrixXd& P);

	// overwitten existing one
	void init(Preview3D *preview);
	void preDraw(int currentTime);
	void UpdateSolverOnce();

	void getMeshFromViewer();
	void getConstraintFromPickingPlugin();
	void setMeshToViewer();

	void TempCommand();

	DIALOG_VOID_FUN_VOID_REGISTER(getMeshFromViewer, DeformMultiPhysUI)
	DIALOG_VOID_FUN_VOID_REGISTER(UpdateSolverOnce, DeformMultiPhysUI)

	DIALOG_VOID_FUN_VOID_REGISTER(RestartSolver, DeformMultiPhysUI)
	DIALOG_VOID_FUN_VOID_REGISTER(getConstraintFromPickingPlugin, DeformMultiPhysUI)

	DIALOG_VOID_FUN_VOID_REGISTER(TempCommand, DeformMultiPhysUI)

	DIALOG_SET_REGISTER(set_recording_mesh, bool, DeformMultiPhysUI)
	DIALOG_GET_REGISTER(get_recording_mesh, bool, DeformMultiPhysUI)

	bool commandLine(std::string c, std::vector<std::string> cl);

public:
	void set_recording_mesh(bool r);
	bool get_recording_mesh() const;

private:

	bool recording_mesh;
	int recorded_mesh_index;
};

#endif /*DEFORM_MULTI_PHYS_H*/
