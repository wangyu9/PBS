#ifndef DEFORM_MULTI_H
#define DEFORM_MULTI_H

#include <viewer/DeformMultiBaseUI.h>

#include <viewer/anttweak_helper.h>

#define DIALOG_VOID_FUN_VOID_REGISTER(fname,cname)  static void TW_CALL dialog_##fname (void *clientData){static_cast< cname*>(clientData)->fname();}

class DeformMultiUI : public DeformMultiBaseUI
{
public:
	
	DeformMultiUI();

	static DeformMultiUI& GetReference();

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

	DIALOG_VOID_FUN_VOID_REGISTER(getMeshFromViewer, DeformMultiUI)
	DIALOG_VOID_FUN_VOID_REGISTER(UpdateSolverOnce, DeformMultiUI)

	DIALOG_VOID_FUN_VOID_REGISTER(RestartSolver, DeformMultiUI)
	DIALOG_VOID_FUN_VOID_REGISTER(getConstraintFromPickingPlugin, DeformMultiUI)

	DIALOG_VOID_FUN_VOID_REGISTER(TempCommand, DeformMultiUI)

	DIALOG_SET_REGISTER(set_recording_mesh, bool, DeformMultiUI)
	DIALOG_GET_REGISTER(get_recording_mesh, bool, DeformMultiUI)

	bool commandLine(std::string c, std::vector<std::string> cl);

public:
	void set_recording_mesh(bool r);
	bool get_recording_mesh() const;

private:
	int update_id;
	int get_id;

	bool recording_mesh;
	int recorded_mesh_index;
};

#endif /*DEFORM_MULTI_H*/
