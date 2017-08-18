#ifndef DEFORM_ARAP_H
#define DEFORM_ARAP_H

#include <viewer/DeformARAPBaseUI.h>

#include <viewer/anttweak_helper.h>

#define DIALOG_VOID_FUN_VOID_REGISTER(fname,cname)  static void TW_CALL dialog_##fname (void *clientData){static_cast< cname*>(clientData)->fname();}

class DeformARAPUI : public DeformARAPBaseUI
{
public:
	
	DeformARAPUI();

	static DeformARAPUI& GetReference();

	// External Call
	void SetConstraint(const Eigen::VectorXi& I, const Eigen::MatrixXd& P);

	// overwitten existing one
	void init(Preview3D *preview);
	void preDraw(int currentTime);
	void UpdateSolverOnce();

	void getMeshFromViewer();
	void getConstraintFromPickingPlugin();
	void setMeshToViewer();

	DIALOG_VOID_FUN_VOID_REGISTER(getMeshFromViewer, DeformARAPUI)
	DIALOG_VOID_FUN_VOID_REGISTER(UpdateSolverOnce, DeformARAPUI)
	DIALOG_VOID_FUN_VOID_REGISTER(RestartSolver, DeformARAPUI)
	DIALOG_VOID_FUN_VOID_REGISTER(getConstraintFromPickingPlugin, DeformARAPUI)

	DIALOG_SET_REGISTER(set_recording_mesh, bool, DeformARAPUI)
	DIALOG_GET_REGISTER(get_recording_mesh, bool, DeformARAPUI)

	bool commandLine(std::string c, std::vector<std::string> cl);

public:
	void set_recording_mesh(bool r);
	bool get_recording_mesh() const;

private:
	bool recording_mesh;
	int recorded_mesh_index;
};

#endif /*DEFORM_MULTI_H*/
