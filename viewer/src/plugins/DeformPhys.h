#ifndef DEFORM_PHYS_H
#define DEFORM_PHYS_H

#include <viewer\DeformPhysBase.h>


class DeformPhysUI: public DeformPhysBaseUI
{
public:

	static DeformPhysUI& GetReference();

	void prepare_connect();// prepare to be connected to skinning deformer

	// initialization (runs every time a mesh is loaded or cleared)
	void init(Preview3D *preview);
	void preDraw(int currentTime);

protected:

	//bool connected_to_skinning;
	//void SetConnectSkinning(bool connect)
	//{
	//	connected_to_skinning = connect;
	//	prepare_connect();
	//}
	//bool GetConnectSkinning() const
	//{
	//	return connected_to_skinning;
	//}

	static void TW_CALL prepare_dialog(void *clientData)
	{
		static_cast<DeformPhysUI *>(clientData)->prepare_connect();
	}

	virtual void GetSubspaceVar(const int dim, Eigen::MatrixXd& Var);
	virtual void GetSubspaceBases(const int dim, Eigen::MatrixXd& M);
	virtual void SetToOutput(const int dim, const Eigen::MatrixXd& Var);

private:
	void GetFromDeformSkinning(const int dim, Eigen::MatrixXd& Var);
	void SetToDeformSkinning(const int dim, const Eigen::MatrixXd& Var);


	//static void TW_CALL SetConnectSkinningCB(const void *value, void *clientData)
	//{
	//	static_cast<DeformPhysUI*>(clientData)->SetConnectSkinning(*static_cast<const bool *>(value));
	//}
	//static void TW_CALL GetConnectSkinningCB(void *value, void *clientData)
	//{
	//	*static_cast<bool *>(value) = static_cast<const DeformPhysUI*>(clientData)->GetConnectSkinning();
	//}
	bool commandLine(std::string c, std::vector<std::string> cl);
};

#endif