//
//  PickingPlugin.h
//  Preview3D
//
//

#ifndef DEFORM_SKINNING_H
#define DEFORM_SKINNING_H

#include <viewer/DeformSkinningBase.h>
#include "ViewerPlugin.h"

class DeformSkinning : public DeformSkinningBaseUI
{
private:
	/*********** Realization of Virtual Functions ****************/ 
	Eigen::MatrixXd RestHandles(int dim) const;
	Eigen::MatrixXd Handles(int dim = 3) const;//TODO remove this default value
	Eigen::MatrixXd HandleTrans(int dim) const;
	Eigen::MatrixXd HandleVars(int dim) const;
	Eigen::VectorXi HandleIndices() const;

	void copy_skinned_mesh_back() const;
	/*************************************************************/

	/************ Overwritting Existing Functions ****************/
	virtual void do_this_when_weights_loaded();
	virtual void do_this_when_only_M_loaded();

public:
	static DeformSkinning& GetReference();
	void init(Preview3D* preview);// overwritten existing one
public:
	bool compute_weights();
	bool load_weights_from_MATLAB();
	bool send_data_to_MATALB();
	bool load_tet_group_from_MATLAB();

	static void TW_CALL dialog_compute_weights(void *clientData);
	static void TW_CALL send_dialog_data_to_MATLAB(void *clientData);
	static void TW_CALL open_dialog_weights_from_MATLAB(void *clientData);
	static void TW_CALL open_dialog_tet_group_from_MATLAB(void *clientData);

public:
	bool commandLine(std::string c, std::vector<std::string> cl);
};

class DeformSkinningUI : public DeformSkinning
{

};

#endif