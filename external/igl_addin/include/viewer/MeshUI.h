#ifndef MESH_UI_H
#define MESH_UI_H

#include "Mesh.h"

#include <libigl_removed/ReAntTweakBar.h>

#define NUM_COLOR_PRESET_IDS 5
#define NUM_COLOR_PRESETS NUM_COLOR_PRESET_IDS-1

#define CLASS_NAME MeshDisplayUI

class MeshDisplayUI : public MeshDisplay
{
private:
	// Pointer to the tweak bar
	igl::ReTwBar* bar;

public:

	MeshDisplayUI();
	void init(const std::string& name = std::string("MeshDisplay"));

	static void TW_CALL invert_normalsCB(void *clientData);
	static void TW_CALL SetShowTextureCB(const void *value, void *clientData);
	static void TW_CALL GetShowTextureCB(void *value, void *clientData);
	static void TW_CALL set_corner_thresholdCB(const void *value, void *clientData);
	static void TW_CALL get_corner_thresholdCB(void *value, void *clientData);
	static void TW_CALL set_numIsoLevelsCB(const void *value, void *clientData);
	static void TW_CALL get_numIsoLevelsCB(void *value, void *clientData);

	static void TW_CALL SetLineWidthCB(const void *value, void *clientData);
	static void TW_CALL GetLineWidthCB(void *value, void *clientData);
	static void TW_CALL open_dialog_texture(void *clientData);
	static void TW_CALL open_dialog_mesh(void *clientData);
	static void TW_CALL open_dialog_property(void *clientData);//wangyu
	static void TW_CALL save_dialog_mesh(void *clientData);//wangyu
	static void TW_CALL save_dialog_tets(void *clientData);//wangyu

	static void TW_CALL ReLoadShader(void *clientData);//wangyu
#ifdef USE_MATLAB
	static void TW_CALL SendStatusToMatlabCB(void *clientData);//wangyu
#endif
	static void TW_CALL compile_dialog_mesh(void *clientData);

	static void TW_CALL open_dialog_pose_mesh(void *clientData);



	static void TW_CALL SetMeshDrawingTypeCB(const void *value, void *clientData)
	{
		static_cast<CLASS_NAME*>(clientData)->SetMeshDrawingType(*static_cast<const MeshDrawingType *>(value));
	}
	static void TW_CALL GetMeshDrawingTypeCB(void *value, void *clientData)
	{
		*static_cast<MeshDrawingType *>(value) = static_cast<const CLASS_NAME*>(clientData)->GetMeshDrawingType();
	}
#ifdef USE_MATLAB
	static void TW_CALL SendColorToMatlabCB(void *clientData)
	{
		static_cast<CLASS_NAME*>(clientData)->send_color_to_matlab();
	}
#endif
	static void TW_CALL SetColorBarTypeCB(const void *value, void *clientData)
	{
		static_cast<CLASS_NAME*>(clientData)->SetColorBarType(*static_cast<const ColorBarType *>(value));
	}
	static void TW_CALL GetColorBarTypeCB(void *value, void *clientData)
	{
		*static_cast<ColorBarType *>(value) = static_cast<const CLASS_NAME*>(clientData)->GetColorBarType();
	}

	static void TW_CALL SetFlipYCoordCB(const void *value, void *clientData)
	{
		static_cast<CLASS_NAME*>(clientData)->SetFlipYCoord(*static_cast<const bool *>(value));
	}
	static void TW_CALL GetFlipYCoordCB(void *value, void *clientData)
	{
		*static_cast<bool *>(value) = static_cast<const CLASS_NAME*>(clientData)->GetFlipYCoord();
	}


};

#undef CLASS_NAME

#endif 