#ifndef DEFORM_DIRECT_H
#define DEFORM_DIRECT_H

#include <viewer\DeformDirectBase.h>

class DeformDirectUI : public DeformDirectBaseUI
{
public:
	static DeformDirectUI& GetReference();

	void init(Preview3D *preview);

	// External Calls:

	virtual void WriteExternal(const std::vector<int>& selected, const std::vector<Eigen::MatrixXd>& trans);

	void InitExternalCallFromHandle();
	void InitFromActiveHandleStruct();
	bool InitFromControlStruct(const char* fname);
	bool AddFromControlStruct(const char* fname);
	bool InitOrAddFromControlStruct(const char* fname, bool isInit);// true for init, false for add

	// AntTweakBar
	static void TW_CALL dialog_init_from_handle_struct(void *clientData);
	static void TW_CALL dialog_init_from_control_struct_file(void *clientData);
	static void TW_CALL dialog_init_control_struct_from_active(void *clientData);
	static void TW_CALL dialog_add_from_control_struct_file(void *clientData);

	bool commandLine(std::string c, std::vector<std::string> cl);
};

#endif /*DEFORM_DIRECT_H*/