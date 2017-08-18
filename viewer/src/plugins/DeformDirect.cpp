#include "DeformDirect.h"
#include <igl/readDMAT.h>

#include "FileDialog.h"

static DeformDirectUI  DeformDirectBaseUIInstance = DeformDirectUI();
DeformDirectUI&  DeformDirectUIInstance_(){ return DeformDirectBaseUIInstance; }

DeformDirectUI& DeformDirectUI::GetReference()
{
	return DeformDirectBaseUIInstance;
}


void DeformDirectUI::WriteExternal(const std::vector<int>& selected, const std::vector<Eigen::MatrixXd>& trans)
{
	//int i = cuIndex;
	//for (int i=0; i<controlUnits.size(); i++)
	{

		//std::vector<int> selected;
		//std::vector<Eigen::MatrixXd> trans;

		HandlePlugin::GetReference().ExternalCallSetSelectedHandleIndices(selected);
		HandlePlugin::GetReference().ExternalCallSetSelecetdHandleTrans(trans);
	}

}

void DeformDirectUI::InitExternalCallFromHandle()
{
	const HandleStructure& hs = HandlePlugin::GetReference().GetHandleStructRef();
	DeformDirectBase::initFromHandle(hs);
}

void DeformDirectUI::InitFromActiveHandleStruct()
{
	const HandleStructure& hs = HandlePlugin::GetReference().GetHandleStructRef();
	DeformDirectBase::initControlStructActive(hs);
}

bool DeformDirectUI::AddFromControlStruct(const char* fname)
{
	return InitOrAddFromControlStruct(fname, false);
}

bool DeformDirectUI::InitFromControlStruct(const char* fname)
{
	return InitOrAddFromControlStruct(fname, true);
}

bool DeformDirectUI::InitOrAddFromControlStruct(const char* fname, bool isInit)
{
	const HandleStructure& hs = HandlePlugin::GetReference().GetHandleStructRef();
	Eigen::VectorXi cs;

	if (!igl::readDMAT(fname, cs) || cs.cols() != 1)
	{
		printf("Error: Unsuccessful in loading control struct file (%d,%d)!\n", cs.rows(), cs.cols());
		return false;
	}

	const int hsize = hs.all_handle_list.size();
	if (cs.rows() < hsize)
	{
		printf("Warning: the control struct file (%d) does not provide enough connecting info (%d), making rest handles free!\n", cs.rows(), hsize);
		Eigen::VectorXi tmp = cs;
		cs.resize(hsize, 1);
		cs.setConstant(-2);
		cs.block(0, 0, tmp.rows(), 1) = tmp;
	}

	if (isInit)
	{
		DeformDirectBase::initControlStruct(hs, cs);
	}
	else
	{
		DeformDirectBase::addControlStruct(hs, cs);
	}

	return true;
}


// AntTreakBar

void TW_CALL DeformDirectUI::dialog_init_from_handle_struct(void *clientData)
{
	static_cast<DeformDirectUI *>(clientData)->InitExternalCallFromHandle();
}

void TW_CALL DeformDirectUI::dialog_init_from_control_struct_file(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_open_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<DeformDirectUI *>(clientData)->InitFromControlStruct(fname);
}

void TW_CALL DeformDirectUI::dialog_add_from_control_struct_file(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_open_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<DeformDirectUI *>(clientData)->AddFromControlStruct(fname);
}

void TW_CALL DeformDirectUI::dialog_init_control_struct_from_active(void *clientData)
{
	static_cast<DeformDirectUI *>(clientData)->InitFromActiveHandleStruct();
}

void DeformDirectUI::init(Preview3D *preview)
{
	DeformDirectBaseUI::init(preview);

	bar->TwAddButton("Init from Handle Struct", dialog_init_from_handle_struct, this, " group='Control Struct'");
	bar->TwAddButton("Init from Control Struct File", dialog_init_from_control_struct_file, this, " group='Control Struct'");
	bar->TwAddButton("Init from Active Handle Struct", dialog_init_control_struct_from_active, this, " group='Control Struct'");
	bar->TwAddButton("Add from Control Struct File", dialog_add_from_control_struct_file, this, " group='Control Struct'");
}


#include <viewer/CommandLineBase.h>
bool CommandLine(DeformDirectUI& plugin, std::vector<std::string> cl)
{
	if (cl.size() < 1)
	{
		printf("Error: No Command Line for DeformSkinning.\n");
		return false;
	}
	else
	{
		printf("DeformDirect");
		print_out_argv(cl);
		printf("\n");
	}


	if (cl[0] == std::string("init_from_control_struct_file"))
	{
		if (cl.size() < 2)
		{
			printf("Error: No mesh file name for init_from_control_struct_file.\n");
			return false;
		}
		return plugin.InitFromControlStruct(cl[1].c_str());
	}
	else if (cl[0] == std::string("load_key_frame_config"))
	{
		if (cl.size() < 2)
		{
			printf("Error: No mesh file name for load_key_frame_config.\n");
			return false;
		}
		return plugin.load_key_frame_config(cl[1].c_str());
	}
	else
	{
		printf("Error: Unknown Command Line Type for Viewer.\n");
		return false;
	}
}

bool DeformDirectUI::commandLine(std::string c, std::vector<std::string> cl)
{
	if (c == std::string("DeformDirect"))
	{
		return CommandLine(*this, cl);
	}
	return false;
}