#include "DeformerPicking.h"

#include <viewer/path_anttweak.h>

#include "PluginManager.h"
#include "PickingPlugin.h"

#include "FileDialog.h"

// Declare the single entity of the plugin
// This should be in all the plugin headers

#include "Viewer.h"
#include "ViewerPlugin.h"

#include <plugins/DeformSkinning.h>
#include "plugins/DeformPhys.h"
#include "plugins/DeformDirect.h"
#include "plugins/HandlePlugin.h"
#include "plugins/PickingPlugin.h"

void TestClass::StaticFunction() { return; }

static DeformerPicking  DeformerPickingInstance;

DeformerPicking&  DeformerPickingInstance_(){ return DeformerPickingInstance; }

DeformerPicking& DeformerPicking::GetReference()
{
	return DeformerPickingInstance;
}

//DeformerPickingConfig::DeformerPickingConfig()
//{
//	// THIS IS NOT USED
//	init_enable_deform_skinning = false;
//	init_enable_deform_locally_injective = false;
//	init_enable_deform_vega = false;
//	init_enable_deform_phys = true;
//	connect_lim_to_skinning = false;
//	connect_vega_to_skinning = false;
//	connect_phys_to_skinning = false;
//	init_connect_lim_to_skinning = connect_lim_to_skinning;
//	init_connect_vega_to_skinning = connect_vega_to_skinning;
//	init_connect_phys_to_skinning = connect_phys_to_skinning;
//	connect_picking_to_skinning = connect_lim_to_skinning||connect_vega_to_skinning||connect_phys_to_skinning;
//}


DeformerPicking::DeformerPicking(): CommandLineBase()
{
	
	bool test = true;

	init_enable_deform_skinning = true;
	init_enable_deform_locally_injective = false;
	init_enable_deform_vega = false;
	init_enable_deform_phys = true;
	connect_lim_to_skinning = false;
	connect_vega_to_skinning = false;
	connect_phys_to_skinning = true;
	init_connect_lim_to_skinning = connect_lim_to_skinning;
	init_connect_vega_to_skinning = connect_vega_to_skinning;
	init_connect_phys_to_skinning = connect_phys_to_skinning;
	connect_picking_to_skinning = connect_lim_to_skinning||connect_vega_to_skinning||connect_phys_to_skinning;
	
	import_handles_in_picking = connect_picking_to_skinning;
	if(init_enable_deform_skinning==true&&init_enable_deform_phys==true
		&&connect_phys_to_skinning==true&&connect_picking_to_skinning==true)
	{
		with_subspace_arap = true;
		//if (false)//turn on or turn off
		//{
		//	import_handles_in_picking = false;//only do this for phys with subspace
		//}
	}
	
}

DeformerPicking::~DeformerPicking()
{

}




void DeformerPicking::init(Preview3D *preview)
{
	CommandLineBase::init(preview);
	if(bar != NULL)
	{
		bar->TwAddVarRW("Enable Skinning", TW_TYPE_BOOLCPP, &init_enable_deform_skinning, "label='Enable Skinning'");
		bar->TwAddVarRW("Enable LIM", TW_TYPE_BOOLCPP, &init_enable_deform_locally_injective, "label='Enable LIM'");
		bar->TwAddVarRW("Enable Vega", TW_TYPE_BOOLCPP, &init_enable_deform_vega, "label='Enable Vega'");
		bar->TwAddVarRW("Enable Phys", TW_TYPE_BOOLCPP, &init_enable_deform_phys, "label='Enable Phys'");
		bar->TwAddVarRW("Connect LIM to Skinning", TW_TYPE_BOOLCPP, &connect_lim_to_skinning, "label='Connect LIM to Skinning'");
		bar->TwAddVarRW("Connect Vega to Skinning", TW_TYPE_BOOLCPP, &connect_vega_to_skinning, "label='Connect Vega to Skinning'");
		bar->TwAddVarRW("Connect Phys to Skinning", TW_TYPE_BOOLCPP, &connect_phys_to_skinning, "label='Connect Phys to Skinning'");
		bar->TwAddButton("Restart", restart, this, " label='Restart'");

		
		bar->TwAddButton("Load All", load_all, this, "group='Fast Loading'");
		bar->TwAddButton("Load Mesh, W, H", load_all2, this, "group='Fast Loading'");
		bar->TwAddButton("Load ARAP subspace", load_all3,this, "group='Fast Loading'");
		bar->TwAddButton("Set ARAP subspace (FPS)", load_all6, this, "group='Fast Loading'");
		bar->TwAddButton("Load Mesh, W (file), H", load_all4, this, "group='Fast Loading'");
		bar->TwAddButton("Cage: Mesh, W, H", load_all5, this, "group='Fast Loading'");
	}
}

//DeformerPickingConfig::DeformerPickingConfig(	
//	bool init_enable_deform_skinning,
//	bool init_enable_deform_locally_injective,
//	bool init_enable_deform_vega,
//	bool init_enable_deform_phys,
//	bool connect_lim_to_skinning,
//	bool connect_vega_to_skinning,
//	bool connect_phys_to_skinning){}

void setDeformerPicking(char *type)
{

	INIT_ENABLE_DEFORM_SKINNING = true;
	INIT_ENABLE_DEFORM_LOCALLY_INJECTIVE = false;
	INIT_ENABLE_DEFORM_VEGA = false;
	CONNECT_LIM_TO_SKINNING = false;
	CONNECT_VEGA_TO_SKINNING = false;

	int i=0;
	while(type[i]!='\0'&&i<5)
	{
		switch (i)
		{
		case 0:
			{
				if(type[i]=='1')
				{
					INIT_ENABLE_DEFORM_SKINNING = true;
				}
				else
				{
					INIT_ENABLE_DEFORM_SKINNING = false;
				}
			}
			break;
		case 1:
			{
				if(type[i]=='1')
				{
					INIT_ENABLE_DEFORM_LOCALLY_INJECTIVE = true;
				}
				else
				{
					INIT_ENABLE_DEFORM_LOCALLY_INJECTIVE = false;
				}
			}
			break;
		case 2:
			{
				if(type[i]=='1')
				{
					INIT_ENABLE_DEFORM_VEGA = true;
				}
				else
				{
					INIT_ENABLE_DEFORM_VEGA = false;
				}
			}
			break;
		case 3:
			{
				if(type[i]=='1')
				{
					CONNECT_LIM_TO_SKINNING = true;
				}
				else
				{
					CONNECT_LIM_TO_SKINNING = false;
				}
			}
			break;
		case 4:
			{
				if(type[i]=='1')
				{
					CONNECT_VEGA_TO_SKINNING = true;
				}
				else
				{
					CONNECT_VEGA_TO_SKINNING = false;
				}
			}
			break;
		default:
			break;
		}
	}
	INIT_CONNECT_LIM_TO_SKINNING = CONNECT_LIM_TO_SKINNING;
	INIT_CONNECT_VEGA_TO_SKINNING = CONNECT_VEGA_TO_SKINNING;
	CONNECT_PICKING_TO_SKINNING = CONNECT_LIM_TO_SKINNING||CONNECT_VEGA_TO_SKINNING;

}

void TW_CALL DeformerPicking::restart(void *clientData)
{ 
	//Preview3D::restart();
	
	//for (unsigned int i = 0; i<PluginManager().plugin_list_.size(); ++i)
	//	PluginManager().plugin_list_[i]->init(this);
	static_cast<DeformerPicking *>(clientData)->restart();
}

void DeformerPicking::restart()
{
	m_preview->Restart();
}

void DeformerPicking::load_all(void *clientData)
{
	static_cast<DeformerPicking *> (clientData)->load_all();
}

void DeformerPicking::load_all2(void *clientData)
{
	static_cast<DeformerPicking *> (clientData)->load_all2();
}

void DeformerPicking::load_all3(void *clientData)
{
	static_cast<DeformerPicking *> (clientData)->load_all3();
}

void DeformerPicking::load_all4(void *clientData)
{
	static_cast<DeformerPicking *> (clientData)->load_all4();
}

void DeformerPicking::load_all5(void *clientData)
{
	static_cast<DeformerPicking *> (clientData)->load_all5();
}

void DeformerPicking::load_all6(void *clientData)
{
	static_cast<DeformerPicking *> (clientData)->load_all6();
}

void DeformerPicking::load_all7(void *clientData)
{
	static_cast<DeformerPicking *> (clientData)->load_all7();
}

void DeformerPicking::load_all8(void *clientData)
{
	static_cast<DeformerPicking *> (clientData)->load_all8();
}

void DeformerPicking::load_all9(void *clientData)
{
	static_cast<DeformerPicking *> (clientData)->load_all9();
}

void DeformerPicking::load_all()
{
	//char fname[256] = "C:\\WorkSpace\\Visual Studio 2010\\PBS_project\\test\\arma2\\mesh.mesh";
	//m_preview->load_mesh_from_file(fname);
	//m_preview->load_vertex_group("C:\\WorkSpace\\Visual Studio 2010\\PBS_project\\test\\arma2\\T_cluster.dmat");
	//DeformSkinning::GetReference().load_weights_from_file("C:\\WorkSpace\\Visual Studio 2010\\PBS_project\\test\\arma2\\W_bc.dmat");
	//HandlePlugin::GetReference().load_handle_mesh_from_file("C:\\WorkSpace\\Visual Studio 2010\\PBS_project\\test\\arma2\\output_handle_mesh.mesh");
	//HandlePlugin::GetReference().MoveHandlesOntoMesh();
	////DeformSkinning::GetReference().reinforceSparseWeights();
	//DeformPhys::GetReference().prepare_connect();
	//PickingPlugin::GetReference().loadSelection("C:\\WorkSpace\\Visual Studio 2010\\PBS_project\\test\\arma2\\selection");
	//DeformPhys::GetReference().runSolver = true;

	load_all2();
	load_all3();
}


inline bool path_from_dir(char* fname, std::string& path)
{// Note: the last \ is not included in the string.
	fname[0] = 0;
	get_open_file_path(fname);

	if (fname[0] == 0)
	{
		printf("No Path is give.\n");
		return false;
	}

	std::string file_name_string = std::string(fname);
	size_t last_slash = file_name_string.rfind('\\');
	if (last_slash == std::string::npos)
	{
		// No file type determined
		printf("Error: No file path found in %s\n", fname);
		return false;
	}
	path = file_name_string.substr(0, last_slash);

	return true;
}


void DeformerPicking::start_with_command(const std::string fname)
{
	read_command_file(fname);
}

void DeformerPicking::load_all2()
{
	char mesh_name[2048];
	std::string folder_path;
	std::string file_name_string;

	if(!path_from_dir(mesh_name, folder_path))
		return;

	m_preview->load_mesh_from_file(mesh_name);
	HandlePlugin::GetReference().load_handles_from_file((folder_path+"\\"+"H").c_str());
	//HandlePlugin::GetReference().sample_handles();
	HandlePlugin::GetReference().MoveHandlesOntoMesh();
	DeformSkinning::GetReference().load_weights_from_MATLAB();

	//char fname[256] = "C:\\WorkSpace\\Visual Studio 2010\\PBS_project\\test\\arma2\\mesh.mesh";
	//m_preview->load_mesh_from_file(fname);
	//DeformSkinning::GetReference().load_weights_from_file("C:\\WorkSpace\\Visual Studio 2010\\PBS_project\\test\\arma2\\W_bc.dmat");
	//HandlePlugin::GetReference().load_handle_mesh_from_file("C:\\WorkSpace\\Visual Studio 2010\\PBS_project\\test\\arma2\\output_handle_mesh.mesh");
	//HandlePlugin::GetReference().MoveHandlesOntoMesh();
	////DeformSkinning::GetReference().reinforceSparseWeights();
}

void DeformerPicking::load_all3()
{
	char file_name[2048];
	std::string folder_path;
	std::string file_name_string;

	if(!path_from_dir(file_name, folder_path))
		return;

	//char file_name[2048];
	//get_open_file_path(file_name);
	//if(file_name[0]==0)
	//	return;

	DeformPhysUI::GetReference().load_rotation_cluster_from_file(file_name);
	DeformPhysUI::GetReference().prepare_connect();
	HandlePlugin::GetReference().load_active( (folder_path+"\\"+"active").c_str() );
	HandlePlugin::GetReference().set_active_handles();
	DeformPhysUI::GetReference().runSolver = true;

	//m_preview->load_vertex_group("C:\\WorkSpace\\Visual Studio 2010\\PBS_project\\test\\arma2\\T_cluster.dmat");
	//DeformPhys::GetReference().prepare_connect();
	//HandlePlugin::GetReference().set_active_handles();
	////PickingPlugin::GetReference().loadSelection("C:\\WorkSpace\\Visual Studio 2010\\PBS_project\\test\\arma2\\selection");
	//DeformPhys::GetReference().runSolver = true;
	// bar->TwAddVarCB("Connect to Skinning",TW_TYPE_BOOLCPP,SetConnectSkinningCB,GetConnectSkinningCB,this," label='Connect SKinning (Init)'");
	// bar->TwAddButton("Connect Skinning (Init)", prepare_dialog, this," label='Connect Skinning (Init)'");
}

void DeformerPicking::load_all4()
{
	char mesh_name[2048];
	std::string folder_path;
	std::string file_name_string;

	if(!path_from_dir(mesh_name, folder_path))
		return;

	m_preview->load_mesh_from_file(mesh_name);
	HandlePlugin::GetReference().load_handles_from_file((folder_path+"\\"+"H").c_str());
	//HandlePlugin::GetReference().sample_handles();
	HandlePlugin::GetReference().MoveHandlesOntoMesh();
	//DeformSkinning::GetReference().load_weights_from_MATLAB();
	char fname[2048];	fname[0] = 0;
	get_open_file_path(fname);
	if(fname[0] == 0)	return;
	DeformSkinning::GetReference().load_weights_from_file(fname);
}

void DeformerPicking::load_all5()
{
	char mesh_name[2048];
	std::string folder_path;
	std::string file_name_string;

	if(!path_from_dir(mesh_name, folder_path))
		return;

	m_preview->load_mesh_from_file(mesh_name);
	char fname[2048];	fname[0] = 0;
	get_open_file_path(fname);
	if(fname[0] == 0)	return;
	HandlePlugin::GetReference().load_handle_mesh_from_file(fname);
	// No need to move nearest: HandlePlugin::GetReference().MoveHandlesOntoMesh();
	fname[0] = 0;
	get_open_file_path(fname);
	if(fname[0] == 0)	return;
	DeformSkinning::GetReference().load_weights_from_file(fname);
}

void DeformerPicking::load_all6()
{
	//char file_name[2048];
	//std::string folder_path;
	//std::string file_name_string;

	//if (!path_from_dir(file_name, folder_path))
	//	return;

	DeformPhysUI::GetReference().set_FPS_rotation_cluster();
	DeformPhysUI::GetReference().prepare_connect();
	//HandlePlugin::GetReference().load_active((folder_path + "\\" + "active").c_str());
	HandlePlugin::GetReference().set_active_handles();
	DeformPhysUI::GetReference().runSolver = true;
}

void DeformerPicking::load_all7()
{
	
}

void DeformerPicking::load_all8()
{

}

void DeformerPicking::load_all9()
{

}





