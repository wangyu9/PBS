// Intialize of deformers settings.
//Make sure that there is at most one active deformer at the same time.

#pragma once


//class DeformerPickingConfig
//{//This is not used !!
//public:
//	bool init_enable_deform_skinning;
//	bool init_enable_deform_locally_injective;
//	bool init_enable_deform_vega;
//	bool init_enable_deform_phys;
//	bool connect_lim_to_skinning;
//	bool connect_vega_to_skinning;
//	bool connect_phys_to_skinning;
//	bool init_connect_lim_to_skinning;
//	bool init_connect_vega_to_skinning;
//	bool init_connect_phys_to_skinning;
//	bool connect_picking_to_skinning;
//	DeformerPickingConfig();
//	DeformerPickingConfig(	
//		bool init_enable_deform_skinning,
//		bool init_enable_deform_locally_injective,
//		bool init_enable_deform_vega,
//		bool init_enable_deform_phys,
//		bool connect_lim_to_skinning,
//		bool connect_vega_to_skinning,
//		bool connect_phys_to_skinning);
//};


class TestClass 
{
public:
	static void StaticFunction();
};

#include <viewer/CommandLineBase.h>

class DeformerPicking : public CommandLineBase
{
public:

	DeformerPicking();
	~DeformerPicking();

	static DeformerPicking& GetReference();

	void init(Preview3D *preview);

	void restart();
	static void TW_CALL restart(void *clientData);

	void start_with_command(const std::string fname);

	void load_all();
	void load_all2();
	void load_all3();
	void load_all4();
	void load_all5();
	void load_all6();
	void load_all7();
	void load_all8();
	void load_all9();
	static void TW_CALL load_all(void *clientData);
	static void TW_CALL load_all2(void *clientData);
	static void TW_CALL load_all3(void *clientData);
	static void TW_CALL load_all4(void *clientData);
	static void TW_CALL load_all5(void *clientData);
	static void TW_CALL load_all6(void *clientData);
	static void TW_CALL load_all7(void *clientData);
	static void TW_CALL load_all8(void *clientData);
	static void TW_CALL load_all9(void *clientData);



protected:


	// Pointer to the tweak bar
	//igl::ReTwBar* bar; // wangyu moved this to PreviewPlugin

	//static TW_CALL void 

	//
	bool test;
public:
	bool init_enable_deform_skinning;
	bool init_enable_deform_locally_injective;
	bool init_enable_deform_vega;
	bool init_enable_deform_phys;
	bool connect_lim_to_skinning;
	bool connect_vega_to_skinning;
	bool connect_phys_to_skinning;
	bool init_connect_lim_to_skinning;
	bool init_connect_vega_to_skinning;
	bool init_connect_phys_to_skinning;
	bool connect_picking_to_skinning;
	bool import_handles_in_picking;
	bool with_subspace_arap;
};


//static DeformerPickingConfig deformerPickingConfig;

//#define 
static bool INIT_ENABLE_DEFORM_SKINNING = true;// DeformerPicking::GetReference().init_enable_deform_skinning;
//#define 
static bool INIT_ENABLE_DEFORM_LOCALLY_INJECTIVE = false;// DeformerPicking::GetReference().init_enable_deform_locally_injective;
//#define 
static bool INIT_ENABLE_DEFORM_VEGA = false;// DeformerPicking::GetReference().init_enable_deform_vega;

static bool INIT_ENABLE_DEFORM_PHYS = true;// DeformerPicking::GetReference().init_enable_deform_phys;

//#define 
static bool CONNECT_LIM_TO_SKINNING = false;// DeformerPicking::GetReference().connect_lim_to_skinning;
//(DeformLocallyInjective::GetReference().connected_to_skinning)
//#define 
static bool CONNECT_VEGA_TO_SKINNING = false;// DeformerPicking::GetReference().connect_vega_to_skinning;
//(DeformVega::GetReference().connected_to_skinning)
static bool CONNECT_PHYS_TO_SKINNING = true;// DeformerPicking::GetReference().connect_phys_to_skinning;


//#define 
static bool INIT_CONNECT_LIM_TO_SKINNING = false;// DeformerPicking::GetReference().connect_lim_to_skinning;
//#define 
static bool INIT_CONNECT_VEGA_TO_SKINNING = false;// DeformerPicking::GetReference().connect_vega_to_skinning;

static bool INIT_CONNECT_PHYS_TO_SKINNING = true;// DeformerPicking::GetReference().connect_phys_to_skinning;

//This should be true if any of the CONNECT_LIM_TO_SKINNING or CONNECT_VEGA_TO_SKINNING are true
//#define 
static bool CONNECT_PICKING_TO_SKINNING = true;// DeformerPicking::GetReference().connect_picking_to_skinning;

static bool IMPORT_HANDLES_IN_PICKING = true; // DeformerPicking::GetReference().import_handles_in_picking;

static bool WITH_SUBSPACE_ARAP = true; // DeformerPicking::GetReference().with_subspace_arap;

void setDeformerPicking(char *type);
