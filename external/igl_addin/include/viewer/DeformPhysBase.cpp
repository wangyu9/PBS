#ifdef __APPLE__
#   include <OpenGL/gl.h>
#   include <OpenGL/glu.h>
#   include <GLUT/glut.h>
#else
#   ifdef _WIN32
#       include <windows.h>
#       include <GL/glew.h>
#       include <GL/glut.h>
#   endif
#   include <GL/gl.h>
#   include <GL/glu.h>
#endif

#include <unsupported/Eigen/OpenGLSupport>

#include "DeformPhysBase.h"

#include "FileDialog.h"
#include "PluginManager.h"
#include "./plugins/PickingPlugin.h"

//#include <FAST/skinning/draw_point.h>
#include <draw_point.h>

#include <viewer/path_anttweak.h>

#ifdef USING_IGL_HEADER_ONLY_MODE
#define IGL_HEADER_ONLY 
#endif 

#include "igl/tetgen/tetrahedralize.h"
//#include "igl/boundary_faces.h"//old igl version
#include <igl/boundary_facets.h>
#include <igl/readDMAT.h>




#include "math_helper.h"

using namespace std;
//using namespace Eigen;





DeformPhysBase::DeformPhysBase()
{
  

  isMeshLoaded = false;
  runSolver= false;
  isTetMesh = false;
  Dim = 2;

  restart_solver = true;
  with_dynamics = false;
  energy = 0;

  arap_iteration = 30;//5;
  phys_iteration = 1;

  time_step = 1;
  mu = 1.;
  rho = 1.;

  add_gravity = false;
  gravity_factor = 1e-2;
  gravity_direction[0] = 0;
  gravity_direction[1] = -1;
  gravity_direction[2] = 0;

  damping = 1.0; // 1.0 means no damping, 

  with_subspace_ARAP = WITH_SUBSPACE_ARAP;

  numElements = 0;

  num_of_fps_cluster = 200;

  isARAP = true;
  fakeUpdate = false;

  ARAP_rotation_group;
  ARAP_per_group_weight;
};

DeformPhysBase::~DeformPhysBase()
{

};

bool DeformPhysBase::UpdateConstraint(const LinearConstraint23d& lc)
{
	int dim = Variables.cols();

	if (dim != Dim)
	{
		printf("Warning: Constraint dimension does not match, constrain not set!");
		return false;
	}
		
	if (dim == 2)
	{
		UpdateConstraint(lc.LC2d.known, lc.LC2d.knownValue, lc.LC2d.Meq, lc.LC2d.Peq);
	}
	else
	{
		//assert(dim == 3);
		UpdateConstraint(lc.LC3d.known, lc.LC3d.knownValue, lc.LC3d.Meq, lc.LC3d.Peq);
	}
	return true;
}

bool DeformPhysBase::UpdateConstraint(const Eigen::VectorXi& known, const Eigen::MatrixXd& knownPos, const Eigen::MatrixXd& Meq, const Eigen::MatrixXd& Peq)
{

	int dim = Variables.cols();

	if (dim != Dim)
	{
		printf("Warning: Constraint dimension does not match, constrain not set!");
		return false;
	}

	//if (Meq.cols() != S.rows())
	//{
	//	printf("Warning: constraint is not set in UpdateConstraint()");
	//	return;
	//}

	S.setConstant(-1);
	for(int i=0; i<known.rows(); i++)
	{
		int idx = known(i);
		for (int d=0; d<dim+1; d++)
		{
			if(idx>=S.rows())
			{
				printf("Error: Constraint in Solver is not set since not initialized yet.\n");
				return false;
			}
			S(idx) = 1;
			for(int d=0;d<dim;d++)
			{
				Variables(idx,d) = knownPos.coeff(i,d);
			}
		}
	}

	//Eigen::MatrixXd Meq_eff;
	//Eigen::MatrixXd Peq_eff;
	//if (dim==2&&DeformSkinning::GetReference().getSkinningType()==LBS)
	//{
	//	extract_2d_m_from_3d(Meq, Meq_eff);
	//	Peq_eff = Peq.leftCols<2>();
	//}
	//else
	//{
	//	Meq_eff = Meq;
	//	Peq_eff = Peq;
	//}

	//geometrySolver.setConstraints(known, knownPos, Meq, Peq);
	geometrySolver.setConstraintsIndicator(S, Variables, Meq, Peq);

	return true;
}

bool DeformPhysBase::initEntireMesh(Eigen::MatrixXd VV, Eigen::MatrixXi TT, Eigen::MatrixXi FF)
{

	entire_mesh_vertices = VV;
	entire_mesh_tets = TT;
	entire_mesh_faces = FF;

	isMeshLoaded = entire_mesh_vertices.rows() > 0;

	if (isMeshLoaded)
	{
		if (entire_mesh_vertices.cols() >= 3 && entire_mesh_vertices.col(2).maxCoeff() - entire_mesh_vertices.col(2).minCoeff() > 1e-6)//(entire_mesh_tets.rows() != 0)
		{
			// 3D
			Dim = 3;
		}
		else
		{
			// 2D
			Dim = 2;
			Eigen::MatrixXd effective_vertices = entire_mesh_vertices.leftCols(2);
			entire_mesh_vertices = effective_vertices;// Cannot directly assign value to entire_mesh_vertices This is important!
		}

		if (entire_mesh_tets.rows() != 0)
		{
			isTetMesh = true;
		}
		else
		{
			isTetMesh = false;
		}
	}

	return isMeshLoaded;
}

void DeformPhysBase::initVariables(const Eigen::MatrixXd& Var, const Eigen::MatrixXd& subspace_bases)
{
	
	Variables = Var;
	Bases = subspace_bases;

	if (with_subspace_ARAP)
	{
		//Here V,S,T,F are for the subspace varible mesh
		// DO not need T,F

		//Init Solver here
		S.resize(Variables.rows(),1);
		S.setConstant(-1);

		printf("S is set to the dimension of %d.\n", S.rows());

		if(isTetMesh)
		{
			TF = T;
		}
		else
		{
			TF = Faces;
		}
		restart_solver = true;
		//restartSolver(V,TF,S,with_subspace_ARAP,entire_mesh_vertices,entire_mesh_faces,entire_mesh_tets);
	} 
	else
	{
		//Init Solver here
		S.resize(Variables.rows(),1);
		S.setConstant(-1);
		//S(0) = 1;
		//S(1) = 1;
		//S(2) = 1;
		//S(3) = 1;

		if(isTetMesh)
		{
			TF = T;
		}
		else
		{
			TF = Faces;
		}
		restart_solver = true;
		//restartSolver(V,TF,S,with_subspace_ARAP,entire_mesh_vertices,entire_mesh_faces,entire_mesh_tets);
	}
}

bool DeformPhysBase::restartSolver()
{

#ifdef USE_IGL_IMPLEMENTATION

	//using namespace Eigen;

	Eigen::VectorXi b(num_in_selection(S));
	assert(S.rows() == V.rows());
	Eigen::MatrixXd bc = Eigen::MatrixXd::Zero(b.size(),S.maxCoeff()+1);
	// get b from S
	{
		int bi = 0;
		for(int v = 0;v<S.rows(); v++)
		{
			if(S(v) >= 0)
			{
				b(bi) = v;
				bc(bi,S(v)) = 1;
				bi++;
			}
		}
	}
	// Store current mesh
	//U = V;
	Eigen::VectorXi _S;
	Eigen::VectorXd _D;
	Eigen::MatrixXd W;
	if(!igl::harmonic(V,F,b,bc,1,W))
	{
		return false;
	}
	igl::partition(W,100,arap_data.G,_S,_D);
	arap_data.with_dynamics = with_dynamics;
	arap_data.max_iter = max_iteration;
	arap_data.h = time_step;
	if(add_gravity)
	{
		f_ext.resize(V.rows(),3);
		//f_ext.setConstant(gravity_factor);
		Eigen::MatrixXd ones(V.rows(),1);
		ones.setOnes();
		f_ext.col(0).array() = gravity_direction[0]*gravity_factor*ones;
		f_ext.col(1).array() = gravity_direction[1]*gravity_factor*ones;
		f_ext.col(2).array() = gravity_direction[2]*gravity_factor*ones;
		//f_ext.setOnes();
		arap_data.f_ext = f_ext;
	}
	igl::arap_precomputation(V,F,b,arap_data);

	assert(with_subspace_ARAP==false&&"This is not supported yet!\n");

#else
#ifdef USE_PHYSICS_SOLVER
	physicsSolver.Init(F,V);
	physicsSolver.precomputation();
#else

	
	ARAP_para ap;

	int dim = Variables.cols();

	ap.var = Variables;

	ap.with_subspace_ARAP = with_subspace_ARAP;
	if (with_subspace_ARAP)
	{
		//Eigen::MatrixXd ARAP_M;

		//GetSubspaceBases(dim, ARAP_M);
	
		ap.subspace_M = Bases;
		ap.rotation_group = ARAP_rotation_group;
	}

	ap.dim = dim;

	ap.with_dynamics = with_dynamics;

	ap.mesh = ARAP_mesh(entire_mesh_vertices, entire_mesh_tets, entire_mesh_faces);

	ap.phys_para = ARAP_phys_para(time_step, mu, rho);

	ap.per_group_weight = ARAP_per_group_weight;

	geometrySolver.init(ap);

#endif

#endif

	restart_solver = false;
	return true;
}


// initialization (runs every time a mesh is loaded or cleared)



void DeformPhysBase::get_rotation_cluster(Eigen::VectorXi& RG) const
{
	RG = ARAP_rotation_group;
}

Eigen::VectorXi DeformPhysBase::get_rotation_cluster() const
{
	return ARAP_rotation_group;
}

void DeformPhysBase::set_rotation_cluster(const Eigen::VectorXi& new_rg)
{
	ARAP_rotation_group = new_rg;
}

void DeformPhysBase::set_per_group_weight(const Eigen::VectorXd& nw)
{
	ARAP_per_group_weight = nw;
}

void DeformPhysBase::update()
{
	if (isMeshLoaded && runSolver)
	{
		if (restart_solver)
			restartSolver();

		if (fakeUpdate)
		{
			//GetSubspaceBases(Dim, Variables);
			energy = geometrySolver.fakeUpdate(isARAP, Variables);
			//fake_update_solver(Variables, isARAP);
		}
		else
		{// update
			double gravity[3];
			for (int c = 0; c < 3; c++)
			{
				if (add_gravity)
				{
					gravity[c] = gravity_direction[c] * gravity_factor;
				}
				else
				{
					gravity[c] = 0.;
				}
			}

			energy = geometrySolver.update(arap_iteration, phys_iteration, isARAP, gravity, damping);
			if (with_subspace_ARAP)
			{
				Variables = geometrySolver.getVar();
			}
			else
			{
				Variables = geometrySolver.getV();
			}

			//*mesh->DeformedVertices = U;
			//(*import_vertices).leftCols(dim) = U;
				
		}
	}
}













DeformPhysBaseUI::DeformPhysBaseUI():DeformPhysBase()
{
	//check in with the manager
	PluginManager().register_plugin(this);

	bar = NULL;

}

DeformPhysBaseUI::~DeformPhysBaseUI()
{
	// Inverse order to call deconstructor function
	delete serializer;
	DeformPhysBase::~DeformPhysBase();
}

// initialization (runs every time a mesh is loaded or cleared)
void DeformPhysBaseUI::init(Preview3D *preview)
{
	PreviewPlugin::init(preview);

	// init menu bar
	if(bar == NULL)
	{
		serializer = new igl::XMLSerializer("Phys");

		// Create a tweak bar
		bar = new igl::ReTwBar;
		bar->TwNewBar("Phys");
		TwDefine(" Phys size='250 500' color='76 76 127' position='235 580' label='Phys Deformation' "); // change default tweak bar size and color
		bar->TwAddVarRO("#Elements", TW_TYPE_INT32, &numElements, "readonly=true");

		bar->TwAddVarRW("RunSolver", TW_TYPE_BOOLCPP, &runSolver, "label='Run Solver'");

		//bar->TwAddVarRW("With Dynamics", TW_TYPE_BOOLCPP, &with_dynamics, "label='Dynamic or Quasi-static'");
		//bar->TwAddVarRW("Add Gravity", TW_TYPE_BOOLCPP, &add_gravity, "label='Add Gravity'");

		//bar->TwAddVarRW("ARAP Iterations", TW_TYPE_INT32, &arap_iteration, "min=1 step=1");
		//bar->TwAddVarRW("ARAP/ASAP", TW_TYPE_BOOLCPP, &isARAP, " label='true for ARAP'");
		//bar->TwAddVarRW("fake update", TW_TYPE_BOOLCPP, &fakeUpdate, " label='true for fake update'");
		//bar->TwAddVarRW("Energy", TW_TYPE_DOUBLE, &energy, "");

		//bar->TwAddVarRW("Time Step", TW_TYPE_DOUBLE, &time_step, "group='Dynamics' min=0.0000000001");
		//bar->TwAddVarRW("Phys Iterations", TW_TYPE_INT32, &phys_iteration, "group='Dynamics' min=1 step=1");
		//bar->TwAddVarRW("Mu", TW_TYPE_DOUBLE, &mu, "group='Dynamics' min=0.0000000001");
		//bar->TwAddVarRW("Rho", TW_TYPE_DOUBLE, &rho, "group='Dynamics' min=0.0000000001");

		bar->TwAddVarCB("With Dynamics", TW_TYPE_BOOLCPP, SetWithDynamicsCB, GetWithDynamicsCB, this, "label='Dynamic or Quasi-static' key='d'");

		bar->TwAddVarCB("ARAP Iterations", TW_TYPE_INT32, DIALOG_OF(set_arap_iteration), DIALOG_OF(get_arap_iteration), this, "min=1 step=1");
		bar->TwAddVarRW("ARAP/ASAP", TW_TYPE_BOOLCPP, &isARAP, " label='true for ARAP'");
		//bar->TwAddVarCB("ARAP/ASAP", TW_TYPE_BOOLCPP, DIALOG_OF(set_arap_iteration), DIALOG_OF(get_arap_iteration), this, " label='true for ARAP'");
		bar->TwAddVarRW("fake update", TW_TYPE_BOOLCPP, &fakeUpdate, " label='true for fake update'");
		//bar->TwAddVarCB("fake update", TW_TYPE_BOOLCPP, DIALOG_OF(set_arap_iteration), DIALOG_OF(get_arap_iteration), this, " label='true for fake update'");
		bar->TwAddVarCB("Energy", TW_TYPE_DOUBLE, DIALOG_OF(set_energy), DIALOG_OF(get_energy), this, "");

		bar->TwAddVarCB("Time Step", TW_TYPE_DOUBLE, DIALOG_OF(set_time_step), DIALOG_OF(get_time_step), this, "group='Dynamics' min=0.0000000001");
		bar->TwAddVarCB("Phys Iterations", TW_TYPE_INT32, DIALOG_OF(set_phys_iteration), DIALOG_OF(get_phys_iteration), this, "group='Dynamics' min=1 step=1");
		bar->TwAddVarCB("Mu", TW_TYPE_DOUBLE, DIALOG_OF(set_mu), DIALOG_OF(get_mu), this, "group='Dynamics' min=0.0000000001");
		bar->TwAddVarCB("Rho", TW_TYPE_DOUBLE, DIALOG_OF(set_rho), DIALOG_OF(get_rho), this, "group='Dynamics' min=0.0000000001");

		//bar->TwAddVarCB("Add Gravity", TW_TYPE_BOOLCPP, SetAddGravityCB, GetAddGravityCB, this, "label='Add Gravity'");
		bar->TwAddVarCB("Add Gravity", TW_TYPE_BOOLCPP, DIALOG_OF(set_add_gravity), DIALOG_OF(get_add_gravity), this, "label='Add Gravity'");

		bar->TwAddVarRW("Gravity Factor", TW_TYPE_DOUBLE, &gravity_factor, "label='Gravity Factor'");
		bar->TwAddVarRW( "Gravity Dir", TW_TYPE_DIR3F, &gravity_direction, 
			" label='Gravity direction' open help='Change the Gravity direction.' ");

		bar->TwAddVarRW("Damping Alpha", TW_TYPE_DOUBLE, &damping, "min=0.0 max=1.0");

		bar->TwAddButton("Restart Solver", RestartSolver, this, " label='Restart Solver'");


		// ---------------------- Vertex Group ------------------
		bar->TwAddButton("Load Vertex Group", open_dialog_vertex_group, this, " group='Subspace'");
		bar->TwAddButton("Draw Vertex Group", draw_vertex_group_to_mesh, this, " group='Subspace'");

		bar->TwAddButton("Load Per Group Weight", open_dialog_per_group_weight, this, " group='Subspace'");

		bar->TwAddVarRW("Num of FPS clusters", TW_TYPE_INT32, &num_of_fps_cluster, " group='Subspace'");

	}
}

bool DeformPhysBaseUI::Serialize(tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element)
{
	//serializer->Add(with_dynamics, "with_dynamics");
	//serializer->Add(max_iteration, "max_iteration");
	//serializer->Add(isARAP, "isARAP");
	//serializer->Add(fakeUpdate, "fakeUpdate");
	//serializer->Add(time_step, "time_step");
	//serializer->Add(mu, "mu");
	//serializer->Add(rho, "rho");
	//serializer->Add(add_gravity, "add_gravity");
	//serializer->Add(gravity_factor, "gravity_factor");

	// serialize previously added objects
	serializer->SaveToXMLDoc(doc);

  return igl::save_ReAntTweakBar(bar,doc);
}

bool DeformPhysBaseUI::Deserialize(tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element)
{
  return igl::load_ReAntTweakBar(bar,doc);
}

bool DeformPhysBaseUI::keyDownEvent(unsigned char key, int modifiers, int mouse_x, int mouse_y)
{
	return false;
}
  
//mouse callback
bool DeformPhysBaseUI::mouseDownEvent(int mouse_x, int mouse_y, int button, int modifiers)
{
  return false;
}

bool DeformPhysBaseUI::mouseUpEvent(int mouse_x, int mouse_y, int button, int modifiers)
{
  return false;
}

bool DeformPhysBaseUI::mouseMoveEvent(int mouse_x, int mouse_y)
{
  return false;
}

bool DeformPhysBaseUI::mouseScrollEvent(int mouse_x, int mouse_y, float delta)
{
  return false;
}
  
//stuff that is drawn by the plugin before the previewer has displayed the mesh
//first draw 3d, then 2d
void DeformPhysBaseUI::preDraw(int currentTime)
{
#ifdef DEBUG_PLUGINS_SEQUENTIAL_ORDER
	printf("DEBUG_PLUGINS_SEQUENTIAL_ORDER: DeformPhysBase update solver.\n");
#endif
	update();
}

//stuff that is drawn by the plugin after the previewer has displayed the mesh
//first draw 3d, then 2d
void DeformPhysBaseUI::postDraw(int currentTime)
{
}

/******** Vertex Group ***********/
void TW_CALL DeformPhysBaseUI::open_dialog_vertex_group(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_open_file_path(fname);

	if(fname[0] == 0)
		return;

	static_cast<DeformPhysBaseUI *>(clientData)->load_rotation_cluster_from_file(fname);
}

void TW_CALL DeformPhysBaseUI::open_dialog_per_group_weight(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_open_file_path(fname);

	if (fname[0] == 0)
		return;

	static_cast<DeformPhysBaseUI *>(clientData)->load_per_group_weight_from_file(fname);
}

void TW_CALL DeformPhysBaseUI::draw_vertex_group_to_mesh(void *clientData)
{
	static_cast<DeformPhysBaseUI *>(clientData)->draw_vertex_group_to_mesh();
}

#include <igl/rgb_to_hsv.h>
#include <igl/hsv_to_rgb.h>
#include <print_matlab.h>
void DeformPhysBaseUI::draw_vertex_group_to_mesh()
{
	Eigen::VectorXi RG = DeformPhysBase::get_rotation_cluster();

	if (false)
	{
		Eigen::VectorXd colors = RG.cast<double>() / (1. + (double)RG.maxCoeff());
		m_preview->GetMainMesh().draw_colors_on_mesh(colors);
	} 

	if (false)
	{
		int NC = RG.maxCoeff();

		Eigen::MatrixXd per_element_color(NC,3);

		// compute random selection color
		// find hue for current material color
		double hsv[3], rgb[3];
		rgb[0] = m_preview->material.g_MatDiffuse[0];
		rgb[1] = m_preview->material.g_MatDiffuse[1];
		rgb[2] = m_preview->material.g_MatDiffuse[2];
		igl::rgb_to_hsv(rgb, hsv);

		double golden_ratio_conjugate = 0.618033988749895;
		double curHue = 0;
		for (int i = 0; i < NC; i++)
		{
			double r, g, b;
			double hue = std::fmod(curHue, 1.0) * 360;

			// find color not similar to yellow
			while (std::abs(hue - hsv[0]) < 15)
			{
				curHue += golden_ratio_conjugate;
				hue = std::fmod(curHue, 1.0) * 360;
			}

			igl::hsv_to_rgb(hue, 0.8, 1.0, r, g, b);
			per_element_color.row(i) << r, g, b;
			curHue += golden_ratio_conjugate;
		}	
	}

	if (true)
	{
		Eigen::MatrixXi toMatlab = RG;
		print_matlab("ARAP_rotation_group", toMatlab);
	}

}

void DeformPhysBaseUI::load_rotation_cluster_from_file(const char* vertex_group_file_name)
{
	Eigen::VectorXi new_rg;
	igl::readDMAT(vertex_group_file_name, new_rg);
	printf("Load vertex group: (%d,1)\n", new_rg.rows());

	//if (vertex_group.rows()!=vertices->rows())
	//{
	//	printf("Vertex group size does not match!\n");
	//}

	set_rotation_cluster(new_rg);
}

void DeformPhysBaseUI::load_per_group_weight_from_file(const char* fname)
{
	Eigen::VectorXd nw;
	igl::readDMAT(fname, nw);

	set_per_group_weight(nw);
}

#include <farthest_point_sampling.h>
void DeformPhysBaseUI::set_FPS_rotation_cluster(int num_of_cluster)
{
	set_num_of_fps_cluster(num_of_cluster);
	set_FPS_rotation_cluster();
}

void DeformPhysBaseUI::set_FPS_rotation_cluster()
{
	Eigen::VectorXi new_rg;

	Eigen::MatrixXi TF;
	if (isTetMesh)
	{
		TF = m_preview->GetMainMesh().tets->cast<int>();
	}
	else
	{
		TF = m_preview->GetMainMesh().faces->cast<int>();
	}

	farthest_element_clustering(m_preview->GetMainMesh().rest_vertices, TF, get_num_of_fps_cluster(), new_rg);

	set_rotation_cluster(new_rg);
}