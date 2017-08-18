#ifndef DEFORM_PHYS_BASE_H
#define DEFORM_PHYS_BASE_H

#include "ViewerPlugin.h"
#include "./plugins/DeformerPicking.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <linear_constraint.h>


//#define USE_IGL_IMPLEMENTATION
//#define USE_PHYSICS_SOLVER
#ifdef USE_IGL_IMPLEMENTATION
// Used in solver
#include <igl/svd3x3/arap.h>
#include <igl/partition.h>
#include <igl/harmonic.h>
#else
#ifdef USE_PHYSICS_SOLVER
#include "physicsSolver.h"
#else
#include "geometrySolver.h"
#endif
#endif

//#ifdef USE_IGL_IMPLEMENTATION
//igl::ARAPData arap_data;
//#else
//#ifdef USE_PHYSICS_SOLVER
//PhysicsSolver physicsSolver;
//#else
//GeometrySolver geometrySolver;
//#endif
//#endif

class PhysBase
{
private:

};

#include <viewer/anttweak_helper.h>

class DeformPhysBase: public PhysBase
{

protected:

	//virtual void SetToOutput(const int dim, const Eigen::MatrixXd& Var) = 0;
	//virtual void GetSubspaceVar(const int dim, Eigen::MatrixXd& Var) = 0;
	//virtual void GetSubspaceBases(const int dim, Eigen::MatrixXd& M) = 0;

public:

	DeformPhysBase();
	~DeformPhysBase();

	// initialization (runs every time a mesh is loaded or cleared)
	// Init entire mesh first, if successful then init variables
	bool initEntireMesh(Eigen::MatrixXd VV, Eigen::MatrixXi TT, Eigen::MatrixXi FF);
	void initVariables(const Eigen::MatrixXd& Var, const Eigen::MatrixXd& subspace_bases);
	
	bool UpdateConstraint(const LinearConstraint23d& lc);
	bool UpdateConstraint(const Eigen::VectorXi& known, const Eigen::MatrixXd& knownPos, const Eigen::MatrixXd& Meq, const Eigen::MatrixXd& Peq);

	bool runSolver;

	void update();

	// to ways to restart solver: do it immediately or do it when the next update() is called.
	bool restartSolver();
	bool restart_solver;


	NormalUpdateMethod normalUpdateMethod;
	
public:
	// the following variables could be changed and the changed will be reflected in the next update()

	double gravity_factor;
	double damping;
	float gravity_direction[3];

	int arap_iteration;
	int phys_iteration;

	bool isARAP;
	bool fakeUpdate; // This means only display the energy of current configuration, no update.

	// The following variables will be used only when fps clustering function is called.

	int num_of_fps_cluster;

	// The following variables are read only, do not try to modify their values at run time externally.
	// Consider making then invisiable from the outside.

	int Dim;
	int numElements;
	bool isMeshLoaded;
	bool isTetMesh;
	double energy;

	// Not clear what to do with this one.

	Eigen::MatrixXd f_ext; 

private:
	
	GeometrySolver geometrySolver;

	// This is used for subspace ARAP.
	bool with_subspace_ARAP;
	Eigen::MatrixXd entire_mesh_vertices;
	Eigen::MatrixXi entire_mesh_faces;
	Eigen::MatrixXi entire_mesh_tets;
	Eigen::VectorXi ARAP_rotation_group;
	Eigen::VectorXd ARAP_per_group_weight;

	bool with_dynamics;
	double time_step;
	double mu;
	double rho;
	bool add_gravity;

	Eigen::VectorXi S;// S has the same rows as Variables does, indicating which rows of Variables are being constrained.

	Eigen::MatrixXd Bases;
	Eigen::MatrixXd Variables;

	Eigen::MatrixXi T;
	Eigen::MatrixXi Faces;

	Eigen::MatrixXi TF;//tets or Faces: depends on 3D (tets) or 2D (faces)
	

public:

	// Get and Set 

	int get_dim() const
	{
		return Dim;
	}

	const Eigen::MatrixXd& get_variables() const
	{
		return Variables;
	}
	void set_variables(const Eigen::MatrixXd& variables)
	{
		this->Variables = variables;
	}

	void get_rotation_cluster(Eigen::VectorXi& RG) const;
	Eigen::VectorXi get_rotation_cluster() const;
	void set_rotation_cluster(const Eigen::VectorXi& new_rg);

	void set_per_group_weight(const Eigen::VectorXd& nw);
	// TODO add get_per_group_weight not really need right now.

	void set_num_of_fps_cluster(int nc)
	{
		num_of_fps_cluster = nc;
	}
	int get_num_of_fps_cluster() const
	{
		return num_of_fps_cluster;
	}

	void set_add_gravity(bool set_gravity)
	{
		add_gravity = set_gravity;
		restart_solver = true;
	}
	bool get_add_gravity() const
	{
		return add_gravity;
	}

	void set_arap_iteration(int it)
	{
		arap_iteration = it;
	}
	int get_arap_iteration() const
	{
		return arap_iteration;
	}

	void set_phys_iteration(int it)
	{
		phys_iteration = it;
	}
	int get_phys_iteration() const
	{
		return phys_iteration;
	}

	void set_with_dynamics(bool set_dynamics)
	{
		with_dynamics = set_dynamics;
		restart_solver = true;
	}
	bool get_with_dynamics() const
	{
		return with_dynamics;
	}

	double get_time_step() const
	{
		return time_step;
	}
	void set_time_step(double time_step)
	{
		this->time_step = time_step;
	}

	void set_mu(double mu)
	{
		this->mu = mu;
	}
	double get_mu() const
	{
		return mu;
	}

	void set_rho(double rho)
	{
		this->rho = rho;
	}
	double get_rho() const
	{
		return rho;
	}

	void set_energy(double energy)
	{
		this->energy = energy;
	}
	double get_energy() const
	{
		return energy;
	}

};

class DeformPhysBaseUI: public DeformPhysBase,  public PreviewPlugin
{

public:

	DeformPhysBaseUI();
	~DeformPhysBaseUI();

	// initialization (runs every time a mesh is loaded or cleared)
	void init(Preview3D *preview);

	igl::XMLSerializer* serializer;
	// implement Serializable interface
	bool Serialize(tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element);
	bool Deserialize(tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element);

	// keyboard callback
	bool keyDownEvent(unsigned char key, int modifiers, int mouse_x, int mouse_y) ;

	//mouse callback
	bool mouseDownEvent(int mouse_x, int mouse_y, int button, int modifiers);
	bool mouseUpEvent(int mouse_x, int mouse_y, int button, int modifiers);
	bool mouseMoveEvent(int mouse_x, int mouse_y);
	bool mouseScrollEvent(int mouse_x, int mouse_y, float delta);

	//stuff that is drawn by the plugin before the previewer has displayed the mesh
	//first draw 3d, then 2d
	void preDraw(int currentTime);
	//stuff that is drawn by the plugin after the previewer has displayed the mesh
	//first draw 3d, then 2d
	void postDraw(int currentTime);


protected:

	// Pointer to the tweak bar
	//igl::ReTwBar* bar;//wangyu moved to PreviewPlugin

protected:

	void draw_vertex_group_to_mesh();

	static void TW_CALL RestartSolver(void *clientData)
	{
		static_cast<DeformPhysBaseUI*>(clientData)->restart_solver = true;
	}
	static void TW_CALL open_dialog_vertex_group(void *clientData);
	static void TW_CALL open_dialog_per_group_weight(void *clientData);
	static void TW_CALL draw_vertex_group_to_mesh(void *clientData);


	static void TW_CALL SetWithDynamicsCB(const void *value, void *clientData)
	{
		static_cast<DeformPhysBaseUI*>(clientData)->set_with_dynamics(*static_cast<const bool *>(value));
	}
	static void TW_CALL GetWithDynamicsCB(void *value, void *clientData)
	{
		*static_cast<bool *>(value) = static_cast<const DeformPhysBaseUI*>(clientData)->get_with_dynamics();
	}

	static void TW_CALL SetAddGravityCB(const void *value, void *clientData)
	{
		static_cast<DeformPhysBaseUI*>(clientData)->set_add_gravity(*static_cast<const bool *>(value));
	}
	static void TW_CALL GetAddGravityCB(void *value, void *clientData)
	{
		*static_cast<bool *>(value) = static_cast<const DeformPhysBaseUI*>(clientData)->get_add_gravity();
	}

	DIALOG_SET_REGISTER(set_num_of_fps_cluster, int, DeformPhysBaseUI)
	DIALOG_GET_REGISTER(get_num_of_fps_cluster, int, DeformPhysBaseUI)

	DIALOG_SET_REGISTER(set_with_dynamics, bool, DeformPhysBaseUI)
	DIALOG_GET_REGISTER(get_with_dynamics, bool, DeformPhysBaseUI)

	DIALOG_SET_REGISTER(set_add_gravity, bool, DeformPhysBaseUI)
	DIALOG_GET_REGISTER(get_add_gravity, bool, DeformPhysBaseUI)

	DIALOG_SET_REGISTER(set_arap_iteration, int, DeformPhysBaseUI)
	DIALOG_GET_REGISTER(get_arap_iteration, int, DeformPhysBaseUI)

	DIALOG_SET_REGISTER(set_phys_iteration, int, DeformPhysBaseUI)
	DIALOG_GET_REGISTER(get_phys_iteration, int, DeformPhysBaseUI)

	DIALOG_SET_REGISTER(set_time_step, double, DeformPhysBaseUI)
	DIALOG_GET_REGISTER(get_time_step, double, DeformPhysBaseUI)

	DIALOG_SET_REGISTER(set_mu, double, DeformPhysBaseUI)
	DIALOG_GET_REGISTER(get_mu, double, DeformPhysBaseUI)

	DIALOG_SET_REGISTER(set_rho, double, DeformPhysBaseUI)
	DIALOG_GET_REGISTER(get_rho, double, DeformPhysBaseUI)

	DIALOG_SET_REGISTER(set_energy, double, DeformPhysBaseUI)
	DIALOG_GET_REGISTER(get_energy, double, DeformPhysBaseUI)

public:	

	void load_rotation_cluster_from_file(const char* vertex_group_file_name);
	void load_per_group_weight_from_file(const char* fname);

	void set_FPS_rotation_cluster();
	void set_FPS_rotation_cluster(int num_of_cluster);

};



#endif /*DEFORM_PHYS_BASE_H*/