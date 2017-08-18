#ifndef IGL_ADDIN_HANDLE_STRUCTURE_H
#define IGL_ADDIN_HANDLE_STRUCTURE_H

#include <handle.h>
#include <vector>

class HandleStructure{
public:
	HandleStructure():has_build(false)
	{}

	PointMatrixType Trans;
	Eigen::MatrixXd Trans2d;
	PointMatrixType Vars;
	Eigen::MatrixXd Vars2d;
	PointMatrixType Handles;

	Eigen::VectorXi HandleIndices;// The indices of handles in the mesh
	Eigen::VectorXi HandleIndicesInGroup;
	Eigen::VectorXi HandleGroup;

	Eigen::MatrixXi Map2Dto3D;

	FaceMatrixType HandleFaces;
	TetMatrixType HandleTets;

	std::vector<handleData> all_handle_list;
	std::vector<int> select_handle_list;

	bool has_build;
	Eigen::MatrixXd M3d;
	Eigen::MatrixXd M2d;

	void update(const Eigen::MatrixXd& new_TH);
	void update_from_list();
	void attach_onto_mesh_vertices(const PointMatrixType * vertices_to_search, const PointMatrixType * vertices_rest_position);
	void attach_onto_mesh_vertices(const PointMatrixType * vertices_to_search, const PointMatrixType * vertices_rest_position, int index);

	void append(
		const Eigen::MatrixXd& VV,
		const Eigen::MatrixXi& TT,
		const Eigen::MatrixXi& FF,
		const Eigen::MatrixXi& EE,
		const Eigen::MatrixXi& PP,
		const Eigen::MatrixXi& BE,
		const Eigen::MatrixXi& CF,
		const std::vector<std::vector<Eigen::VectorXi>>& GG);

	void build( 
		const Eigen::MatrixXd& VV,
		const Eigen::MatrixXi& TT,
		const Eigen::MatrixXi& FF,
		const Eigen::MatrixXi& EE,
		const Eigen::MatrixXi& PP,
		const Eigen::MatrixXi& BE,
		const Eigen::MatrixXi& CF,
		const std::vector<std::vector<Eigen::VectorXi>>& GG);
	void build();

	void build_old(const bool default_pure_point, const Eigen::MatrixXd& H, Eigen::VectorXi& HG);
	
	int total_rows();
	// old usage
	void edit_insert_new_handles(const Eigen::MatrixXd& new_P, const bool in_same_group);
	// new usage
	void edit_add_point(const Eigen::MatrixXd& new_P);
	void edit_add_boneedge(const Eigen::MatrixXd& new_P);
	void edit_add_region(const Eigen::MatrixXd& new_P);
	bool edit_delete_handle(const int index);

	void set_all_active();
	void set_active(const Eigen::MatrixXi& active);// TODO, change to VectorXi
	void get_active(Eigen::VectorXi& active);
	void get_active_mask(Eigen::VectorXi& active_mask);//0 for inactive, 1 for active
	void set_one_active(int index, bool active);
	void get_one_active(int index, bool& active);

	void set_unique_group();
	bool set_group(const Eigen::VectorXi& hg);

	void get_M(Eigen::MatrixXd& rg3d, Eigen::MatrixXd& rg2d);
private:
	void setup_M();
public:
	void set_pos_trans(Eigen::MatrixXd& Pose);
	void set_pos(Eigen::MatrixXd& Pose, Eigen::VectorXi& Mask, bool abs_pos = true); // if true, the pose is absolute position, if false, it is displacement w.r.t each handle's rest pose 
	void get_pos(Eigen::MatrixXd& Pose, Eigen::VectorXi& Mask, bool abs_pos = true);

	void set_trans(Eigen::MatrixXd& Trans, Eigen::VectorXi& Mask);
	void get_trans(Eigen::MatrixXd& Trans, Eigen::VectorXi& Mask);

	void clear_selected_list();
	void add_selected(int index);

	void set_rotation_centers(const Eigen::MatrixXd& RCs, const Eigen::VectorXi& Is);

	void clear();

	bool get_constraint(
		const int dim,
		//const bool b_constrain_lbs_affine_trans,
		//const Eigen::MatrixXd& M,
		Eigen::VectorXi& known,
		Eigen::MatrixXd& knownValue,
		Eigen::MatrixXd& Meq,
		Eigen::MatrixXd& Peq);

	void get_boundary_condition(Eigen::MatrixXi& BC);

	Eigen::MatrixXd centerPos();

	template <typename DerivedV, typename DerivedT, typename DerivedF, typename DerivedE, typename DerivedP>
	inline void exportHANDLE(
		// outputs:
		Eigen::PlainObjectBase<DerivedV> & V, 
		Eigen::PlainObjectBase<DerivedT> & T,
		Eigen::PlainObjectBase<DerivedF> & F,
		Eigen::PlainObjectBase<DerivedE> & E,
		Eigen::PlainObjectBase<DerivedP> & P,
		Eigen::PlainObjectBase<DerivedE> & BE,
		Eigen::PlainObjectBase<DerivedF> & CF,
		std::vector<std::vector<Eigen::VectorXi>>& G);

	bool exportState();

	bool writeToFile(const std::string str);
};

#endif /*IGL_ADDIN_HANDLE_STRUCTURE_H*/