#ifndef CONTROL_UNIT_H
#define CONTROL_UNIT_H


#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

#include <transWidget.h>

#include <./plugins/HandlePlugin.h>

class ControlUnit
{
public:
	ControlUnit(double x, double y, double z, int i, double cx, double cy, double cz);
	ControlUnit(double x, double y, double z, int i);
	TransWidget rotateWidget;
	void set_rw_radius(double r = 91.);
	void set_rw_scaling(double s = 1.);
	bool mouseMove(int mouse_x, int mouse_y, int height);
	bool mouseUp(int mouse_x, int mouse_y, int height);
	bool mouseDown(int mouse_x, int mouse_y, int height);
	void draw();

	/*********** Data ***********/
public:
	int index;// this is the list in all_handle_list of handle structure

	Eigen::MatrixXd UnitTrans;
	Eigen::MatrixXd ParentsTrans;
	Eigen::MatrixXd TotalTrans;

	ControlUnit *parent;
	std::vector<ControlUnit *> children;


	Eigen::Vector4d rotCenter;
	Eigen::Vector4d restRotCenter;

private:
	Eigen::Vector4d center;
	Eigen::Vector4d restCenter;
public:
	void family_tree(std::vector<ControlUnit*>& tree);
	void print_family_tree(int depth_so_far = 0, bool on_a_new_row = false);// default setting means start as the root

	void renew_trans();
	void propagateTrans();
	void update_local();
	void updateFromRoot();
private:
	void innerUpdateFromParent();

	void push_parent_trans();
	void pop_parent_trans();
};

class ControlStruct{
public:
	std::vector<ControlUnit> controlUnits;
	void set_units_radius(double r);
	bool set_units_scaling(double s, std::vector<int>& selected, std::vector<Eigen::MatrixXd>& trans);
	// This is low cost one, but only works for when one trans is changed.
	void gather_trans_change_local(int cuIndex, std::vector<int>& selected, std::vector<Eigen::MatrixXd>& trans);
	// This is high cost one, works for many trans are changed.
	void gather_trans_change_global(std::vector<int>& selected, std::vector<Eigen::MatrixXd>& trans);
	void update_global();

	bool mouseMove(int mouse_x, int mouse_y, int height, /*output:*/std::vector<int>& selected, std::vector<Eigen::MatrixXd>& trans);
	bool mouseUp(int mouse_x, int mouse_y, int height, /*output:*/std::vector<int>& selected, std::vector<Eigen::MatrixXd>& trans);
	bool mouseDown(int mouse_x, int mouse_y, int height, /*output:*/std::vector<int>& selected, std::vector<Eigen::MatrixXd>& trans);
	void draw();

	void initTest(const HandleStructure& hs);// should remove at some point
	void initAsIndependent(const HandleStructure& hs);// should remove at some point
	void initActive(const HandleStructure& hs);
	void init(const HandleStructure& hs, const Eigen::VectorXi& cs);

	void add(const HandleStructure& hs, const Eigen::VectorXi& acs);

	std::vector<int> handle2csindex;

	std::vector<bool> selected_list;
	void set_selected_list(bool init_value = false);

	bool draw_skeleton;
	std::vector<int> CUL;
	//std::vector<std::pair<int, int>> BEL;
	Eigen::MatrixXd C;
	Eigen::MatrixXi BE;

	void print();

};

class ControlStructState{
public:
	std::string name;
	Eigen::MatrixXd csState;
	ControlStructState(const std::string& n = "", const Eigen::MatrixXd& css = Eigen::MatrixXd(4, 0)) :name(n), csState(css){}
};

bool saveControlStructState(const std::string& name, const ControlStruct& cs);

bool setControlStructState(const Eigen::MatrixXd& st, ControlStruct& cs, std::vector<int>& selected, std::vector<Eigen::MatrixXd>& trans, bool safe_mode_for_dof = true);

bool loadControlStructState(const std::string& name, ControlStruct& cs, std::vector<int>& selected, std::vector<Eigen::MatrixXd>& trans);

void setControlStructTranslate(const float *t, ControlStruct& cs, std::vector<int>& selected, std::vector<Eigen::MatrixXd>& trans);

bool load_one_keyframe(const char * file_name, ControlStructState& keyframe);

void blend_between_two_frames(const ControlStructState& a, const ControlStructState& b, const double f, ControlStructState& result);



#endif /*CONTROL_UNIT_H*/