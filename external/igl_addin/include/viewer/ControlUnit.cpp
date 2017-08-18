#include "ControlUnit.h"

#include <igl/readDMAT.h>

ControlUnit::ControlUnit(double x, double y, double z, int i, double cx, double cy, double cz)
{
	restRotCenter(0) = rotCenter(0) = rotateWidget.pos(0) = x;//0;//x;
	restRotCenter(1) = rotCenter(1) = rotateWidget.pos(1) = y;//0;//y;
	restRotCenter(2) = rotCenter(2) = rotateWidget.pos(2) = z;//0;//z;
	restRotCenter(3) = rotCenter(3) = 1.;
	index = i;

	parent = NULL;

	center(0) = restCenter(0) = cx;
	center(1) = restCenter(1) = cy;
	center(2) = restCenter(2) = cz;
	center(3) = restCenter(3) = 1.;

	UnitTrans = TotalTrans = ParentsTrans = Eigen::MatrixXd::Identity(4, 4);
}

ControlUnit::ControlUnit(double x, double y, double z, int i) : ControlUnit(x, y, z, i, x, y, z)
{}

void ControlUnit::set_rw_radius(double r/* =91. */)
{
	rotateWidget.outer_radius_on_screen = r;
}

void ControlUnit::set_rw_scaling(double s /* = 1. */)
{
	rotateWidget.scaling = s;
	update_local();
}

bool ControlUnit::mouseDown(int mouse_x, int mouse_y, int height)
{
	push_parent_trans();

	bool r = rotateWidget.down(mouse_x, mouse_y, height);

	pop_parent_trans();

	if (r)
		update_local();
	return r;
}

bool ControlUnit::mouseUp(int mouse_x, int mouse_y, int height)
{
	push_parent_trans();

	bool r = rotateWidget.up(mouse_x, mouse_y, height);

	pop_parent_trans();

	if (r)
		update_local();
	return r;
}

bool ControlUnit::mouseMove(int mouse_x, int mouse_y, int height)
{
	push_parent_trans();

	bool r = rotateWidget.drag(mouse_x, mouse_y, height);

	pop_parent_trans();

	if (r)
		update_local();
	return r;
}

#include "draw_point.h"
#include <igl/draw_skeleton_vector_graphics.h>
#include <igl/draw_skeleton_3d.h>
void ControlUnit::draw()
{
	//if (parent==NULL)
	//{
	//	draw_point(rotateWidget.pos(0), rotateWidget.pos(1), rotateWidget.pos(2), 5., true);
	//}

	push_parent_trans();



	if (true)
	{


		glPushMatrix();
		glTranslated(rotateWidget.pos(0), rotateWidget.pos(1), rotateWidget.pos(2));
		glMultMatrixd(Eigen::Affine3d(rotateWidget.rot).matrix().data());
		glTranslated(-rotateWidget.pos(0), -rotateWidget.pos(1), -rotateWidget.pos(2));

		// this kind of usage gives a way to break into the loop
		if (parent != NULL && rotateWidget.enable_rotation)
		{
			Eigen::MatrixXd C(2, 3);
			C <<
				//parent->rotateWidget.pos(0),
				//parent->rotateWidget.pos(1),
				//parent->rotateWidget.pos(2),
				rotateWidget.pos(0) + rotateWidget.translation(0),
				rotateWidget.pos(1) + rotateWidget.translation(1),
				rotateWidget.pos(2) + rotateWidget.translation(2),
				center(0) + rotateWidget.translation(0),
				center(1) + rotateWidget.translation(1),
				center(2) + rotateWidget.translation(2);
			Eigen::MatrixXi BE(1, 2);
			BE << 0, 1;
			if (false)
			{
				igl::draw_skeleton_vector_graphics(C, BE);
				//igl::draw_skeleton_3d(C, BE);
			}
		}


		glPopMatrix();

		PreTransWidgetGLSettings ps;
		rotateWidget.draw_pre();
		rotateWidget.draw_mid_pre(ps);

		rotateWidget.draw_mid_mid();
		rotateWidget.draw_mid_post(ps);
		rotateWidget.draw_post();
	}
	else
	{
		rotateWidget.draw();
		//this is equivalent to
		/*
		PreTransWidgetGLSettings ps;
		rotateWidget.draw_pre();// this kind of usage gives a way to break into the loop
		rotateWidget.draw_mid_pre(ps);
		rotateWidget.draw_mid_mid();
		rotateWidget.draw_mid_post(ps);
		rotateWidget.draw_post();
		*/
	}



	pop_parent_trans();
}

void ControlUnit::push_parent_trans()
{
	Eigen::MatrixXf Mat = ParentsTrans.cast<float>();
	glPushMatrix();
	glMultMatrixf(Mat.data());
}

void ControlUnit::pop_parent_trans()
{
	glPopMatrix();
}

#define MAX_FAMILY_TREE_SIZE 512
void ControlUnit::family_tree(std::vector<ControlUnit*>& tree)
{// DFS: Depth First Transverse for the tree

	if (tree.size()>MAX_FAMILY_TREE_SIZE)
	{
		printf("Error: exceeds max family tree size, there is probably a loop in the given control struct!\n");
		return;
	}

	tree.push_back(this);
	for (int i = 0; i<children.size(); i++)
	{
		children[i]->family_tree(tree);
	}
}

#define MAX_FAMILY_TREE_DEPTH (MAX_FAMILY_TREE_SIZE/2)
void ControlUnit::print_family_tree(int depth_so_far, bool on_a_new_row)
{
	if (depth_so_far>MAX_FAMILY_TREE_DEPTH)
	{
		printf("Error: exceeds max family tree depth, there is probably a loop in the given control struct!\n");
		return;
	}

	if (on_a_new_row)
	{
		for (int i = 0; i<depth_so_far; i++)
			printf("    ");
	}

	printf("-%2d ", index);
	if (children.size() == 0)
	{
		printf("\n");
	}
	else
	{
		for (int i = 0; i<children.size(); i++)
		{
			children[i]->print_family_tree(depth_so_far + 1, i != 0);
		}
	}

}

#include <transform.h>
void ControlUnit::renew_trans()
{
	float rot[4];
	rot[0] = rotateWidget.rot.x();
	rot[1] = rotateWidget.rot.y();
	rot[2] = rotateWidget.rot.z();
	rot[3] = -rotateWidget.rot.w();

	//HandlePlugin::GetReference().ExternalCallRotateSelectedHandles(rot);

	Eigen::MatrixXd Rot;
	ConvertQuaternionToMatrix3x3(rot, Rot);
	Rot = Rot * rotateWidget.scaling;

	Eigen::MatrixXd Trans = Eigen::MatrixXd::Identity(4, 4);
	Trans.block(0, 0, 3, 3) = Rot;
	if (true)//parent==NULL)
	{
		Trans.block(0, 3, 3, 1) = rotateWidget.translation;// Now only supports translation for *root*;
	}

	UnitTrans = Trans;
	// does not support additional translation now
}

void ControlUnit::update_local()
{
	renew_trans();
	propagateTrans();
}

void ControlUnit::propagateTrans()
{
	//UnitTrans = new_UnitTrans;
	innerUpdateFromParent();
}

void ControlUnit::updateFromRoot()
{
	if (parent != NULL)
	{
		printf("Error in calling updateFromRoot from non-root!\n");
		return;
	}

	innerUpdateFromParent();
}

void ControlUnit::innerUpdateFromParent()
{
	// does not support additional translation now

#if 0

	if (parent != NULL)
		ParentsTrans = parent->TotalTrans;
	Eigen::Vector4d disp = restRotCenter;
	if (parent != NULL)
		disp = disp - parent->restRotCenter;

	TotalTrans.block(0, 0, 3, 3) = ParentsTrans.block(0, 0, 3, 3) * UnitTrans.block(0, 0, 3, 3);

	TotalTrans.block(0, 3, 3, 1) = ParentsTrans.block(0, 3, 3, 1) +
		(UnitTrans.block(0, 0, 3, 3) - Eigen::MatrixXd::Identity(3, 3)) * disp.block(0, 0, 3, 1);
#else

	if (parent != NULL)
		ParentsTrans = parent->TotalTrans;

	Eigen::Vector3d pDist = Eigen::Vector3d::Zero();
	if (parent != NULL)
		pDist = parent->TotalTrans.block(0, 3, 3, 1);

	//if (parent==NULL)
	//	pDist += UnitTrans.block(0, 3, 3, 1);// not sure if this is correct, only apply this for root now.

	//Eigen::Vector4d rRC = restRotCenter;
	//if (parent!=NULL)
	//	rRC = rRC - parent->restRotCenter;

	TotalTrans.block(0, 0, 3, 3) = ParentsTrans.block(0, 0, 3, 3) * UnitTrans.block(0, 0, 3, 3);

	TotalTrans.block(0, 3, 3, 1) = pDist.block(0, 0, 3, 1) + (ParentsTrans.block(0, 0, 3, 3) - TotalTrans.block(0, 0, 3, 3)) * restRotCenter.block(0, 0, 3, 1)
		+ ParentsTrans.block(0, 0, 3, 3) * UnitTrans.block(0, 3, 3, 1);

	//	ParentsTrans.block(0,3,3,1) + 
	//	( UnitTrans.block(0,0,3,3) - Eigen::MatrixXd::Identity(3,3) ) * disp.block(0,0,3,1);
#endif
	//rotCenter = ParentsTrans * restRotCenter; // rotCenter is attached to parent

	for (int i = 0; i<children.size(); i++)
	{
		children[i]->innerUpdateFromParent();
	}
}


void ControlStruct::set_units_radius(double r)
{
	for (int i = 0; i < controlUnits.size(); i++)
	{
		controlUnits[i].set_rw_radius(r);
	}
}

bool ControlStruct::set_units_scaling(double s, std::vector<int>& selected, std::vector<Eigen::MatrixXd>& trans)
{
	for (int i = 0; i < controlUnits.size(); i++)
	{
		if (selected_list[i])
		{
			controlUnits[i].set_rw_scaling(s);

			gather_trans_change_local(i, selected, trans);

			return true;
		}
	}
	return false;
}

void ControlStruct::gather_trans_change_local(int cuIndex, std::vector<int>& selected, std::vector<Eigen::MatrixXd>& trans)
{
	std::vector<ControlUnit*> change_tree;
	controlUnits[cuIndex].family_tree(change_tree);

	for (int j = 0; j<change_tree.size(); j++)
	{
		Eigen::MatrixXd Tr;
		Tr = change_tree[j]->TotalTrans.block(0, 0, 3, 4);//(change_tree[j]->TotalTrans).inverse().block(0,0,3,4);

		selected.push_back(change_tree[j]->index);
		trans.push_back(Tr);
	}
}

void ControlStruct::gather_trans_change_global(std::vector<int>& selected, std::vector<Eigen::MatrixXd>& trans)
{
	selected.clear();
	trans.clear();

	for (int i = 0; i<controlUnits.size(); i++)
	{
		Eigen::MatrixXd Tr;
		Tr = controlUnits[i].TotalTrans.block(0, 0, 3, 4);//(change_tree[j]->TotalTrans).inverse().block(0,0,3,4);

		selected.push_back(controlUnits[i].index);
		trans.push_back(Tr);
	}
}

void ControlStruct::update_global()
{
	for (int i = 0; i<controlUnits.size(); i++)
	{
		controlUnits[i].renew_trans();
	}

	for (int i = 0; i<controlUnits.size(); i++)
	{
		if (controlUnits[i].parent == NULL)// is root
		{
			controlUnits[i].update_local();
		}
	}
}


void ControlStruct::initTest(const HandleStructure& hs)
{
	controlUnits.clear();
	for (int i = 0; i<hs.all_handle_list.size(); i++)
	{
		const handleData& hd = hs.all_handle_list[i];
		if (hd.type() != HandleTypePoint)
		{
			ControlUnit cu(hd.CenterPos()(0, 0), hd.CenterPos()(0, 1), hd.CenterPos()(0, 2), i);
			controlUnits.push_back(cu);
		}
	}

	for (int i = 1; i<controlUnits.size(); i++)
	{
		controlUnits[i].parent = &controlUnits[i - 1];
		controlUnits[i - 1].children.push_back(&controlUnits[i]); // just for now
	}

	for (int i = 0; i < controlUnits.size(); i++)
	{
		if (controlUnits[i].parent != NULL)
		{
			controlUnits[i].rotateWidget.enable_translation = false;// only allow roots translation now
		}
	}

	set_selected_list(false);

	print();
}

void ControlStruct::initActive(const HandleStructure& hs)
{
	Eigen::VectorXi cs(hs.all_handle_list.size());

	for (int i = 0; i < hs.all_handle_list.size(); i++)
	{
		cs(i) = (hs.all_handle_list[i].active) ? -1 : -2;
	}

	init(hs, cs);
}

void ControlStruct::init(const HandleStructure& hs, const Eigen::VectorXi& cs)
{
	// cs 
	// -2: not in the list
	// -1: in the list as root
	// >=0 the index to its parent.

	if (cs.rows() != hs.all_handle_list.size())
	{
		printf("Error: the handle struct does not match provided control struct!\n");
		return;
	}

	handle2csindex.clear();
	controlUnits.clear();

	add(hs, cs);
}


void ControlStruct::add(const HandleStructure& hs, const Eigen::VectorXi& cs)
{
	const bool verbose = false;


	if (cs.rows() != hs.all_handle_list.size())
	{
		printf("Error: the handle struct does not match provided control struct!\n");
		return;
	}

	// indexInCS
	// -2: for init purpose, not in the list; if add to exsiting list, all pre-existing element should be -2
	// -1: root
	// >=0: the index into controlUnits 
	Eigen::VectorXi indexInCS(cs.rows());
	indexInCS.setConstant(-2);

	// If handle2csindex is already non-empty, copy the conversion table there
	for (int i = 0; i < handle2csindex.size(); i++)
	{
		indexInCS(i) = handle2csindex[i];
	}

	if (true)//verbose
	{
		std::cout << "cs:" << cs << std::endl;
		std::cout << "indexincs:" << indexInCS << std::endl;

		printf("1 controlUnits (%d):\n", controlUnits.size());
		for (int i = 0; i < controlUnits.size(); i++)
		{
			printf("[%d]: (%d)", i, controlUnits[i].index);
		}
	}

	for (int i = 0; i < hs.all_handle_list.size(); i++)
	{
		const handleData& hd = hs.all_handle_list[i];
		if (cs(i) >= -1)
		{
			indexInCS(i) = controlUnits.size();
			Eigen::MatrixXd rc = hd.getRotCenter();
			Eigen::MatrixXd cc = hd.CenterPos();
			ControlUnit cu(rc(0, 0), rc(0, 1), rc(0, 2), i, cc(0, 0), cc(0, 1), cc(0, 2));
			if (hd.type() == HandleTypePoint)
				cu.rotateWidget.enable_rotation = false;
			controlUnits.push_back(cu);
		}
	}

	// setup connectivity
	// No need to clear previous setup-ed struct
	for (int i = 0; i < hs.all_handle_list.size(); i++)
	{
		const handleData& hd = hs.all_handle_list[i];
		if (cs(i) >= 0)
		{
			if (cs(i) >= indexInCS.rows() || indexInCS(cs(i)) < 0)
			{
				printf("Error: illegal control struct!\n");
				if (verbose)
				{
					std::cout << "cs:" << cs << std::endl;
					std::cout << "indexInCS" << indexInCS << std::endl;
				}

				controlUnits.clear();
				return;
			}
			if (i == cs(i)) //This condition shoudl be wrong: if (indexInCS(i)>=0 && indexInCS(i) == cs(i))
			{
				printf("Error: illegal control struct, cannot point to itself as parent node!\n");
				controlUnits.clear();
				return;
			}
			if (verbose)
			{
				printf("ControlStruct::add(): parenting %d to %d\n", indexInCS(i), indexInCS(cs(i)));
			}

			controlUnits[indexInCS(i)].parent = &controlUnits[indexInCS(cs(i))];
			controlUnits[indexInCS(cs(i))].children.push_back(&controlUnits[indexInCS(i)]);
		}
	}

	// update the list for future add usage.
	handle2csindex.resize(indexInCS.rows());
	for (int i = 0; i < handle2csindex.size(); i++)
	{
		handle2csindex[i] = indexInCS(i);
	}

	if (verbose)
	{
		std::cout << "Indexincs:" << indexInCS << std::endl;
		printf("1 controlUnits (%d):\n", controlUnits.size());
		for (int i = 0; i < controlUnits.size(); i++)
		{
			printf("[%d]: (%d)", i, controlUnits[i].index);
		}
	}

	{
		//std::vector<int> CUL;
		CUL.clear();
		for (int i = 0; i < controlUnits.size(); i++)
		{
			//if (true) // any controlUnits
			{
				CUL.push_back(i);
			}
		}
		//Eigen::MatrixXd C(CUL.size(), 3);
		C.resize(CUL.size(), 3);

		//for (int i = 0; i < CUL.size(); i++)
		//{
		//	Eigen::Vector4d u = controlUnits[CUL[i]].ParentsTrans * controlUnits[CUL[i]].rotCenter;

		//	C(i, 0) = u(0);
		//	C(i, 1) = u(1);
		//	C(i, 2) = u(2);
		//}

		std::vector<std::pair<int, int>> BEL;
		for (int i = 0; i < CUL.size(); i++)
		{
			if (controlUnits[CUL[i]].parent != NULL)
			{
				BEL.push_back(std::make_pair(CUL[i], controlUnits[CUL[i]].parent->index));
			}
		}
		//Eigen::MatrixXi BE(BEL.size(), 2);
		BE.resize(BEL.size(), 2);

		for (int i = 0; i < BEL.size(); i++)
		{
			BE(i, 0) = BEL[i].first;
			BE(i, 1) = BEL[i].second;
		}
	}

	set_selected_list(false);

	if (verbose)
	{
		print();
	}
}


void ControlStruct::set_selected_list(bool init_value /* = false */)
{
	selected_list.resize(controlUnits.size());
	for (int i = 0; i < selected_list.size(); i++)
	{
		selected_list[i] = init_value;
		controlUnits[i].rotateWidget.selected = init_value;
	}
}

void ControlStruct::print()
{
	for (int i = 0; i<controlUnits.size(); i++)
	{
		if (controlUnits[i].parent == NULL)// is root of any tree
		{
			controlUnits[i].print_family_tree();
		}
	}
}

bool ControlStruct::mouseDown(int mouse_x, int mouse_y, int height, std::vector<int>& selected, std::vector<Eigen::MatrixXd>& trans)
{

	for (int i = 0; i<controlUnits.size(); i++)
	{
		bool response = controlUnits[i].mouseDown(mouse_x, mouse_y, height);
		if (response)
		{

			set_selected_list(false);
			selected_list[i] = true;
			controlUnits[i].rotateWidget.selected = true;

			gather_trans_change_local(i, selected, trans);//WriteExternalToHandle(i);
			//any_response |= response;
			return true;
		}
		else
		{
			selected_list[i] = false;
			controlUnits[i].rotateWidget.selected = false;
		}
	}

	return false;

}

bool ControlStruct::mouseUp(int mouse_x, int mouse_y, int height, std::vector<int>& selected, std::vector<Eigen::MatrixXd>& trans)
{

	for (int i = 0; i<controlUnits.size(); i++)
	{
		bool response = controlUnits[i].mouseUp(mouse_x, mouse_y, height);
		if (response)
		{
			gather_trans_change_local(i, selected, trans);//WriteExternalToHandle(i);
			//any_response |= response;
			return true;
		}
	}

	return false;
}

bool ControlStruct::mouseMove(int mouse_x, int mouse_y, int height, std::vector<int>& selected, std::vector<Eigen::MatrixXd>& trans)
{

	for (int i = 0; i<controlUnits.size(); i++)
	{
		bool response = controlUnits[i].mouseMove(mouse_x, mouse_y, height);
		if (response)
		{
			gather_trans_change_local(i, selected, trans);//WriteExternalToHandle(i);
			//any_response |= response;
			return true;
		}
	}

	return false;
}

Eigen::Vector4d ApplyTrans(const Eigen::MatrixXd& Trans, const Eigen::Vector4d& v)
{
	//Eigen::Vector4d u(v(0), v(1), v(2), v(3));
	return Trans*v;
}

void ControlStruct::draw()
{
	for (int i = 0; i<controlUnits.size(); i++)
	{
		controlUnits[i].draw();
	}


	if (draw_skeleton)
	{
		if (C.rows()>0 && BE.rows() > 0)
		{
			// update the positions of controlUnits
			for (int i = 0; i < CUL.size(); i++)
			{
				Eigen::Vector4d d = Eigen::Vector4d::Zero(4, 1);
				d.block(0, 0, 3, 1) = controlUnits[CUL[i]].rotateWidget.translation;
				Eigen::Vector4d u = controlUnits[CUL[i]].ParentsTrans * (controlUnits[CUL[i]].rotCenter + d);

				C(i, 0) = u(0);
				C(i, 1) = u(1);
				C(i, 2) = u(2);
			}
			{
				std::vector<std::pair<int, int>> BEL;
				for (int i = 0; i < CUL.size(); i++)
				{
					if (controlUnits[CUL[i]].parent != NULL)
					{
						BEL.push_back(std::make_pair(CUL[i], controlUnits[CUL[i]].parent->index));
					}
				}
				//Eigen::MatrixXi BE(BEL.size(), 2);
				BE.resize(BEL.size(), 2);

				for (int i = 0; i < BEL.size(); i++)
				{
					BE(i, 0) = BEL[i].first;
					BE(i, 1) = BEL[i].second;
				}
			}

			// draw the skeleton
			igl::draw_skeleton_3d(C, BE);
		}

	}
}

#include <igl/writeDMAT.h>
bool saveControlStructState(const std::string& name, const ControlStruct& cs)
{
	int h = cs.controlUnits.size();
	Eigen::MatrixXd st(h * 2, 4);
	for (int i = 0; i<h; i++)
	{
		st(i, 0) = cs.controlUnits[i].rotateWidget.rot.x();
		st(i, 1) = cs.controlUnits[i].rotateWidget.rot.y();
		st(i, 2) = cs.controlUnits[i].rotateWidget.rot.z();
		st(i, 3) = cs.controlUnits[i].rotateWidget.rot.w();

		st(i + h, 0) = cs.controlUnits[i].rotateWidget.translation.x();
		st(i + h, 1) = cs.controlUnits[i].rotateWidget.translation.y();
		st(i + h, 2) = cs.controlUnits[i].rotateWidget.translation.z();
		st(i + h, 3) = 1.;
	}
	return igl::writeDMAT(name, st);
}

bool setControlStructState(const Eigen::MatrixXd& st, ControlStruct& cs, std::vector<int>& selected, std::vector<Eigen::MatrixXd>& trans, bool safe_mode_for_dof)
{
	int h = cs.controlUnits.size();

	if (safe_mode_for_dof)
	{
		if (st.rows() != h * 2 || st.cols() != 4)
		{
			printf("Error in setting control struct state, DOFs do not match, should have 2 x %d DOFs, now the state matrix is (%d,%d)!\n", h, st.rows(), st.cols());
			return false;
		}
	}
	else
	{
		if (st.rows() != h * 2 || st.cols() != 4)// This is necessary!
		{
			if (st.rows() < h * 2 && st.rows() % 2 == 0 && st.cols() == 4)
			{
				h = st.rows() / 2;
				printf("Warning: insufficient state variables, only set the first %d variables' states.\n", h);
			}
			else
			{
				printf("Error in setting control struct state, DOFs do not match, should have 2 x %d DOFs, now the state matrix is (%d,%d)!\n", h, st.rows(), st.cols());
				return false;
			}
		}
	}

	assert(h <= cs.controlUnits.size());

	for (int i = 0; i<h; i++)
	{
		cs.controlUnits[i].rotateWidget.down_rot = cs.controlUnits[i].rotateWidget.rot = Eigen::Quaterniond(st(i, 3), st(i, 0), st(i, 1), st(i, 2));

		//cs.controlUnits[i].rotateWidget.rot.x = st(i,0);
		//cs.controlUnits[i].rotateWidget.rot.y = st(i,1);
		//cs.controlUnits[i].rotateWidget.rot.z = st(i,2);
		//cs.controlUnits[i].rotateWidget.rot.w = st(i,3);

	}

	for (int i = 0; i < h; i++)
	{
		//now only apply to all. if (cs.controlUnits[i].parent == NULL)// by now only apply to all roots.
		{
			cs.controlUnits[i].rotateWidget.translation = Eigen::Vector3d(st(i + h, 0), st(i + h, 1), st(i + h, 2));
		}
	}

	cs.update_global();
	cs.gather_trans_change_global(selected, trans);

	return true;
}


void setControlStructTranslate(const float *t, ControlStruct& cs, std::vector<int>& selected, std::vector<Eigen::MatrixXd>& trans)
{
	for (int i = 0; i < cs.controlUnits.size(); i++)
	{
		if (cs.controlUnits[i].parent == NULL)// by now only apply to all roots.
		{
			cs.controlUnits[i].rotateWidget.translation = Eigen::Vector3d(t[0], t[1], t[2]);
		}
	}
	cs.update_global();
	cs.gather_trans_change_global(selected, trans);
}

bool loadControlStructState(const std::string& name, ControlStruct& cs, std::vector<int>& selected, std::vector<Eigen::MatrixXd>& trans)
{
	Eigen::MatrixXd st;
	if (!igl::readDMAT(name, st))
		return false;
	return setControlStructState(st, cs, selected, trans, false);
}

bool load_one_keyframe(const char * file_name, ControlStructState& keyframe)
{
	Eigen::MatrixXd st;
	if (!igl::readDMAT(std::string(file_name), st))
	{
		printf("load_one_keyframe(%s) fails!\n", file_name);
		return false;
	}

	keyframe.name = std::string(file_name);
	keyframe.csState = st;

	return true;
}



//template<typename T>
//void blend_between_two_frames(T& a, T& b, double f, T& result)
//{
//	result = (1-f) * a + f * b;
//}

void blend_between_two_frames(const ControlStructState& a, const ControlStructState& b, const double f, ControlStructState& result)
{
	if (a.csState.rows() != b.csState.rows() || a.csState.cols() != b.csState.cols())
	{
		printf("Error in blend_between_two_frames(). Different frame size!\n");
		return;
	}

	double ef = f;
	//double ef = -2.0*f*f*f + 3.0*f*f;

	result.csState = (1 - ef) * a.csState + ef * b.csState;
}

