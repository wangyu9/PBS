#include "handle_structure.h"

#ifdef USING_IGL_HEADER_ONLY_MODE
#define IGL_HEADER_ONLY 
#endif
#include <writeHANDLE.h>
#include <move_point_onto_mesh.h>

#include "eigen_helper.h"

void set_3d_trans_from_2d(const Eigen::MatrixXd& T0, Eigen::MatrixXd& T)
{
	assert(T0.rows()%3==0);

	int num = T0.rows()/3;
	T.resize(4*num,3);
	T.setZero();

	for (int i=0; i<num; i++)
	{
		T.block(4*i,0,2,2) = T0.block(3*i,0,2,2);
		T(4*i+2,2) = 1;
		T.block(4*i+3,0,1,2) = T0.block(3*i+2,0,1,2);
	}
}

void HandleStructure::update(const Eigen::MatrixXd& new_TH)
{
	int dim = new_TH.cols();
	assert(dim==3||dim==2);
	//assert(new_trans.rows()==4*all_handle_list.size());
	//print_matlab("new_TH",new_TH);

	int th_pointer = 0;
	for(int i=0; i<all_handle_list.size(); i++)
	{
		if (all_handle_list[i].type()==HandleTypePoint)
		{
			if (dim==2)
			{
				all_handle_list[i].updateFromPoint3D(new_TH(th_pointer,0), new_TH(th_pointer,1), 0);// Set the last dimension to be 0.
			}
			else
			{
				all_handle_list[i].updateFromPoint3D(new_TH(th_pointer,0), new_TH(th_pointer,1), new_TH(th_pointer,2));
			}

			th_pointer += 1;//PBS takes one row in the output TH
		}
		else
		{
			//if (all_handle_list[i].type()==HandleTypePoint)// Do not need this because passive handle could have some active DOFs for LBS

			Eigen::MatrixXd T4x3;
			if (dim==2)
			{
				Eigen::MatrixXd T3x2 = new_TH.block(th_pointer,0,3,2);
				set_3d_trans_from_2d(T3x2, T4x3);
			}
			else
			{
				T4x3 = new_TH.block(th_pointer,0,4,3);
			}

			all_handle_list[i].updateTrans(T4x3);

			if (dim==2)
			{
				th_pointer += 3;//PBS takes 3 rows in the output TH in 2D.
			} 
			else
			{
				th_pointer += 4;//PBS takes 4 rows in the output TH in 3D.
			}	
		}

		assert(th_pointer<=new_TH.rows());
	}

	update_from_list();
}

void HandleStructure::update_from_list()
{
	int h = all_handle_list.size();

	Trans.resize(4*h,3);
	Trans2d.resize(3*h,2);

	int num_var_3d = 0;
	int num_var_2d = 0;
	for (int i=0; i<h; i++)
	{
		num_var_3d += all_handle_list[i].Trans(3).rows();
		num_var_2d += all_handle_list[i].Trans(2).rows();
	}
	Vars.resize(num_var_3d,3);
	Vars2d.resize(num_var_2d,2);

	int num_points = 0;
	for(int i=0; i<h; i++)
	{
		num_points += all_handle_list[i].IndexInMesh().rows();
	}
	//assert(num_points==Handles.rows());

	Handles.resize(num_points,3);
	HandleIndices.resize(num_points);
	HandleIndices.setConstant(-1);

	HandleGroup.resize(num_points, 1);
	HandleGroup.setConstant(-1);

	for(int i=0; i<h; i++)
	{

		Eigen::VectorXi I = all_handle_list[i].IndexInMesh();
		Eigen::VectorXi IH = all_handle_list[i].IndexInHandles();
		Eigen::MatrixXd P = all_handle_list[i].Pos();

		for (int j=0; j<I.rows(); j++)
		{
			HandleIndices(IH(j)) = I(j);
			Handles(IH(j),0) = P(j,0);
			Handles(IH(j),1) = P(j,1);
			Handles(IH(j),2) = P(j,2);

			HandleGroup(IH(j)) = i;
		}
	}

	for(int i=0; i<h; i++)
	{
		if (all_handle_list[i].type()!=HandleTypePoint)
		{
			Trans.block(4*i,0,4,3) = all_handle_list[i].Trans(3);
			Trans2d.block(3*i,0,3,2) = all_handle_list[i].Trans(2);
		}
		else
		{
			{
				Eigen::MatrixXd Tr3d(4,3);
				Tr3d.setZero();
				Tr3d(0,0) = Tr3d(1,1) = Tr3d(2,2) = 1;
				Trans.block(4*i,0,4,3) = Tr3d;
			}
			{
				Eigen::MatrixXd Tr2d(3,2);
				Tr2d.setZero();
				Tr2d(0,0) = Tr2d(1,1) = 1;
				Trans2d.block(3*i,0,3,2) = Tr2d;
			}
		}
	}

	int current_var_3d = 0;
	int current_var_2d = 0;
	for (int i=0; i<h; i++)
	{
		{
			int r3 = all_handle_list[i].Trans(3).rows();
			Vars.block(current_var_3d,0,r3,3) = all_handle_list[i].Trans(3);
			current_var_3d += r3;
		}
		{
			int r2 = all_handle_list[i].Trans(2).rows();
			Vars2d.block(current_var_2d,0,r2,2) = all_handle_list[i].Trans(2);
			current_var_2d += r2;

			//print_matlab(all_handle_list[i].Trans(2),"Tr2d");
		}
	}
}

void HandleStructure::attach_onto_mesh_vertices(const Eigen::MatrixXd * vertices_to_search, const Eigen::MatrixXd * vertices_rest_position, int index)
{
	int i = index;

	std::vector<Point3D> sampled_point_list;
	sampled_point_list.clear();

	Eigen::VectorXi I = all_handle_list[i].IndexInMesh();
	Eigen::MatrixXd P = all_handle_list[i].Pos();

	for (int j = 0; j < I.rows(); j++)
	{
		sampled_point_list.push_back(Point3D(P(j, 0), P(j, 1), P(j, 2)));
	}

	attach_all_handles_to_nearest_vertices(vertices_to_search, sampled_point_list);

	for (int j = 0; j < I.rows(); j++)
	{
		I(j) = sampled_point_list[j].index;
	}

	all_handle_list[i].setIndices(I);
	Eigen::MatrixXd restPos = Rows_of(*vertices_rest_position, I).cast<double>();
	Eigen::MatrixXd centerPos = Rows_Average(Rows_of(*vertices_to_search, I).cast<double>());
	all_handle_list[i].setRestPos(restPos);
	all_handle_list[i].setCenterPos(centerPos);
}

void HandleStructure::attach_onto_mesh_vertices(const Eigen::MatrixXd * vertices_to_search, const Eigen::MatrixXd * vertices_rest_position)
{
	
	for (int i=0; i<all_handle_list.size(); i++)
	{
		attach_onto_mesh_vertices(vertices_to_search, vertices_rest_position, i);
	}
}

void HandleStructure::set_unique_group()
{
	HandleGroup.resize(Handles.rows());
	for (int i=0; i<HandleGroup.rows(); i++)
	{
		HandleGroup(i) = i;
		// each handle to be in a unique group.
	}
}

bool HandleStructure::set_group(const Eigen::VectorXi& hg)
{
	int num_group = hg.maxCoeff()+1;
	if (num_group<=hg.rows()&&hg.minCoeff()>=0)
	{
		printf("Read handle group file successfully!");
		HandleGroup = hg;
		return true;
	}
	else
	{
		printf("Handle group file incorrect!");
		return false;
	}
}


bool add_one_handle(const Eigen::MatrixXd& P, const Eigen::VectorXi& I_in_handle, const bool default_point, std::vector<handleData>& all_handle_list)
{
	assert(P.rows()>=1);

	if (P.rows()==1)
	{
		if(default_point)
		{
			handleData& new_h = handleData::Point(P,I_in_handle);	
			all_handle_list.push_back(new_h);
		}
		else
		{
			handleData& new_h = handleData::FramePoint(P,I_in_handle);
			all_handle_list.push_back(new_h);
		}
	}
	else
	{
		// group with more than 1 control handle vertices
		handleData& new_h = handleData::Vertices(P,I_in_handle);	
		all_handle_list.push_back(new_h);
	}

	return true;
}

bool add_one_handle(const HandleType ht, const Eigen::MatrixXd& P, const Eigen::VectorXi& I_in_handle, std::vector<handleData>& all_handle_list)
{
	assert(P.rows()>=1);

	switch(ht)
	{
	case HandleTypePoint:
		{
			assert(P.rows()==1);
			handleData& new_h = handleData::Point(P,I_in_handle);	
			all_handle_list.push_back(new_h);
		}
		break;
	case HandleTypeVertices:
		{
			assert(P.rows()>1);
			handleData& new_h = handleData::Vertices(P,I_in_handle);	
			all_handle_list.push_back(new_h);
		}
		break;
	case HandleTypePointFrame:
		{
			assert(P.rows()==1);
			handleData& new_h = handleData::FramePoint(P,I_in_handle);
			all_handle_list.push_back(new_h);
		}
		break;
	case HandleTypeBoneEdge:
		{
			assert(P.rows()>1);
			handleData& new_h = handleData::BoneEdge(P,I_in_handle);	
			all_handle_list.push_back(new_h);
		}
		break;
	}
	return true;
}

bool add_one_handle_nature_order(const HandleType ht, const Eigen::MatrixXd& P, std::vector<handleData>& all_handle_list)
{
	Eigen::VectorXi I_in_handle = NaturalSeq(P.rows());
	return add_one_handle(ht, P, I_in_handle, all_handle_list);
}

// Set all_handle_list from Handles and Handle Group
void set_handle_list(
	// Input:
	const Eigen::MatrixXd& Handles, 
	const Eigen::VectorXi& HandleGroup, 
	const bool default_point,// whether for 1D point to be set to pure point or point frame.
	// Output:
	Eigen::VectorXi& HandleIndicesInMesh, 
	Eigen::VectorXi& HandleIndicesInGroup,
	std::vector<handleData>& all_handle_list)
{
	if (HandleIndicesInMesh.rows()!=Handles.rows())
	{
		HandleIndicesInMesh.resize(Handles.rows());
		HandleIndicesInMesh.setConstant(-1);
	}

	/***************collect handle group****************/
	HandleIndicesInGroup.resize(Handles.rows());
	int num_group = HandleGroup.maxCoeff()+1;

	int * group_number = new int[num_group];
	Eigen::MatrixXd * group_mat = new Eigen::MatrixXd[num_group];
	Eigen::VectorXi * group_index = new Eigen::VectorXi[num_group];

	for (int i=0; i<num_group; i++)
	{
		group_number[i] = 0;
	}
	for (int i=0; i<HandleGroup.rows(); i++)
	{
		HandleIndicesInGroup(i) = group_number[HandleGroup(i)]++;
	}
	for (int i=0; i<num_group; i++)
	{
		group_mat[i].resize(group_number[i],3);
		group_index[i].resize(group_number[i]);
	}
	for (int i=0; i<HandleIndicesInMesh.rows(); i++)
	{
		int g = HandleGroup(i);
		int k = HandleIndicesInGroup(i);
		group_mat[g].row(k) = Handles.row(i);
		group_index[g](k) = i;
	}

	all_handle_list.clear();


	for(int i=0; i<num_group; i++)
	{
		add_one_handle(group_mat[i],group_index[i],default_point,all_handle_list);
	}

	delete [] group_number;
	delete [] group_mat;
	delete [] group_index;
}

bool all_slot_occupied(const Eigen::VectorXi& I)
{
	//bool all_occupied = true;
	const int h = I.maxCoeff()+1;
	if (h<=0||h>I.rows())// The later is not necessary, but could save some space sometimes.
	{
		//all_occupied = false;
		return false;
	}
	bool* occupied = new bool[h];
	for (int i=0; i<h; i++)
		occupied[i] = false;
	for (int i=0; i<I.rows(); i++)
	{
		const int& k = I(i);
		if(k<0||k>=h)
		{
			//all_occupied = false;// exceeds possible region!
			delete [] occupied;
			return false;
			//break;
		}
		else
		{
			occupied[k] = true;
		}
	}
	for (int i=0; i<h; i++)
	{
		if(occupied[i]==false)
		{
			//all_occupied = false;// some slot is not occupied!
			delete [] occupied;
			return false;
			//break;
		}
	}
	delete [] occupied;
	return true;
}

void check_handle_group(const int m, Eigen::VectorXi& HandleGroup)
{
	assert(m>=0);
	if(m!=HandleGroup.rows())
	{
		printf("Warning: HandleGroup not set. Set each to be in a unique group.\n");
		HandleGroup = NaturalSeq(m);
	}
	else
	{

		if (all_slot_occupied(HandleGroup)==false)
		{//Each number between 1 and HandleGroup.maxCoeff() 
			//should appears at least once.
			printf("Warning: HandleGroup is incorrect. Set each to be in a unique group.\n");
			HandleGroup = NaturalSeq(m);
		}

	}
}

void HandleStructure::build_old(const bool default_pure_point, const Eigen::MatrixXd& H, Eigen::VectorXi& HG)
{
	// check if HandleGroup File is set up, if not setup the HandleGroup
	check_handle_group(H.rows(), HG);

	set_handle_list(H, 
		HG,
		default_pure_point,
		HandleIndices, 
		HandleIndicesInGroup,
		all_handle_list);

	select_handle_list.clear();

	// Order is important 
	//update_handles_from_list();
	update_from_list();

	setup_M();

	has_build = true;
}

void HandleStructure::build()
{
	select_handle_list.clear();

	update_from_list();

	setup_M();

	has_build = true;
}

void HandleStructure::append(
	const Eigen::MatrixXd& VV,
	const Eigen::MatrixXi& TT,
	const Eigen::MatrixXi& FF,
	const Eigen::MatrixXi& EE,
	const Eigen::MatrixXi& PP,
	const Eigen::MatrixXi& BE,
	const Eigen::MatrixXi& CF,
	const std::vector<std::vector<Eigen::VectorXi>>& GG)
{
	assert(VV.rows() == 0 || VV.cols() == 3);

	assert(TT.rows() == 0 || TT.cols() == 4);
	assert(FF.rows() == 0 || FF.cols() == 3);
	assert(EE.rows() == 0 || EE.cols() == 2);
	assert(PP.rows() == 0 || PP.cols() == 1);
	assert(BE.rows() == 0 || BE.cols() == 2);
	assert(CF.rows() == 0 || CF.cols() == 3);
	assert(GG.size() == 3);
	for (int i = 0; i<GG.size(); i++)
		for (int j = 0; j<GG[i].size(); j++)
			assert(GG[i][j].rows() == 0 || GG[i][j].cols() == 1);

	int pre_num_effe_vert = 0;
	int pre_num_of_handle = 0;
	int pre_num_of_vars_3d = 0;
	int pre_num_of_vars_2d = 0;

	for (int i = 0; i < all_handle_list.size(); i++)
	{
		switch (all_handle_list[i].type())
		{
		case HandleTypePoint:
			pre_num_effe_vert++;
			pre_num_of_handle++;
			pre_num_of_vars_3d++;
			pre_num_of_vars_2d++;
			break;
		case HandleTypePointFrame:
		case HandleTypeBoneEdge:
		case HandleTypeVertices:
			//Treating all types of groups same here.
			pre_num_effe_vert += all_handle_list[i].Pos().rows();
			pre_num_of_handle++;
			pre_num_of_vars_3d += 4;
			pre_num_of_vars_2d += 3;
			break;
		}
	}


	int num_effe_vert = pre_num_effe_vert;
	int num_of_handle = pre_num_of_handle;
	int num_of_vars_3d = pre_num_of_vars_3d;
	int num_of_vars_2d = pre_num_of_vars_2d;

	int num_effe_vert2 = pre_num_effe_vert;

	for (int i = 0; i<PP.rows(); i++)
	{
		num_effe_vert++;
		num_of_handle++;
		num_of_vars_3d++;
		num_of_vars_2d++;
	}
	for (int i = 0; i<GG.size(); i++)
	{
		for (int j = 0; j<GG[i].size(); j++)
		{
			//Treating all types of groups same here.
			num_effe_vert += GG[i][j].rows();
			num_of_handle++;
			num_of_vars_3d += 4;
			num_of_vars_2d += 3;
		}
	}

	//int num_effe_vert2 = pre_num_effe_vert;

	for (int i = 0; i<PP.rows(); i++)
	{
		handleData new_h = handleData::Point(VV.row(PP(i)), NaturalSeq(1, num_effe_vert2));
		all_handle_list.push_back(new_h);
		num_effe_vert2++;
	}

	for (int i = 0; i<GG.size(); i++)
	{
		switch (i)
		{
		case 0: // Group of Points
			for (int j = 0; j<GG[i].size(); j++)
			{
				const Eigen::MatrixXi& b = GG[i][j];
				handleData new_h = handleData::Vertices(Rows_of(VV, b), NaturalSeq(b.rows(), num_effe_vert2));
				all_handle_list.push_back(new_h);
				num_effe_vert2 += b.rows();
			}
			break;
		case 1: // Bone Edges
			for (int j = 0; j<GG[i].size(); j++)
			{
				const Eigen::MatrixXi& b = GG[i][j];
				handleData new_h = handleData::BoneEdge(Rows_of(VV, b), NaturalSeq(b.rows(), num_effe_vert2));
				*(new_h.BE) << BE(j, 0), BE(j, 1);
				all_handle_list.push_back(new_h);
				num_effe_vert2 += b.rows();
			}
			break;
		case 2: // Cage Faces
			//TODO
			break;
		}

	}


	build();
}

void HandleStructure::build(
	const Eigen::MatrixXd& VV, 
	const Eigen::MatrixXi& TT, 
	const Eigen::MatrixXi& FF, 
	const Eigen::MatrixXi& EE, 
	const Eigen::MatrixXi& PP, 
	const Eigen::MatrixXi& BE, 
	const Eigen::MatrixXi& CF, 
	const std::vector<std::vector<Eigen::VectorXi>>& GG)
{
	assert(VV.rows()==0||VV.cols()==3);

	assert(TT.rows()==0||TT.cols()==4);
	assert(FF.rows()==0||FF.cols()==3);
	assert(EE.rows()==0||EE.cols()==2);
	assert(PP.rows()==0||PP.cols()==1);
	assert(BE.rows()==0||BE.cols()==2);
	assert(CF.rows()==0||CF.cols()==3);
	assert(GG.size()==3);
	for (int i=0; i<GG.size(); i++)
		for (int j=0; j<GG[i].size(); j++)
			assert(GG[i][j].rows()==0||GG[i][j].cols()==1);

	all_handle_list.clear();

	append(VV,TT,FF,EE,PP,BE,CF,GG);
}



/*************** Handle Editing *****************/

// For functions in Handles Editing, only Handles and HandleGroup need to be update

void HandleStructure::edit_insert_new_handles(const Eigen::MatrixXd& new_P, const bool in_same_group)
{
	// Update HandleGroup
	Eigen::VectorXi new_part(new_P.rows());
	int new_index = 0;
	if (HandleGroup.rows()>0)
		new_index = HandleGroup.maxCoeff()+1;
	if (in_same_group)
	{
		new_part.setConstant(new_index);
	}
	else
	{
		for (int i=0; i<new_part.rows(); i++)
			new_part(i) = new_index + i;
	}

	Eigen::VectorXi new_HG(HandleGroup.rows()+new_part.rows());
	new_HG << HandleGroup, new_part;
	HandleGroup = new_HG;

	// Update Handles
	Eigen::MatrixXd new_Handles(Handles.rows()+new_P.rows(),3);
	new_Handles << Handles, new_P;
	Handles = new_Handles;
}

int HandleStructure::total_rows()
{
	int total = 0;
	for (int i=0; i<all_handle_list.size(); i++)
	{
		total += all_handle_list[i].Pos().rows();
	}
	return total;
}

void HandleStructure::edit_add_boneedge(const Eigen::MatrixXd& new_P)
{
	handleData new_h = handleData::BoneEdge(new_P, NaturalSeq(new_P.rows(),total_rows()));
	all_handle_list.push_back(new_h);

	has_build = false;
}

void HandleStructure::edit_add_region(const Eigen::MatrixXd& new_P)
{
	handleData new_h = handleData::Vertices(new_P, NaturalSeq(new_P.rows(),total_rows()));
	all_handle_list.push_back(new_h);

	has_build = false;
}

void HandleStructure::edit_add_point(const Eigen::MatrixXd& new_P)
{
	if(new_P.rows()!=1)
	{
		printf("Error in calling edit_add_point!\n");
		return;
	}

	handleData new_h = handleData::Point(new_P, NaturalSeq(new_P.rows(),total_rows()));
	all_handle_list.push_back(new_h);

	has_build = false;
}

bool HandleStructure::edit_delete_handle(const int index)
{
	if (index >= all_handle_list.size() || index<0 )
	{
		printf("Handle index to delete exceeds size!\n");
		return false;
	}

	all_handle_list.erase(all_handle_list.begin()+index);

	has_build = false;
}

/*************** Handle Pose *****************/

void HandleStructure::get_M(Eigen::MatrixXd& rg3d, Eigen::MatrixXd& rg2d)
{
	assert(has_build);

	rg3d = M3d;
	rg2d = M2d;
}

void HandleStructure::setup_M()
{
		
	int m = Handles.rows();
	int h = all_handle_list.size();
	assert(m==Handles.rows());

	Map2Dto3D = Eigen::MatrixXi::Zero(h,2);

	int cols_3d = 0;
	int cols_2d = 0;
	int * col_M3d = new int[h];
	int * col_M2d = new int[h];
	for (int i=0; i<h; i++)
	{
		col_M3d[i] = col_M2d[i] = 0;
	}
	for (int i=0; i<all_handle_list.size(); i++)
	{
		col_M3d[i] = cols_3d;
		col_M2d[i] = cols_2d;
		Map2Dto3D(i,0) = cols_3d;
		if (all_handle_list[i].type()==HandleTypePoint)
		{
			cols_2d += 1;
			cols_3d += 1;
			Map2Dto3D(i,1) = 1;
		}
		else
		{
			cols_2d += 3;
			cols_3d += 4;
			Map2Dto3D(i,1) = 4;
		}
	}

	// Reorder and Group Matrix
	Eigen::MatrixXd RG3d = Eigen::MatrixXd::Zero(m,cols_3d);
	Eigen::MatrixXd RG2d = Eigen::MatrixXd::Zero(m,cols_2d);
	for (int i=0; i<all_handle_list.size(); i++)
	{
		const Eigen::VectorXi& IH = all_handle_list[i].IndexInHandles();
		const Eigen::MatrixXd& Mi2d = all_handle_list[i].Mi(2);
		const Eigen::MatrixXd& Mi3d = all_handle_list[i].Mi(3);
		int cols_Mi3d = Mi3d.cols();
		int cols_Mi2d = Mi2d.cols();
		for (int k=0; k<IH.rows(); k++)
		{
			RG3d.block(IH(k),col_M3d[i],1,cols_Mi3d) = Mi3d.row(k);
			RG2d.block(IH(k),col_M2d[i],1,cols_Mi2d) = Mi2d.row(k);
		}
	}

	//print_matlab(RG3d,"RG3d");
	//print_matlab(RG2d,"RG2d");

	//Eigen::MatrixXd M3d_hs = Weights * RG3d;//(n,cols_3d);
	//Eigen::MatrixXd M2d_hs = Weights * RG2d;//(n,cols_2d);

	delete [] col_M2d;
	delete [] col_M3d;

	//rg2d = RG2d;
	//rg3d = RG3d;

	M2d = RG2d;
	M3d = RG3d;
}

// Handle operations that requires only update all structures

void HandleStructure::set_pos_trans(Eigen::MatrixXd& Pose)
{
	int n = Handles.rows();
	assert(Pose.rows()==4*n);
	assert(Pose.cols()==3);
	assert(Trans.rows()==4*n);

	Trans.resize(4*n,3);
	Trans = Pose;

	//// update Handles position as well
	//Eigen::MatrixXd M;
	//Eigen::MatrixXd Weights(n,n);
	//Weights.setIdentity();

	//igl::lbs_matrix(ori_Handles,Weights,M);
	//Handles = M*Trans;
	//Handles.block(0,0,n,3) = M * Trans;

	for(int i=0; i<n; i++)
	{
		if (all_handle_list[i].type()!=HandleTypePoint)
		{
			Eigen::MatrixXd Tr = Trans.block(4*i,0,4,3);
			all_handle_list[i].updateTrans(Tr);
		}
	}

	update_from_list();
}

void HandleStructure::set_pos(Eigen::MatrixXd& Pose, Eigen::VectorXi& Mask, bool abs_pos)
{
	assert(Pose.cols()==3);

	assert(Mask.rows()==all_handle_list.size());

	int k = 0;
	for(int i=0; i<all_handle_list.size(); i++)
	{
		if (Mask(i)>0)
		{ 
			if (all_handle_list[i].type()!=HandleTypePoint
				&&all_handle_list[i].b_constrain_affine_trans)
			{// do not update region handles that specifying the entire affine trans.
				continue;
			}			
			//all_handle_list[i].updateFromPoint3D(Pose(i,0),Pose(i,1),Pose(i,2));
			all_handle_list[i].setCenterPos(Pose.row(k++), abs_pos);
		}
		
	}

	update_from_list();
}

void HandleStructure::get_pos(Eigen::MatrixXd& Pose, Eigen::VectorXi& Mask, bool abs_pos /* = true */)
{
	int h = all_handle_list.size();
	assert(Mask.rows() == all_handle_list.size());

	int eh = 0;
	for (int i = 0; i < h; i++)
	{
		if (Mask(i)>0)
		{
			eh++;
		}
	}

	Pose.resize(eh,3);
	int k = 0;
	for (int i = 0; i < h; i++)
	{
		if (Mask(i)>0)
		{
			Eigen::MatrixXd Pi;
			all_handle_list[i].getCenterPos(Pi, abs_pos);
			assert(Pi.rows() == 1 && Pi.cols() == 3);
			Pose.row(k++) = Pi;
		}
	}
}

void HandleStructure::set_trans(Eigen::MatrixXd& TPose, Eigen::VectorXi& Mask)
{
	assert(TPose.cols() == 3);

	assert(Mask.rows() == all_handle_list.size());

	int k = 0;
	for (int i = 0; i < all_handle_list.size(); i++)
	{
		if (Mask(i)>0)
		{
			if (all_handle_list[i].type() != HandleTypePoint)
			{
				Eigen::MatrixXd Tr = TPose.block(4 * k, 0, 4, 3);
				all_handle_list[i].updateTrans(Tr);				
			}
			else
			{
				printf("set_trans() Warning: skip Point Type Handle!\n");
			}

			k++;
		}
	}

	update_from_list();
}

void HandleStructure::get_trans(Eigen::MatrixXd& TPose, Eigen::VectorXi& Mask)
{
	int h = all_handle_list.size();
	assert(Mask.rows() == all_handle_list.size());

	int eh = 0;
	for (int i = 0; i < h; i++)
	{
		if (Mask(i)>0)
		{
			eh += 4;
		}
	}

	TPose = Eigen::MatrixXd::Zero(eh, 3);
	int k = 0;
	for (int i = 0; i < h; i++)
	{
		if (Mask(i)>0)
		{
			if (all_handle_list[i].type() != HandleTypePoint)
			{
				Eigen::MatrixXd Pi;
				Pi = all_handle_list[i].Trans(3);
				assert(Pi.rows() == 4 && Pi.cols() == 3);
				TPose.block(4 * k, 0, 4, 3) = Pi;
			} 
			else
			{
				printf("get_trans() Warning: skip Point Type Handle!\n");
			}
			
			k++;
		}
	}
}



void HandleStructure::set_all_active()
{
	for (int i=0; i<all_handle_list.size(); i++)
	{
		all_handle_list[i].active = true;
	}
}

void HandleStructure::set_active(const Eigen::MatrixXi& active)
{
	assert(active.cols()==1);
	for (int i=0; i<all_handle_list.size(); i++)
	{
		all_handle_list[i].active = false;
	}
	for (int i=0; i<active.rows(); i++)
	{
		int k = active(i,0);
		if (k<all_handle_list.size())
		{
			all_handle_list[k].active = true;
		}
		else
		{
			printf("Active list with index[%d] exceeds current handle size[%d], Ignore!\n", k, all_handle_list.size());
			continue;
		}
	}
}

void HandleStructure::get_active(Eigen::VectorXi& active)
{
	int num_active=0;
	for (int i=0; i<all_handle_list.size(); i++)
	{
		if (all_handle_list[i].active)
		{
			num_active++;
		}
	}
	//Eigen::MatrixXi 
	active.resize(num_active);
	int k=0;
	for (int i=0; i<all_handle_list.size(); i++)
	{
		if (all_handle_list[i].active)
		{
			active(k) = i;
			k++;
		}
	}
}

void HandleStructure::get_active_mask(Eigen::VectorXi& active_mask)
{
	active_mask.resize(all_handle_list.size());
	int k = 0;
	for (int i = 0; i < all_handle_list.size(); i++)
	{
		active_mask(i) = (all_handle_list[i].active) ? 1 : 0;
	}
}

void HandleStructure::set_one_active(int index, bool active)
{
	if (index < all_handle_list.size())
	{
		all_handle_list[index].active = active;
	}
	else
	{
		printf("Error in calling set_one_active(), index exceeds handle list size!\n");
	}
}

void HandleStructure::get_one_active(int index, bool& active)
{
	if (index<all_handle_list.size())
	{
		active = all_handle_list[index].active;
	}
	else
	{
		printf("Error in calling get_one_active(), index exceeds handle list size!\n");
	}
}

void HandleStructure::clear_selected_list()
{
	select_handle_list.clear();
	for(int i=0; i<all_handle_list.size(); i++)
	{
		all_handle_list[i].selected = false;
	}
}

void HandleStructure::add_selected(int index)
{
	if(index>=0 && index<all_handle_list.size())
	{
		select_handle_list.push_back(index);
		all_handle_list[index].selected = true;
	}
}

void HandleStructure::clear()
{
	all_handle_list.clear();
	select_handle_list.clear();

	update_from_list();
}

void HandleStructure::set_rotation_centers(const Eigen::MatrixXd& RCs, const Eigen::VectorXi& Is)
{
	assert(RCs.rows()==Is.rows());
	assert(RCs.cols()==3);
	for (int i = 0; i < Is.rows(); i++)
	{
		assert(Is(i)>=0&&Is(i)<all_handle_list.size());
		all_handle_list[Is(i)].setRotCenter(RCs.row(i));
	}
}

bool HandleStructure::get_constraint(
	const int dim,
	//const bool b_constrain_lbs_affine_trans,
	//const Eigen::MatrixXd& M,
	Eigen::VectorXi& known,
	Eigen::MatrixXd& knownValue,
	Eigen::MatrixXd& Meq,
	Eigen::MatrixXd& Peq)
{

	if (!has_build)
	{
		return false; // must build before calling get_constraint
	}

	// Gather Constraints for all handles (active and passive)
	//bool active_case_one = !b_constrain_lbs_affine_trans;
	// First loop, gather rows
	int rows_known = 0;
	int rows_Meq = 0;
	int cols_Meq = 0;// for double check purpose
	for (int i=0; i<all_handle_list.size(); i++)
	{
		switch(all_handle_list[i].type())
		{
		case HandleTypePoint:
			{
				cols_Meq += 1;
				if (all_handle_list[i].active==true)
				{
					rows_known += 1;
					rows_Meq += 0;
				}
			}
			break;
		case HandleTypePointFrame:
		case HandleTypeVertices:
		case HandleTypeBoneEdge:
			{
				cols_Meq += (dim==2)?3:4;// this has nothing to do with case 1 or case 2;
				if (all_handle_list[i].active==true)
				{	
					bool active_case_one = !all_handle_list[i].b_constrain_affine_trans;
					if(active_case_one)
					{// case 1: only specifying the position of active handles
						rows_known += 0;
						rows_Meq += 1;
					}
					else
					{// case 2: specifying the transformation of active handles
						rows_known += dim+1;
						rows_Meq += 0;
					}
				}
			}
			break;		
		//case HandleTypeBoneEdge:
			{
				cols_Meq += (dim==2)?3:4;// this has nothing to do with case 1 or case 2;
				if (all_handle_list[i].active==true)
				{	
					bool active_case_one = !all_handle_list[i].b_constrain_affine_trans;
					if(active_case_one)
					{// case 1: only specifying the position of bones
						rows_known += 0;
						rows_Meq += 1;
					}
					else
					{// case 2: specifying the two end of bones
						rows_known += 0;
						rows_Meq += 2;
					}
				}
			}
			break;
		}
	}

	// Second loop, writing into constaints
	// TODO: change the way how M is calculated for HS
	//const Eigen::MatrixXd& M = DeformSkinning::GetReference().GetM(dim);
	
	//assert(M.cols()==cols_Meq);
	if(dim==3)
		assert(M3d.cols()==cols_Meq);
	else
		assert(M2d.cols()==cols_Meq);

	known.resize(rows_known);
	knownValue.resize(rows_known,dim);
	Meq.resize(rows_Meq,cols_Meq);
	Peq.resize(rows_Meq,dim);

	int current_known = 0;
	int current_Meq = 0;
	int current_var = 0;
	for (int i=0; i<all_handle_list.size(); i++)
	{
		switch(all_handle_list[i].type())
		{
		case HandleTypePoint:
			{
				if (all_handle_list[i].active==true)
				{
					known(current_known) = current_var;
					knownValue.row(current_known) = all_handle_list[i].CenterPos().leftCols(dim);

					current_known += 1;
					current_Meq += 0;
				}
				current_var += 1;
			}
			break;
		case HandleTypePointFrame:
		case HandleTypeVertices:
		case HandleTypeBoneEdge:
			{
				if (all_handle_list[i].active==true)
				{	
					bool active_case_one = !all_handle_list[i].b_constrain_affine_trans;
					if(active_case_one)
					{// case 1: only specifying the position of active handles
						// TODO: this could be optimized by precomputing when set up all_handle_list.
						// And this is based on the assumption that the specified position is average
						// position, should be consistent with what is in centerPos.
						Eigen::MatrixXd new_Meq_row = Eigen::MatrixXd::Zero(1,cols_Meq);
						const Eigen::VectorXi& I = all_handle_list[i].IndexInHandles(); //IndexInMesh();
						const Eigen::MatrixXd& P = all_handle_list[i].CenterPos().leftCols(dim);
						for (int k=0; k<I.rows(); k++)
						{
							if(dim==3)
								new_Meq_row += M3d.row(I(k));//M.row(I(k));
							else
								new_Meq_row += M2d.row(I(k));//M.row(I(k));
						}
						new_Meq_row *= 1.0/I.rows();
						Meq.row(current_Meq) = new_Meq_row;
						Peq.row(current_Meq) = P;

						current_known += 0;
						current_Meq += 1;
					}
					else
					{// case 2: specifying the transformation of active handles
						for (int k=0; k<dim+1; k++)
							known(current_known+k) = current_var+k;
						knownValue.block(current_known,0,dim+1,dim) = all_handle_list[i].Trans(dim);//TODO: consider 2D as well.

						current_known += dim+1;
						current_Meq += 0;
					}
				}
				current_var += dim+1;
			}
			break;
		//case HandleTypeBoneEdge:
			{
				if (all_handle_list[i].active==true)
				{	
					bool active_case_one = !all_handle_list[i].b_constrain_affine_trans;
					if(active_case_one)
					{// case 1: only specifying the position of active handles
						// TODO: this could be optimized by precomputing when set up all_handle_list.
						// And this is based on the assumption that the specified position is average
						// position, should be consistent with what is in centerPos.
						Eigen::MatrixXd new_Meq_row = Eigen::MatrixXd::Zero(1,cols_Meq);
						const Eigen::VectorXi& I = all_handle_list[i].IndexInHandles(); //IndexInMesh();
						const Eigen::MatrixXd& P = all_handle_list[i].CenterPos().leftCols(dim);
						for (int k=0; k<I.rows(); k++)
						{
							if(dim==3)
								new_Meq_row += M3d.row(I(k));//M.row(I(k));
							else
								new_Meq_row += M2d.row(I(k));//M.row(I(k));
						}
						new_Meq_row *= 1.0/I.rows();
						Meq.row(current_Meq) = new_Meq_row;
						Peq.row(current_Meq) = P;

						current_known += 0;
						current_Meq += 1;
					}
					else
					{// case 2: specifying the transformation of active handles
						Eigen::MatrixXd new_Meq_row = Eigen::MatrixXd::Zero(2,cols_Meq);
						const Eigen::VectorXi& I = all_handle_list[i].IndexInHandles(); //IndexInMesh();
						const Eigen::MatrixXd& P = all_handle_list[i].CenterPos().leftCols(dim);
						// Assuming that the two end are always stored at first two rows.
						for (int k=0; k<2; k++)
						{
							if(dim==3)
								new_Meq_row.row(k) += M3d.row(I(k));//M.row(I(k));
							else
								new_Meq_row.row(k) += M2d.row(I(k));//M.row(I(k));
						}
						
						Meq.row(current_Meq) = new_Meq_row.row(0);
						Peq.row(current_Meq) = P.row(0);
						Meq.row(current_Meq+1) = new_Meq_row.row(1);
						Peq.row(current_Meq+1) = P.row(1);

						current_known += 0;
						current_Meq += 2;
					}
				}
				current_var += dim+1;
			}
			break;
		}
	}

	return true;
}

void boundary_condition(const Eigen::VectorXi& HG, Eigen::MatrixXi& BC)
{
	int cols = 0;
	if(HG.rows()>0)
	{
		cols = HG.maxCoeff()+1;
	}
	assert(cols<=HG.rows());
	BC = Eigen::MatrixXi::Zero(HG.rows(),cols);
	for (int i=0; i<HG.rows(); i++)
	{
		BC(i,HG(i)) = 1;
	}
}


void HandleStructure::get_boundary_condition(Eigen::MatrixXi& BC)
{
	boundary_condition(HandleGroup,BC);
}

Eigen::MatrixXd HandleStructure::centerPos()
{
	int h = all_handle_list.size();
	Eigen::MatrixXd CP(h,3);
	for (int i=0; i<h; i++)
	{
		CP.row(i) = all_handle_list[i].CenterPos();
	}
	return CP;
}


template <typename DerivedV, typename DerivedT, typename DerivedF, typename DerivedE, typename DerivedP>
inline void HandleStructure::exportHANDLE(
	// outputs:
	Eigen::PlainObjectBase<DerivedV> & V, 
	Eigen::PlainObjectBase<DerivedT> & T,
	Eigen::PlainObjectBase<DerivedF> & F,
	Eigen::PlainObjectBase<DerivedE> & E,
	Eigen::PlainObjectBase<DerivedP> & P,
	Eigen::PlainObjectBase<DerivedE> & BE,
	Eigen::PlainObjectBase<DerivedF> & CF,
	std::vector<std::vector<Eigen::VectorXi>>& G)
{
	const int h = all_handle_list.size();

	int rows_of_v = 0;
	int num_of_points = 0;
	int num_of_group_points = 0;
	int num_of_group_boneedges = 0;
	for (int i=0; i<h; i++)
	{
		rows_of_v += all_handle_list[i].Pos().rows();

		switch(all_handle_list[i].type())
		{
		case HandleTypePoint:
			{
				num_of_points++;
			}
			break;
		case HandleTypePointFrame:
			{

			}
			break;
		case HandleTypeVertices:
			{
				num_of_group_points++;
			}
			break;
		case HandleTypeBoneEdge:
			{
				num_of_group_boneedges++;
			}
			break;
		}
	}

	V.resize(rows_of_v,3);
	T.resize(0,4);
	F.resize(0,3);
	E.resize(0,2);
	P.resize(num_of_points,1);
	BE.resize(num_of_group_boneedges,2);
	CF.resize(0,3);

	G.clear();
	std::vector<Eigen::VectorXi> GP;
	std::vector<Eigen::VectorXi> GBE;
	std::vector<Eigen::VectorXi> GCF;

	int current_rows_of_v = 0;
	int current_points = 0;
	int current_group_points = 0;
	int current_group_boneedges = 0;
	for (int i=0; i<h; i++)
	{
		const Eigen::MatrixXd& Pos = all_handle_list[i].Pos();

		V.block(current_rows_of_v,0,Pos.rows(),3) = Pos;

		switch(all_handle_list[i].type())
		{
		case HandleTypePoint:
			{
				P(current_points,0) = current_rows_of_v;
				current_points++;
			}
			break;
		case HandleTypePointFrame:
			{

			}
			break;
		case HandleTypeVertices:
			{
				Eigen::VectorXi new_GP = NaturalSeq(Pos.rows(),current_rows_of_v);
				GP.push_back(new_GP);
				current_group_points++;
			}
			break;
		case HandleTypeBoneEdge:
			{
				BE(current_group_boneedges,0) = current_rows_of_v;
				BE(current_group_boneedges,1) = current_rows_of_v+1;
				Eigen::VectorXi new_GBE = NaturalSeq(Pos.rows(),current_rows_of_v);
				GBE.push_back(new_GBE);
				current_group_boneedges++;
			}
			break;
		}

		current_rows_of_v += Pos.rows();

	}
	
	G.push_back(GP);
	G.push_back(GBE);
	G.push_back(GCF);
}

bool HandleStructure::writeToFile(const std::string str)
{
	Eigen::MatrixXd V;
	Eigen::MatrixXi T;
	Eigen::MatrixXi F;
	Eigen::MatrixXi E;
	Eigen::MatrixXi P;
	Eigen::MatrixXi BE;
	Eigen::MatrixXi CF;
	std::vector<std::vector<Eigen::VectorXi>> G;

	exportHANDLE(V,T,F,E,P,BE,CF,G);

	return writeHANDLE(str,V,T,F,E,P,BE,CF,G);
}

//#include <igl\xml\XMLSerializer.h>
bool HandleStructure::exportState()
{
	return false;


	//std::string name = "HandleStruct State";
	//igl::XMLSerializer* s = new ::igl::XMLSerializer(name);

	//std::vector<char**> buffer;

	//for (int i = 0; i < all_handle_list.size(); i++)
	//{
	//	std::string val = bar->get_value_as_string(it->var, it->type);
	//	char** cval = new char*; // create char* on heap
	//	*cval = new char[val.size() + 1];
	//	buffer.push_back(cval);
	//	strcpy(*cval, val.c_str());
	//	s->Add(*cval, it->name);
	//}
}