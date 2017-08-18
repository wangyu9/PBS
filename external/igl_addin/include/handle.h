#ifndef IGL_ADDIN_HANDLE_H
#define IGL_ADDIN_HANDLE_H

#include "types.h"
#include <string>
#include <Eigen/Core>

#include <eigen_helper.h>
#include <matrix_spline.h>

typedef enum {HandleTypePoint, HandleTypePointFrame, HandleTypeVertices, HandleTypeBoneEdge, HandleTypeSpline} HandleType;

class Point3D{
public:
	double x,y,z;
	int index;
	Point3D() : Point3D(0., 0., 0., 0) {}
	Point3D(double xx, double yy, double zz):x(xx),y(yy),z(zz),index(-1){}
	Point3D(double xx, double yy, double zz, int ii):x(xx),y(yy),z(zz),index(ii){}
};

class handleData{

/****************************Data*******************************/

private:
	// This four variables should never be changed after initialization
	HandleType handleType;
	Eigen::MatrixXd* restPos;
	Eigen::MatrixXd* centerRestPos;
	Eigen::MatrixXd* M3d;
	Eigen::MatrixXd* M2d;
protected:
	Eigen::MatrixXd* pos;
	Eigen::MatrixXd* centerPos;
	Eigen::VectorXi* I;//Index in all vertices of the mesh
	Eigen::VectorXi* IH;//Index in all Handles
	Eigen::MatrixXd* T;
	Eigen::MatrixXd* T2d;
	Eigen::MatrixXd* Visualization;
	Eigen::MatrixXd* temp_T;

	bool has_set_rot_center;
	Eigen::MatrixXd* rotCenter;
	Eigen::MatrixXd* rotRestCenter;
public:
	bool selected;
	bool active;
	
	// data for different type of handle
	Eigen::MatrixXi* BE;
	//
	bool b_constrain_affine_trans; // if true, for active non-pure-point handles the entire affine transformation is constrained, in the function get_constraint() of HandleStructre

	// For spline type
	int spline_order;
	int spline_nodes;
	int spline_points;
	Eigen::MatrixXd* Nodes;
	Eigen::MatrixXd* Nodes_t;
	Eigen::MatrixXd* Points_t;
	MatrixCubicSpline* matrixSpline;
/**************************Functions*******************************/

public:
	std::string string_type()
	{
		std::string s;
		switch(handleType)
		{
		case HandleTypePoint:
			s = "HandleTypePoint";
			break;
		case HandleTypePointFrame:
			s = "HandleTypePointFrame";
			break;
		case HandleTypeVertices:
			s = "HandleTypeVertices";
			break;
		case HandleTypeBoneEdge:
			s = "HandleTypeBoneEdge";
			break;
		}
		return s;
	}
	void print_pos()
	{
		printf("Handle Type %s.\n", string_type());
	}

	double x() const
	{
		return (*centerPos)(0,0);
	}
	double y() const
	{
		return (*centerPos)(0,1);
	}
	double z() const
	{
		return (*centerPos)(0,2);
	}
	//double index() const
	//{
	//	assert(handleType==HandleTypePoint);
	//	return (*I)(0);
	//}
	Eigen::MatrixXd DefaultRotCenter() const
	{
		Eigen::MatrixXd drc(1,3);
		switch(handleType)
		{
		case HandleTypePoint:
			{
				drc = CenterPos();
			}
			break;
		case HandleTypePointFrame:
			{
				drc = CenterPos();
			}
			break;
		case HandleTypeVertices:
			{
				drc = CenterPos();
			}
			break;
		case HandleTypeBoneEdge:
			{
				drc = pos->row(0);
			}
			break;
		}
		return drc;
	}
	Eigen::MatrixXd getRotCenter() const
	{
		if (!has_set_rot_center)
			return DefaultRotCenter();

		Eigen::MatrixXd drc(1, 3);
		drc = *rotCenter;
		
		return drc;
	}
	void setRotCenter(const Eigen::MatrixXd& rc)
	{
		assert(rc.rows()==1&&rc.cols()==3);
		has_set_rot_center = true;
		(*rotCenter) = (*rotRestCenter) = rc;
	}
	Eigen::MatrixXd& Trans(int dim) const
	{
		return (dim==3)?(*T):(*T2d);
	}
	Eigen::MatrixXd CenterPos() const
	{
		return (*centerPos);
	}
	Eigen::MatrixXd CenterRestPos() const
	{
		return (*centerRestPos);
	}
	Eigen::MatrixXd Pos() const
	{
		return (*pos);
	}
	Eigen::VectorXi IndexInMesh() const
	{
		return (*I);
	}
	Eigen::VectorXi IndexInHandles() const
	{
		return (*IH);
	}
	Eigen::MatrixXd& Mi(int dim) const
	{
		return (dim==3)?(*M3d):(*M2d);
	}
	void SetVis(const Eigen::MatrixXd& new_vis)
	{
		*Visualization = new_vis;
	}
	Eigen::MatrixXd& GetVis() const
	{
		return *Visualization;
	}
public:
	const HandleType& type() const{
		return handleType;
	}

	//static handleData FramePoint(const Point3D& P)
	//{
	//	return FramePoint(P.x, P.y, P.z, P.index);
	//}
	//static handleData FramePoint(const double x, const double y, const double z, const int index)
	//{
	//	Eigen::MatrixXd P(1,3);
	//	Eigen::VectorXi I(1);
	//	P(0,0) = x;
	//	P(0,1) = y;
	//	P(0,2) = z;
	//	I(0) = index;
	//	return handleData(P, HandleTypePointFrame, I);
	//}
private:
	static handleData GroupType(const HandleType ht, const Eigen::MatrixXd& rest_pos, const Eigen::VectorXi& index_in_handle)
	{
		assert(ht!=HandleTypePoint);
		Eigen::VectorXi index_in_mesh(index_in_handle.rows());
		index_in_mesh.setConstant(-1);
		return GroupType(ht, rest_pos, index_in_handle, index_in_mesh);
	}
	static handleData GroupType(const HandleType ht, const Eigen::MatrixXd& rest_pos, const Eigen::VectorXi& index_in_handle, const Eigen::VectorXi& index_in_mesh)
	{
		return handleData(rest_pos, ht, index_in_handle, index_in_mesh);
	}
public:
	static handleData BoneEdge(const Eigen::MatrixXd& rest_pos, const Eigen::VectorXi& index_in_handle)
	{
		return GroupType(HandleType::HandleTypeBoneEdge, rest_pos, index_in_handle);
	}
	static handleData BoneEdge(const Eigen::MatrixXd& rest_pos, const Eigen::VectorXi& index_in_handle, const Eigen::VectorXi& index_in_mesh)
	{
		return GroupType(HandleType::HandleTypeBoneEdge, rest_pos, index_in_handle, index_in_mesh);
	}
	static handleData Vertices(const Eigen::MatrixXd& rest_pos, const Eigen::VectorXi& index_in_handle)
	{
		return GroupType(HandleType::HandleTypeVertices, rest_pos, index_in_handle);
	}
	static handleData Vertices(const Eigen::MatrixXd& rest_pos, const Eigen::VectorXi& index_in_handle, const Eigen::VectorXi& index_in_mesh)
	{
		return GroupType(HandleType::HandleTypeVertices, rest_pos, index_in_handle, index_in_mesh);
	}
	static handleData FramePoint(const Eigen::MatrixXd& rest_pos, const Eigen::VectorXi& index_in_handle)
	{
		return GroupType(HandleType::HandleTypePointFrame, rest_pos, index_in_handle);
	}
	static handleData FramePoint(const Eigen::MatrixXd& rest_pos, const Eigen::VectorXi& index_in_handle, const Eigen::VectorXi& index_in_mesh)
	{
		return GroupType(HandleType::HandleTypePointFrame, rest_pos, index_in_handle, index_in_mesh);
	}

	static handleData Point(const Point3D& P, const int index_in_handle)
	{
		return Point(P.x, P.y, P.z, index_in_handle, P.index);
	}
	static handleData Point(const double x, const double y, const double z, const int index_in_handle, const int index)
	{
		Eigen::MatrixXd P(1,3);
		Eigen::VectorXi IH(1);
		Eigen::VectorXi I(1);
		P(0,0) = x;
		P(0,1) = y;
		P(0,2) = z;
		I(0) = index;
		IH(0) = index_in_handle;
		return handleData(P, HandleTypePoint, IH, I);
	}
	static handleData Point(const Eigen::MatrixXd& rest_pos, const Eigen::VectorXi& index_in_handle)
	{
		Eigen::VectorXi index_in_mesh(index_in_handle.rows());
		index_in_mesh.setConstant(-1);
		return Point(rest_pos,index_in_handle,index_in_mesh);
	}
	static handleData Point(const Eigen::MatrixXd& rest_pos, const Eigen::VectorXi& index_in_handle, const Eigen::VectorXi& index_in_mesh)
	{
		return handleData(rest_pos, HandleType::HandleTypePoint, index_in_handle, index_in_mesh);
	}

	void setIndices(const Eigen::VectorXi new_I)
	{
		assert((*I).rows()==new_I.rows());
		(*I) = new_I;
	}
	//void setCenterPos(const Eigen::MatrixXd& new_centerPos)// moving the handle as a whole by setting its center Pos
	//{
	//	assert(new_centerPos.rows() == 1 && new_centerPos.cols() == 3);
	//	Eigen::MatrixXd new_Pos = *pos;
	//	for (int i = 0; i < new_Pos.rows(); i++)
	//	{
	//		new_Pos.row(i) = new_Pos.row(i) + new_centerPos - (*centerPos);
	//	}
	//	setPos(new_Pos);
	//}
	void setCenterPos(const Eigen::MatrixXd& new_Pos, bool abs_pos = true)// this should be called new_centerPos maybe, // if abs_pos is true, the pose is absolute position, if false, it is displacement w.r.t each handle's rest pose center
	{
		assert(new_Pos.rows() == 1 && new_Pos.cols() == 3);

		Eigen::MatrixXd NP = new_Pos;
		if (!abs_pos)
		{
			NP = NP + (*centerRestPos);
		}

		if (true)
		{
			if (handleType==HandleTypePoint)
			{
				updateFromPoint3D(NP(0, 0), NP(0, 1), NP(0, 2));
			}
			else
			{
				Eigen::MatrixXd new_Trans = *T;
				new_Trans.block(3, 0, 1, 3) = new_Trans.block(3, 0, 1, 3) + NP - (*centerPos);
				updateTrans(new_Trans);
			}	
		} 
		else
		{
			// old implementation TO remove, not consider NP change

			*pos = new_Pos;
			*centerPos = Rows_Average(*pos);

			(*rotCenter) = (*centerPos) - (*centerRestPos) + (*rotRestCenter);

			switch (handleType)
			{
			case HandleTypePoint:
			{
				(*T).block(0, 0, 1, 3) = *centerPos;
				(*T2d).block(0, 0, 1, 2) = centerPos->leftCols(2);
			}
			break;
			case HandleTypePointFrame:
			case HandleTypeVertices:
			case HandleTypeBoneEdge:
				break;
			}
		}
	
	}
	void getCenterPos(Eigen::MatrixXd& cP, bool abs_pos = true)// this should be called new_centerPos maybe, // if abs_pos is true, the pose is absolute position, if false, it is displacement w.r.t each handle's rest pose 
	{
		cP = *centerPos;
		if (!abs_pos)
		{
			cP = cP - *centerRestPos;
		}
	}
	void setRestPos(const Eigen::MatrixXd& new_RestPos)
	{
		*restPos = new_RestPos;
		*centerRestPos = Rows_Average(*restPos);

		(*rotRestCenter) = (*centerPos) - (*centerRestPos) + (*rotRestCenter);
	}
	Point3D toPoint3D()
	{
		assert(handleType==HandleTypePoint);
		Point3D P((*pos)(0,0),(*pos)(0,1),(*pos)(0,2),(*I)(0));
		return P;
	}
	void updateFromPoint3D(const Point3D& P)
	{
		updateFromPoint3D(P.x,P.y,P.z);
		(*I)(0) = P.index;
	}
	void updateFromPoint3D(const double x, const double y, const double z)
	{
		assert(handleType==HandleTypePoint);
		(*T)(0,0) = (*pos)(0,0) = (*centerPos)(0,0) = x;
		(*T)(0,1) = (*pos)(0,1) = (*centerPos)(0,1) = y;
		(*T)(0,2) = (*pos)(0,2) = (*centerPos)(0,2) = z;
		(*T2d)(0,0) = (*T)(0,0);
		(*T2d)(0,1) = (*T)(0,1);
		(*rotCenter) = (*centerPos) - (*centerRestPos) + (*rotRestCenter); // should be equavalent to: (*rotCenter) = (*pos) - (*restPos) + (*rotRestCenter);
	}

	static void extract_2d_trans_from_3d(const Eigen::MatrixXd& T4x3, Eigen::MatrixXd& T3x2)
	{
		assert(T4x3.rows()%4==0);
		assert(T4x3.cols()==3);

		int num = T4x3.rows()/4;
		T3x2.resize(3*num,2);
		T3x2.setZero();

		for (int i=0; i<num; i++)
		{
			T3x2.block(3*i,0,2,2) = T4x3.block(4*i,0,2,2);
			T3x2.block(3*i+2,0,1,2) = T4x3.block(4*i+3,0,1,2);
		}
	}

	static void extract_2d_spline_from_3d_wrong(const Eigen::MatrixXd& T3, Eigen::MatrixXd& T2)
	{
		assert(T3.cols()==3);

		T2 = T3.leftCols<2>();
	}

	static void extract_2d_spline_from_3d(const Eigen::MatrixXd& T3, Eigen::MatrixXd& T2)
	{
		assert(T3.cols() == 3);

		T2 = T3.leftCols<2>();
	}

	static void set_spline_nodes_wrong(const Eigen::MatrixXd& N, int dim, int order, Eigen::MatrixXd& TT)
	{
		assert(dim == 2 || dim == 3);
		assert(dim == N.cols() );

		int nodes = N.rows();

		TT = Eigen::MatrixXd::Ones( nodes*(order+1), dim);

		for (int i = 0; i < order+1; i++)
		{
			for (int j = i; j > 0; j--)
			{
				Eigen::MatrixXd temp = TT.block(i*nodes, 0, nodes, dim);
				TT.block(i*nodes, 0, nodes, dim) = temp.cwiseProduct(N);
			}
		}
	}

	void resumeToRest()
	{
		if (handleType==HandleTypePoint)
		{
			Eigen::MatrixXd p = (*centerRestPos);
			setPointPos(p);
		} 
		else
		{
			Eigen::MatrixXd NT = Eigen::MatrixXd::Zero(4,3);
			NT(0, 0) = NT(1, 1) = NT(2, 2) = 1.;
			updateTrans(NT);
		}
	}

	void updateTrans(const Eigen::MatrixXd new_Trans)
	{
		assert(handleType!=HandleTypePoint);

		assert(new_Trans.rows()==4);
		assert(new_Trans.cols()==3);

		*T = new_Trans;
		Eigen::MatrixXd new_Trans_2d;
		extract_2d_trans_from_3d(new_Trans,new_Trans_2d);
		*T2d = new_Trans_2d;

		*pos = (*M3d)*(*T);
		*rotCenter = (*rotRestCenter)*(*T).block(0,0,3,3)+(*T).block(3,0,1,3);

		Eigen::MatrixXd new_centerPos = Eigen::MatrixXd::Zero(1,3);
		for (int i=0; i<pos->rows(); i++)
		{
			new_centerPos = new_centerPos + pos->row(i);
		}

		*centerPos = new_centerPos * 1.0 / pos->rows();
	}
	void setRotationAroundCenter(const Eigen::MatrixXd& Rot)
	{
		setRotation(Rot, (*centerRestPos));
	}
	void setRotationAroundFristP(const Eigen::MatrixXd& Rot)
	{
		setRotation(Rot, pos->row(0));
	}
	void setRotation(const Eigen::MatrixXd& Rot, const Eigen::MatrixXd& C)
	{
		assert(handleType!=HandleTypePoint);

		assert(Rot.rows()==3);
		assert(Rot.cols()==3);

		switch(handleType)
		{
		case HandleTypePoint:
			{
				printf("Error: there is no rotation for HandleTypePoint!\n");

				break;
			}
		case HandleTypePointFrame:
		case HandleTypeVertices:
		case HandleTypeBoneEdge:
			{
				// T1 and T2 are transpose of transformation, just like T is the transpose of the affine transformation.
				
				Eigen::MatrixXd T1 = Eigen::MatrixXd::Identity(4,4);
				Eigen::MatrixXd T2 = Eigen::MatrixXd::Identity(4,4);

				T1.block(0,0,4,3) = *temp_T;

				T2.block(0,0,3,3) = Rot.transpose();
				T2.block(3,0,1,3) = C-C*Rot.transpose();//(*centerPos)-(*centerRestPos)*Rot.transpose();

				Eigen::MatrixXd T3 = T1 * T2;// This is reverse order of transformation multiplication

				*T = T3.block(0,0,4,3);
				
				printf("Warning: not exactly correct!!!\n");
				Eigen::MatrixXd new_Trans_2d;
				extract_2d_trans_from_3d(*T,new_Trans_2d);
				*T2d = new_Trans_2d;

				//print_matlab("T3d",*T);
				//print_matlab("T2d",*T2d);

				break;
			}
		}

		(*pos) = (*M3d)*(*T);// This is necessary!
		*rotCenter = (*rotRestCenter)*(*T).block(0, 0, 3, 3) + (*T).block(3, 0, 1, 3);

		Eigen::MatrixXd new_centerPos = Eigen::MatrixXd::Zero(1,3);
		for (int i=0; i<pos->rows(); i++)
		{
			new_centerPos = new_centerPos + pos->row(i);
		}

		*centerPos = new_centerPos * 1.0 / pos->rows();
	}
	void setRotation_old(const Eigen::MatrixXd& Rot, const Eigen::MatrixXd& C)
	{
		assert(handleType!=HandleTypePoint);

		assert(Rot.rows()==3);
		assert(Rot.cols()==3);

		switch(handleType)
		{
		case HandleTypePoint:
			{
				printf("Error: there is no rotation for HandleTypePoint!\n");

				break;
			}
		case HandleTypePointFrame:
		case HandleTypeVertices:
		case HandleTypeBoneEdge:
			{
				
				(*T).block(0,0,3,3) = Rot.transpose();
				(*T).block(3,0,1,3) = (*centerPos)-C*Rot.transpose();//(*centerPos)-(*centerRestPos)*Rot.transpose();

				printf("Warning: not exactly correct!!!\n");
				Eigen::MatrixXd new_Trans_2d;
				extract_2d_trans_from_3d(*T,new_Trans_2d);
				*T2d = new_Trans_2d;

				//print_matlab("T3d",*T);
				//print_matlab("T2d",*T2d);

				break;
			}
		}

		(*pos) = (*M3d)*(*T);// This is necessary!
		*rotCenter = (*rotRestCenter)*(*T).block(0, 0, 3, 3) + (*T).block(3, 0, 1, 3);

		Eigen::MatrixXd new_centerPos = Eigen::MatrixXd::Zero(1,3);
		for (int i=0; i<pos->rows(); i++)
		{
			new_centerPos = new_centerPos + pos->row(i);
		}

		*centerPos = new_centerPos * 1.0 / pos->rows();
	}
	// save as translate, but set absolute position instead relative displacement
	void setPointPos(const Eigen::MatrixXd& t)
	{
		assert(handleType==HandleTypePoint);

		float tt[3];
		tt[0] = t(0, 0);// +(*restPos)(0, 0);// -(*pos)(0, 0);
		tt[1] = t(0, 1);// +(*restPos)(0, 1);// -(*pos)(0, 1);
		tt[2] = t(0, 2);// +(*restPos)(0, 2);// -(*pos)(0, 2);
		setPointPos(tt);
	}
	// translate the handle for a given value
	void translate(const Eigen::MatrixXd& t)// notice the t is the delta t, which means the new relative displacement, not abs position
	{
		float tt[3];
		tt[0] = t(0,0);
		tt[1] = t(0,1);
		tt[2] = t(0,2);
		translate(tt);
	}
	void translate(float* t)// notice the t is the delta t, which means the new relative displacement, not abs position
	{
		Eigen::MatrixXd disp(1,3);
		disp(0,0) = t[0];
		disp(0,1) = t[1];
		disp(0,2) = t[2];
		switch(handleType)
		{
		case HandleTypePoint:
			{
				(*T).block(0,0,1,3) += disp;
				(*T2d).block(0,0,1,2) += disp.leftCols(2);
				(*pos) += disp;// This is equivalent to *pos = (*M)*(*T);
				(*centerPos) += disp;
				(*rotCenter) += disp;
				break;
			}
		case HandleTypePointFrame:
		case HandleTypeVertices:
		case HandleTypeBoneEdge:
			{
				(*T).block(3,0,1,3) += disp;

				//printf("Warning: not exactly correct!!!\n");
				Eigen::MatrixXd new_Trans_2d;
				extract_2d_trans_from_3d(*T,new_Trans_2d);
				*T2d = new_Trans_2d;

				for (int i=0; i<pos->rows(); i++)
				{
					pos->row(i) += disp;// This is equivalent to *pos = (*M)*(*T);
				}
				(*centerPos) += disp;
				(*rotCenter) += disp;
				break;
			}
		}
		commit_trans();
	}
	void setPointPos(float* t)// notice the t is the delta t, which means the new relative displacement, not abs position
	{
		assert(handleType == HandleTypePoint);

		Eigen::MatrixXd disp(1, 3);
		disp(0, 0) = t[0];
		disp(0, 1) = t[1];
		disp(0, 2) = t[2];

		(*T).block(0, 0, 1, 3) = disp;
		(*T2d).block(0, 0, 1, 2) = disp.leftCols(2);
		(*pos) = disp;// This is equivalent to *pos = (*M)*(*T);
		(*centerPos) = disp;
		(*rotCenter) = (*rotRestCenter) + disp;// wangyu this should be wrong
		commit_trans();
	}
	~handleData()
	{
		//TODO: figure out how to do this for eigen

		//delete restPos;
		//delete centerRestPos;
		//delete M3d;
		//delete M2d;
		//delete pos;
		//delete centerPos;
		//delete I;
		//delete IH;
		//delete T;
		//delete T2d;
	}
	void commit_trans()
	{
		*temp_T = *T;
	}
private:
	handleData(const Eigen::MatrixXd& rest_pos, const HandleType ht, const Eigen::VectorXi& index_in_handle, const Eigen::VectorXi& index_in_mesh, const int s_order = 3, const Eigen::MatrixXd SN = Eigen::MatrixXd(0,4), const Eigen::MatrixXd Pt = Eigen::MatrixXd(0,1) )
		:restPos(NULL),pos(NULL),selected(false),active(true),I(NULL),IH(NULL)
	{
		handleType = ht;

		restPos = new Eigen::MatrixXd();
		centerRestPos = new Eigen::MatrixXd();
		M3d = new Eigen::MatrixXd();
		M2d = new Eigen::MatrixXd();
		pos = new Eigen::MatrixXd();
		centerPos = new Eigen::MatrixXd();
		I = new Eigen::VectorXi();
		IH = new Eigen::VectorXi();
		T = new Eigen::MatrixXd();
		temp_T = new Eigen::MatrixXd();
		T2d = new Eigen::MatrixXd();
		Visualization = new Eigen::MatrixXd();
		rotCenter = new Eigen::MatrixXd();
		rotRestCenter = new Eigen::MatrixXd();
		has_set_rot_center = false;

		BE = new Eigen::MatrixXi();
		b_constrain_affine_trans = true;

		*restPos = rest_pos;
		*pos = rest_pos;
		*I = index_in_mesh;
		*IH = index_in_handle;

		assert(SN.cols()==4);

		Nodes = new Eigen::MatrixXd();
		*Nodes = SN.leftCols<3>();
		Nodes_t = new Eigen::MatrixXd();
		*Nodes_t = SN.col(3);
		Points_t = new Eigen::MatrixXd();
		*Points_t = Pt;

		spline_order = s_order;
		spline_points = Pt.rows();
		spline_nodes = Nodes->rows();
		matrixSpline = new MatrixCubicSpline();

		init();
	}
	void init()
	{

		switch(handleType)
		{
		case HandleTypePoint:
			{
				assert(restPos->rows()==1);;

				*T = Eigen::MatrixXd::Zero(1,3);
				*T = (*restPos).row(0);

				*temp_T = Eigen::MatrixXd();
				*temp_T = *T;

				*T2d = Eigen::MatrixXd::Zero(1,2);
				*T2d = (*T).leftCols(2);

				*M3d = Eigen::MatrixXd::Ones(1,1);  
				*M2d = Eigen::MatrixXd::Ones(1,1);

				assert(restPos->rows()==1);
				*centerPos = *centerRestPos = *restPos;

				break;
			}
		case HandleTypePointFrame:
		case HandleTypeVertices:
		case HandleTypeBoneEdge:
			{
				
				*T = Eigen::MatrixXd::Zero(4,3);
				(*T)(0,0) = (*T)(1,1) = (*T)(2,2) = 1;

				*temp_T = Eigen::MatrixXd();
				*temp_T = *T;

				//printf("Warning: not exactly correct!!!\n");
				*T2d = Eigen::MatrixXd::Zero(3,2);
				Eigen::MatrixXd new_Trans_2d;
				extract_2d_trans_from_3d(*T,new_Trans_2d);
				*T2d = new_Trans_2d;

				int n = restPos->rows();
				M3d->resize(n,4);
				(*M3d).block(0,0,n,3) = *restPos;
				(*M3d).block(0,3,n,1) = Eigen::MatrixXd::Ones(n,1);
				M2d->resize(n,3);
				(*M2d).block(0,0,n,2) = (*restPos).leftCols<2>();
				(*M2d).block(0,2,n,1) = Eigen::MatrixXd::Ones(n,1);

				assert(restPos->rows()>0);
				*centerPos = Eigen::MatrixXd::Zero(1,3);
				for (int i=0; i<restPos->rows(); i++)
				{
					(*centerPos).block(0,0,1,3) += (*restPos).block(i,0,1,3);
				}
				*centerPos *= 1.0/restPos->rows();
				*centerRestPos = *centerPos;

				break;
			}
		case HandleTypeSpline:
			{


#if 0
				assert(Nodes_t->rows() == restPos->rows());

				//int default_nodes = spline_nodes;
				//int default_order = spline_order;

				//*T = Eigen::MatrixXd::Zero(spline_order*spline_nodes*3+1, 3);

				set_spline_nodes(*Nodes, 3, spline_order, *T);

				*temp_T = Eigen::MatrixXd();
				*temp_T = *T;

				set_spline_nodes(*Nodes, 2, spline_order, *T2d);

				matrixSpline->clear();
				for (int i = 0; i < spline_nodes; i++)
				{
					matrixSpline->append((*Nodes_t)(i));
					matrixSpline->append_matrix((*Nodes).row(i,0));
				}

				matrixSpline->solve_coeffs();
				matrixSpline->build_matrix_coeffs();

				matrixSpline->print_coeffs();

				int m_cols = spline_nodes*(spline_order+1);


				int n = restPos->rows();
				assert(n == spline_points);

				M3d->resize(n, m_cols);
				M2d->resize(n, m_cols);

				for (int i = 0; i < spline_points; i++)
				{
					/*Eigen::MatrixXd D, C, B, A;
					matrixSpline->get_interpolate_matrix( (*Nodes_t)(i), D, C, B, A);
					M3d->block(i, 0 * 3, 1, 3) = D;
					M3d->block(i, 1 * 3, 1, 3) = C;
					M3d->block(i, 2 * 3, 1, 3) = B;
					M3d->block(i, 3 * 3, 1, 3) = A;*/

					int index = matrixSpline->get_index( (*Nodes_t)(i) );
				}

#else
				assert(Nodes_t->rows() == restPos->rows());

				*T = *restPos;

				*temp_T = Eigen::MatrixXd();
				*temp_T = *T;

				*T2d = (*T).leftCols<2>();

				matrixSpline->clear();
				for (int i = 0; i < spline_nodes; i++)
				{
					matrixSpline->append((*Nodes_t)(i));
					//matrixSpline->append_matrix((*Nodes).row(i, 0));
				}

				matrixSpline->solve_coeffs();
				//matrixSpline->build_matrix_coeffs();

				matrixSpline->print_coeffs();

				int m_cols = spline_nodes;

				int n = restPos->rows();
				assert(n == spline_points);

				M3d->resize(n, m_cols);
				M2d->resize(n, m_cols);

				for (int i = 0; i < spline_points; i++)
				{
					double ti = (*Nodes_t)(i);
					int index = matrixSpline->get_index(ti);
					Eigen::MatrixXd Coeffs_i = matrixSpline->coeffs[index];
					Eigen::MatrixXd Polys_t(1, 4);
					Polys_t << 1, ti, ti*ti, ti*ti*ti;
					M3d->row(i) = Polys_t * Coeffs_i;
					M2d->row(i) = Polys_t * Coeffs_i;
				}

#endif
				assert(restPos->rows() > 0);
				*centerPos = Eigen::MatrixXd::Zero(1, 3);
				for (int i = 0; i < restPos->rows(); i++)
				{
					(*centerPos).block(0, 0, 1, 3) += (*restPos).block(i, 0, 1, 3);
				}
				*centerPos *= 1.0 / restPos->rows();
				*centerRestPos = *centerPos;

				break;
			}

		}

		(*rotCenter) = (*rotRestCenter) = (*centerRestPos);
		*BE = - Eigen::MatrixXi::Ones(1, 2);
	}
};

class HandleFrame
{
public:
	std::string name;
	PointMatrixType Handles;
	HandleFrame(std::string n, PointMatrixType H):name(n),Handles(H){}
};


#ifdef IGL_HEADER_ONLY
#	include "handle.cpp"
#endif


#endif /*IGL_ADDIN_HANDLE_H*/