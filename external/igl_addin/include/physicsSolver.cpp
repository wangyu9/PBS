#include "physicsSolver.h"

#include <Eigen/Dense>
#include <unsupported/Eigen/SparseExtra>

#ifdef USING_IGL_HEADER_ONLY_MODE
#define IGL_HEADER_ONLY 
#endif

#include <igl/polar_svd3x3.h> // new path //old path in libigl #include <igl/svd3x3/polar_svd3x3.h>

PhysicsSolver::PhysicsSolver()
{

}	

void PhysicsSolver::Init(Eigen::MatrixXi& TT,Eigen::MatrixXd& VV)
{
	T = TT;
	V = VV;
	V0 = VV;
	Vel.resize(V.rows(),V.cols());
	Vel.setZero();
	bool tet_is_one_index = false;
	if (tet_is_one_index)
	{
		T = T - Eigen::MatrixXi::Ones(T.rows(),T.cols());
	}
	D_m = NULL;
	B_m = NULL;
	W = NULL;
}

void PhysicsSolver::setConstraints(Eigen::VectorXi& SS, Eigen::MatrixXd& V_new)
{
	S = SS;
	for (int i=0; i<S.rows(); i++)
	{
		if(S(i)>=0)//constrained vertices
		{
			V.row(i) = V_new.row(i);
		}
	}
}

void PhysicsSolver::precomputation()
{

	if (D_m!=NULL)
	{
		delete [] D_m;
	}
	if (B_m!=NULL)
	{
		delete [] B_m;
	}
	if (W!=NULL)
	{
		delete [] W;
	}
	D_m = new Eigen::MatrixXd[T.rows()];
	B_m = new Eigen::MatrixXd[T.rows()];
	W = new double[T.rows()];
	for (int tet_index=0; tet_index<T.rows(); tet_index++)//for all tetrahedra
	{
		int dim = 3;
		D_m[tet_index].resize(dim,dim);
		for (int d=0; d<dim; d++)
		{
			//not sure correct yet: D_m[tet_index].col(d).array() = V0.row(T(tet_index,d)).array()
			//	-V0.row(T(tet_index,dim)).array();
			for (int c=0; c<dim; c++)
			{
				D_m[tet_index](c,d) = V0(T(tet_index,d),c)
					-V0(T(tet_index,dim),c);
			}
		}
		B_m[tet_index] = D_m[tet_index].inverse();
		W[tet_index] = D_m[tet_index].determinant()/6.0;
	}
}

void PhysicsSolver::computeElasticForces(double mu)
{
	Forces.resize(V.rows(),3);
	Forces.setZero();//Force should be initialized to zero
	for (int tet_index=0; tet_index<T.rows(); tet_index++)//for all tetrahedra
	{
		int dim=3;
		Eigen::MatrixXd Ds;
		Ds.resize(dim,dim);
		for (int d=0; d<dim; d++)
		{
			//not right yet Ds.col(d).array() = V.col(T(tet_index,d)).array()
			//	-V.col(T(tet_index,dim)).array();
			for (int c=0; c<dim; c++)
			{
				Ds(c,d) = V(T(tet_index,d),c)
					-V(T(tet_index,dim),c);
			}
		}
		Eigen::Matrix<double,3,3> F;
		Eigen::Matrix<double,3,3> R;
		Eigen::MatrixXd P;
		//Eigen::MatrixXd R;
		//Eigen::MatrixXd F;//DeformationGradient;
		F = Ds*B_m[tet_index];

		igl::polar_svd3x3(F,R);

		//P is computed here, it depends on the material type.

		//Linear Elasticity, P(F) = mu*(F + F' − 2I) + lamda*tr(F − I)I.
		Eigen::MatrixXd e;
		e = (F+F.transpose())/2 - Eigen::Matrix<double,3,3>::Identity();
		double lamada = mu;
		P = 2*mu*e+lamada*(F-Eigen::Matrix<double,3,3>::Identity()).trace()*Eigen::Matrix<double,3,3>::Identity();

		//Co-rotated Linear Elasticity
		P = 2*mu*(F-R);

		Eigen::MatrixXd H;
		H = -W[tet_index]*P*B_m[tet_index];

		for (int d=0; d<dim; d++)
		{
			for(int c=0; c<dim;c++)
			{
				Forces(T(tet_index,d),c) += H(c,d);
				Forces(T(tet_index,dim),c) -= H(c,d);
			}
		}
	}
}

void PhysicsSolver::computeForceDifferentials(double mu)
{
	Forces.resize(V.rows(),3);
	Forces.setZero();//Force should be initialized to zero
	for (int tet_index=0; tet_index<T.rows(); tet_index++)//for all tetrahedra
	{
		int dim=3;
		Eigen::MatrixXd Ds;
		Ds.resize(dim,dim);
		for (int d=0; d<dim; d++)
		{
			//not right yet Ds.col(d).array() = V.col(T(tet_index,d)).array()
			//	-V.col(T(tet_index,dim)).array();
			for (int c=0; c<dim; c++)
			{
				Ds(c,d) = V(T(tet_index,d),c)
					-V(T(tet_index,dim),c);
			}
		}
		Eigen::Matrix<double,3,3> F;
		Eigen::Matrix<double,3,3> R;
		Eigen::MatrixXd P;
		//Eigen::MatrixXd R;
		//Eigen::MatrixXd F;//DeformationGradient;
		F = Ds*B_m[tet_index];

		igl::polar_svd3x3(F,R);

		//P is computed here, it depends on the material type.
		P = 2*mu*(F-R);

		Eigen::MatrixXd H;
		H = -W[tet_index]*P*B_m[tet_index];

		for (int d=0; d<dim; d++)
		{
			for(int c=0; c<dim;c++)
			{
				Forces(T(tet_index,d),c) += H(c,d);
				Forces(T(tet_index,dim),c) -= H(c,d);
			}
		}
	}
}

void PhysicsSolver::doTimeStep(double timeStep)
{
	integrateExplicitEuler(timeStep);
}

void PhysicsSolver::integrateExplicitEuler(double timeStep)
{
	computeElasticForces(1);
	for (int i=0; i<S.rows(); i++)
	{
		if(S(i)<0)//unconstrained vertices
		{
			Vel.row(i) += Forces.row(i)*timeStep - Vel.row(i)*0.5;
		}
		else
		{
			Vel.row(i).setZero();
		}
		V.row(i) += Vel.row(i);
	}
}