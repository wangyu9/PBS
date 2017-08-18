#include "reproject_coordinate.h"

//#define IGL_HEADER_ONLY

#include "igl/min_quad_with_fixed.h"

#include <vector>

void reproject_coordinate(Eigen::MatrixXd & W_value, Eigen::MatrixXi & W_index, Eigen::MatrixXd & pre_W_value, Eigen::MatrixXi & pre_W_index, Eigen::MatrixXd & V, Eigen::MatrixXd & H, int dim)
{
	assert(dim==2||dim==3);
	assert(W_value.rows()==V.rows());
	
	if(H.rows()!=(W_index.maxCoeff()+1))
	{
		printf("Cannot do coordinate reinforcement since the dimension does not match!\n");
		return;
	}

	assert(H.rows()!=(W_index.maxCoeff()+1));

	int size = W_value.size();
	int n = W_value.rows();
	int spm = W_value.cols();

	Eigen::SparseMatrix<double> Q(size,size);///( V.rows()*H.rows(), V.rows()*H.rows());
	Q.setIdentity();

	Eigen::VectorXd B(size,1);
	for (int i=0; i<spm; i++)
	{
		B.block(i*n,0,n,1) = -2.0 * W_value.block(0,i,n,1);
	}

	Eigen::VectorXi known;
	Eigen::VectorXd knownY;

	Eigen::SparseMatrix<double> Aeq((dim+1)*n,size);
	Eigen::VectorXd Beq((dim+1)*n);

	for (int k=0; k<W_index.cols(); k++)
	{
		for (int i=0; i<dim; i++)
		{
			Eigen::SparseMatrix<double> H_diag(n,n);
			std::vector<Eigen::Triplet<double> > triplets;
			for (int diag_index=0; diag_index<n; diag_index++)
			{
				triplets.push_back(Eigen::Triplet<double>(diag_index,diag_index,H(W_index(diag_index,k),i)));
			}
			H_diag.setFromTriplets(triplets.begin(),triplets.end());
			Aeq.block(i*n,k*n,n,n) = H_diag.block(0,0,n,n);
		}
	}
	for (int i=0; i<dim; i++)
	{
		// coordiante property for every dimension
		Beq.block(i*n,0,n,1) = V.col(i);
	}
	Beq.block(dim*n,0,n,1) = Eigen::VectorXd::Ones(n);

	igl::min_quad_with_fixed_data<double> quad_data;

	Eigen::MatrixXd result;

	igl::min_quad_with_fixed_precompute(Q,known,Aeq,true,quad_data);
	igl::min_quad_with_fixed_solve(quad_data,B,knownY,Beq,result);

	printf("Dimension of Result (%d,%d)\n",result.rows(), result.cols());

	for (int k=0; k<W_index.cols(); k++)
	{
		W_value.col(k) = result.block(k*W_index.rows(),0,W_index.rows(),1);
	}
}

void reproject_coordinate_individually(Eigen::MatrixXd & W_value, const Eigen::MatrixXi & W_index, const Eigen::MatrixXd & V, const Eigen::MatrixXd & H, const int dim, const double row_sum)
{
	assert(dim==2||dim==3);
	assert(W_value.rows()==V.rows());

	if(H.rows()!=(W_index.maxCoeff()+1))
	{
		printf("Cannot do coordinate reinforcement since the dimension does not match!\n");
		return;
	}

	assert(H.rows()!=(W_index.maxCoeff()+1));

	int size = W_value.size();
	int n = W_value.rows();
	int spm = W_value.cols();

	for (int index_vertex=0; index_vertex<W_value.rows(); index_vertex++)
	{

		Eigen::SparseMatrix<double> Q(spm,spm);
		Q.setIdentity();

		Eigen::VectorXd B(spm,1);
		B = -2.0 * (W_value.row(index_vertex)).transpose();

		Eigen::VectorXi known;
		Eigen::VectorXd knownY;

		Eigen::SparseMatrix<double> Aeq(dim+1,spm);
		//Eigen::MatrixXd 
		Eigen::VectorXd Beq(dim+1);

		for (int i=0; i<dim+1; i++)
		{
			for (int j=0; j<spm; j++)
			{
				if (i<dim)
				{
					int index = W_index(index_vertex,j);
					Aeq.coeffRef(i,j) = H(index,i);
					Beq.coeffRef(i) = V(index_vertex,i);
				} 
				else
				{
					Aeq.coeffRef(i,j) = 1;
					Beq.coeffRef(i) = row_sum;
				}
				
			}
		}

		igl::min_quad_with_fixed_data<double> quad_data;

		Eigen::MatrixXd result;

		igl::min_quad_with_fixed_precompute(Q,known,Aeq,true,quad_data);
		igl::min_quad_with_fixed_solve(quad_data,B,knownY,Beq,result);

		W_value.row(index_vertex) = result.transpose();

	}




	//printf("Dimension of Result (%d,%d)\n",result.rows(), result.cols());

	//for (int k=0; k<W_index.cols(); k++)
	//{
	//	W_value.col(k) = result.block(k*W_index.rows(),0,W_index.rows(),1);
	//}
}


