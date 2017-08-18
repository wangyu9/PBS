#ifndef IGL_ADDIN_NORMAL_DEFORM_FACTORS_H
#define IGL_ADDIN_NORMAL_DEFORM_FACTORS_H
#include <igl/igl_inline.h>
#include <Eigen/Core>


#include <cotangent.h>
#include <igl/doublearea.h>
#include <igl/per_vertex_attribute_smoothing.h>

  //// Compute vertex normals via vertex position list, face list
  //// Inputs:
  ////   V  #V by 3 eigen Matrix of mesh vertex 3D positions
  ////   F  #F by 3 eigne Matrix of face (triangle) indices
  ////   weighting  Weighting type
  //// Output:
  ////   N  #V by 3 eigen Matrix of mesh vertex 3D normals
  //template <typename DerivedV, typename DerivedF>
  //IGL_INLINE void per_vertex_normals(
  //  const Eigen::PlainObjectBase<DerivedV>& V,
  //  const Eigen::PlainObjectBase<DerivedF>& F,
  //  const igl::PerVertexNormalsWeightingType weighting,
  //  Eigen::PlainObjectBase<DerivedV> & N);
  //// Without weighting
  //template <typename DerivedV, typename DerivedF>
  //IGL_INLINE void per_vertex_normals(
  //  const Eigen::PlainObjectBase<DerivedV>& V,
  //  const Eigen::PlainObjectBase<DerivedF>& F,
  //  Eigen::PlainObjectBase<DerivedV> & N);
  //// Inputs:
  ////   FN  #F by 3 matrix of face (triangle) normals
  //template <typename DerivedV, typename DerivedF>
  //IGL_INLINE void per_vertex_normals(
  //  const Eigen::PlainObjectBase<DerivedV>& V,
  //  const Eigen::PlainObjectBase<DerivedF>& F,
  //  const PerVertexNormalsWeightingType weighting,
  //  const Eigen::PlainObjectBase<DerivedV>& FN,
  //  Eigen::PlainObjectBase<DerivedV> & N);
  //// Without weighting
  //template <typename DerivedV, typename DerivedF>
  //IGL_INLINE void per_vertex_normals(
  //  const Eigen::PlainObjectBase<DerivedV>& V,
  //  const Eigen::PlainObjectBase<DerivedF>& F,
  //  const Eigen::PlainObjectBase<DerivedV>& FN,
  //  Eigen::PlainObjectBase<DerivedV> & N);


template <typename DerivedV, typename DerivedT>
IGL_INLINE void per_vertex_tangent_plane(
	const Eigen::PlainObjectBase<DerivedV>& V,
	const Eigen::PlainObjectBase<DerivedV>& Normals_vertex0,//N,//Normals per vertex at rest pose.
	Eigen::PlainObjectBase<DerivedT>& t_dir,//Tangents,
	Eigen::PlainObjectBase<DerivedT>& b_dir//Binormals
	)
{
	int n = V.rows();

	t_dir.resize(V.rows(),3);
	b_dir.resize(V.rows(),3);

	for (int v=0; v<V.rows(); v++)
	{
		Eigen::Vector3d normal_dir;
		normal_dir(0) = Normals_vertex0(v,0);
		normal_dir(1) = Normals_vertex0(v,1);
		normal_dir(2) = Normals_vertex0(v,2);
		//Eigen::Vector3d t0 = Eigen::Vector3d(-1,-1,-1);
		//Eigen::Vector3d b0 = Eigen::Vector3d(-1,0,1);
		//t_dir.block(0,v,3,1) = normal_dir.cross(t0.cross(normal_dir)).block(0,0,3,1);
		//b_dir.block(0,v,3,1) = normal_dir.cross(b0.cross(normal_dir)).block(0,0,3,1);

		/*************start of orthogonal vectors calculate*****************/

		double a = normal_dir(0);
		double b = normal_dir(1);
		double c = normal_dir(2);
		Eigen::Vector3d t0;
		Eigen::Vector3d b0;
		if (abs(a)>=abs(b))
		{
			if (abs(c)>=abs(a))
			{
				// |c| is the max
				t0 = Eigen::Vector3d(1,0,0);
				b0 = Eigen::Vector3d(0,-1,0);
			} 
			else
			{
				// |a| is the max
				t0 = Eigen::Vector3d(0,-1,0);
				b0 = Eigen::Vector3d(0,0,-1);
			}
		}
		else
		{
			if (abs(c)>=abs(b))
			{
				// |c| is the max
				t0 = Eigen::Vector3d(1,0,0);
				b0 = Eigen::Vector3d(0,-1,0);
			} 
			else
			{
				// |b| is the max
				t0 = Eigen::Vector3d(1,0,0);
				b0 = Eigen::Vector3d(0,0,-1);
			}
		}
		t0 = Eigen::Vector3d(1,-1,0);
		b0 = Eigen::Vector3d(5,6,0);

		Eigen::Vector3d orthogonal_v1 = normal_dir.cross(t0.cross(normal_dir)).normalized();
		//Eigen::Vector3d orthogonal_v2 = normal_dir.cross(b0.cross(normal_dir)).normalized();
		Eigen::Vector3d orthogonal_v2 = - normal_dir.cross(orthogonal_v1).normalized();
		if ( normal_dir.dot(orthogonal_v1.cross(orthogonal_v2))>0 )
		{
			;// nothing
		} 
		else
		{
			orthogonal_v2 = -orthogonal_v2;
		}

		double prod = normal_dir.dot((orthogonal_v1.cross(orthogonal_v2)).normalized());
		if (prod<0.9)
		{
			printf("Warning: initializing tangent vectors incorrectly for vertex %d! with inner product: %f.\n", v, prod);
		}

		/**************************************************/

		t_dir.block(v,0,1,3) = orthogonal_v1.transpose();
		b_dir.block(v,0,1,3) = orthogonal_v2.transpose();
	}
}

template <typename DerivedV, typename DerivedF, typename DerivedW>
IGL_INLINE void per_face_attributes_gradient(
	const Eigen::PlainObjectBase<DerivedV>& V,
	const Eigen::PlainObjectBase<DerivedF>& F,
	const Eigen::PlainObjectBase<DerivedV>& Normals_face0,//NF,//Normals per face at rest pose.
	const Eigen::PlainObjectBase<DerivedW>& Weights,//W,
	Eigen::PlainObjectBase<DerivedW>& a_hat_faces
	)
{
	int m = Weights.cols();

	a_hat_faces.resize(F.rows()*4,m);

	for(int tri=0; tri<F.rows(); tri++)
	{
		Eigen::Matrix4d P;
		P.setZero();
		for(int i=0; i<3; i++)
		{
			//P.row(i) = V.row(F(tri,i));
			if (F(tri,i)>=V.rows())
			{
				printf("Error F(%d,%d)=%d exceeds V dimension %d\n",tri,i,F(tri,i),V.rows());
			}
			P.block(i,0,1,3) = V.block(F(tri,i),0,1,3);
			//P(i,0) = V(F(tri,i),0);
			//P(i,1) = V(F(tri,i),1);
			//P(i,2) = V(F(tri,i),2);
			P(i,3) = 1;	
		}
		//P.block(3,0,1,3) = Normals_face0.block(tri,0,1,3);
		P(3,0)=Normals_face0(tri,0);
		P(3,1)=Normals_face0(tri,1);
		P(3,2)=Normals_face0(tri,2);
		P(3,3) = 0;//wangyu This should be zero!!
		Eigen::MatrixXd B;
		B.resize(4,m);
		for(int j=0; j<m; j++)
		{
			B(0,j) = Weights(F(tri,0),j);
			B(1,j) = Weights(F(tri,1),j);
			B(2,j) = Weights(F(tri,2),j);
			B(3,j) = 0;
		}
		Eigen::Matrix4d inv_P = P.inverse();
		Eigen::MatrixXd X = inv_P*B;
		a_hat_faces.block(4*tri,0,4,m) = X.block(0,0,4,m);
	}
}

template <typename DerivedV, typename DerivedF, typename DerivedW>
IGL_INLINE void per_vertex_attributes_gradient(
	const Eigen::PlainObjectBase<DerivedV>& V,
	const Eigen::PlainObjectBase<DerivedF>& F,
	Eigen::PlainObjectBase<DerivedW>& a_hat_faces,
	Eigen::PlainObjectBase<DerivedW>& a_hat_vertices
	)
{
	int m = a_hat_faces.cols();

	a_hat_vertices.resize(V.rows()*4,m);

	Eigen::VectorXd num_adjacent(V.rows());
	a_hat_vertices.setZero();
	num_adjacent.setZero();

	// Cotangent Weighting
	Matrix<double,Dynamic,Dynamic> C;
	igl::cotangent(V,F,C);
	// Area Weighting
	Eigen::VectorXd A;
	igl::doublearea(V,F,A);
	A = A/2.0;

	for (int fIndex=0; fIndex<F.rows(); fIndex++)
	{
		// Uniform Weighting
		//a_hat_vertices.block(4*F(fIndex,0),0,4,m) += a_hat_faces.block(4*fIndex,0,4,m);
		//a_hat_vertices.block(4*F(fIndex,1),0,4,m) += a_hat_faces.block(4*fIndex,0,4,m);
		//a_hat_vertices.block(4*F(fIndex,2),0,4,m) += a_hat_faces.block(4*fIndex,0,4,m);
		//num_adjacent(F(fIndex,0)) += 1.0;
		//num_adjacent(F(fIndex,1)) += 1.0;
		//num_adjacent(F(fIndex,2)) += 1.0;

		// Cotangent Weighting
		//a_hat_vertices.block(4*F(fIndex,0),0,4,m) += a_hat_faces.block(4*fIndex,0,4,m)*C(fIndex,0);
		//a_hat_vertices.block(4*F(fIndex,1),0,4,m) += a_hat_faces.block(4*fIndex,0,4,m)*C(fIndex,1);
		//a_hat_vertices.block(4*F(fIndex,2),0,4,m) += a_hat_faces.block(4*fIndex,0,4,m)*C(fIndex,2);
		//num_adjacent(F(fIndex,0)) += C(fIndex,0);
		//num_adjacent(F(fIndex,1)) += C(fIndex,1);
		//num_adjacent(F(fIndex,2)) += C(fIndex,2);

		// Area Weighting
		a_hat_vertices.block(4*F(fIndex,0),0,4,m) += a_hat_faces.block(4*fIndex,0,4,m)*A(fIndex);
		a_hat_vertices.block(4*F(fIndex,1),0,4,m) += a_hat_faces.block(4*fIndex,0,4,m)*A(fIndex);
		a_hat_vertices.block(4*F(fIndex,2),0,4,m) += a_hat_faces.block(4*fIndex,0,4,m)*A(fIndex);
		num_adjacent(F(fIndex,0)) += A(fIndex);
		num_adjacent(F(fIndex,1)) += A(fIndex);
		num_adjacent(F(fIndex,2)) += A(fIndex);
	}
	for (int vIndex=0; vIndex<V.rows(); vIndex++)
	{
		if (num_adjacent(vIndex)>0)
		{
			// All Weighting
			a_hat_vertices.block(4*vIndex,0,4,m) *= 1.0/num_adjacent(vIndex);
		}
	}
}

template <typename DerivedV, typename DerivedF, typename DerivedW, typename DerivedT, typename DerivedA>
IGL_INLINE void normal_deform_factors(
	const Eigen::PlainObjectBase<DerivedV>& V,
	const Eigen::PlainObjectBase<DerivedF>& F,
	const Eigen::PlainObjectBase<DerivedV>& Normals_vertex0,//N,//Normals per vertex at rest pose.
	const Eigen::PlainObjectBase<DerivedV>& Normals_face0,//NF,//Normals per face at rest pose.
	const Eigen::PlainObjectBase<DerivedW>& Weights,//W,
	Eigen::PlainObjectBase<DerivedT>& t_dir,//Tangents,
	Eigen::PlainObjectBase<DerivedT>& b_dir,//Binormals
	Eigen::PlainObjectBase<DerivedA>& alphaFactors,
	Eigen::PlainObjectBase<DerivedA>& betaFactors
	)
{
	assert(Weights.rows()==V.rows());
	//assert(Weights.cols()==Handles().rows());//do no need

	//int n = V.rows();
	//int m = Weights.cols();

	// Step 1: set up Weights Gradient 
	// a_hat is the piece-wise linear weights gradient per triangle
	// the column of a_hat is the gradient of triangle i 
	
	// using MATLAB expression, here a_hat_faces(4i:(4i+3),j) store the j'th weight's gradient on triangle i 

	per_vertex_tangent_plane(V,Normals_vertex0,t_dir,b_dir);

	Eigen::MatrixXd a_hat_faces;
	per_face_attributes_gradient(V,F,Normals_face0,Weights,a_hat_faces);

	Eigen::MatrixXd a_hat_vertices;
	per_vertex_attributes_gradient(V,F,a_hat_faces,a_hat_vertices);

	// nullify the weight gradients at all vertices, see the EG lighting section 4.3 for explanation
	// which taking advantage of the fact that all weights' gradients sum to 0 at any vertex
	//for (int j=m-1; j>=0; j--)
	//{
	//	// reverse order is important
	//	a_hat_vertices.col(j) -= a_hat_vertices.col(0);
	//}

	// Directional Derivatives

	alphaFactors.resize(Weights.rows(),Weights.cols());
	betaFactors.resize(Weights.rows(),Weights.cols());
	alphaFactors.setZero();
	betaFactors.setZero();
	for (int i=0; i<Weights.rows(); i++)
	{
		for (int j=0; j<Weights.cols() ;j++)
		{
			for (int c=0; c<3; c++)
			{
				alphaFactors(i,j) += a_hat_vertices(4*i+c,j)*t_dir(i,c);
				betaFactors(i,j)  += a_hat_vertices(4*i+c,j)*b_dir(i,c);
			}
		}
	}

}

#ifndef IGL_HEADER_ONLY
#  include "normal_deform_factors.cpp"
#endif

#endif /*IGL_ADDIN_NORMAL_DEFORM_FACTORS_H*/
