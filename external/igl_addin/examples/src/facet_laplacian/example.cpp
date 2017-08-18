
#include <Eigen/Core>
#include <Eigen/Sparse>



#define IGL_HEADER_ONLY

#include <igl/readMESH.h>
#include <igl/writeDMAT.h>

#include <facet_laplacian.h>

#include <igl/cotmatrix.h>

#define Mesh_File_Path "C:\\WorkSpace\\Visual Studio 2010\\PBS_project\\data2\\arma\\mesh.mesh"

void main()
{
	Eigen::MatrixXd V;
	Eigen::MatrixXd T;
	Eigen::MatrixXd F;
	
	igl::readMESH(Mesh_File_Path,V,T,F);

	Eigen::SparseMatrix<double> L;
	Eigen::SparseMatrix<double> M;

	Eigen::MatrixXd Facets;

	//igl::cotmatrix(V,T,L);

	facet_laplacian(V,T,L,M,Facets);

	//igl::writeDMAT("Ltz.dmat",Eigen::MatrixXd(L));

	printf("%f,%f,%f,%f\n",L.coeff(20,30), L.coeff(66,32), L.coeff(22,34), L.coeff(24,34));

	//  MATLAB(21,31)      0.0013
	//  MATLAB(67,33)      0.0007
	//(23,35)      0.0004
	//(25,35)      0.0026
}