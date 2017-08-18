// Copyright 2013 - Christian Schüller 2013, schuellc@inf.ethz.ch
// Interactive Geometry Lab - ETH Zurich

#include "MassSpring_LIMSolver3D.h"
#include "TetrahedronMesh.h"

MassSpring_LIMSolver3D::MassSpring_LIMSolver3D():
has_inited(false)
{
}

MassSpring_LIMSolver3D::~MassSpring_LIMSolver3D()
{
}

void MassSpring_LIMSolver3D::debugOutput(std::stringstream& info)
{
  std::cout << "I: " << info.str() << "\n"; 
}

void MassSpring_LIMSolver3D::prepareProblemData(std::vector<int>& hessRowIdx, std::vector<int>& hessColIdx)
{
  const int numNodes = mesh->InitalVertices->rows();
  const int numTets = mesh->Tetrahedra->rows();

  // create sparse identity matrix
  Identity.resize(numVariables,numVariables);
  std::vector<Eigen::Triplet<double> > triplets;
  triplets.reserve(numVariables);
  for(int i=0;i<numVariables;i++)
  {
    triplets.push_back(Eigen::Triplet<double>(i,i,1));	
  }
  Identity.setFromTriplets(triplets.begin(),triplets.end());

  for (int i=0;i<numVariables;i++)
  {
    hessRowIdx.push_back(i);
    hessColIdx.push_back(i);
  }


}

double MassSpring_LIMSolver3D::computeFunction(const Eigen::Matrix<double,Eigen::Dynamic,1>& x)
{
	double E=0;
	const int numNodes = mesh->InitalVertices->rows();
	for(int i=0; i<spring_list.size(); i++)
	{
		int start = spring_list[i].start_index;
		int end = spring_list[i].end_index;
		double dx = x(start+0*numNodes) - x(end+0*numNodes);
		double dy = x(start+1*numNodes) - x(end+1*numNodes);
		double dz = x(start+2*numNodes) - x(end+2*numNodes);
		double len = sqrt(dx*dx+dy*dy+dz*dz);
		E+=0.5*spring_list[i].k*(len-spring_list[i].rest_len)*(len-spring_list[i].rest_len);
	}

  return E;
}

void MassSpring_LIMSolver3D::computeGradient(const Eigen::Matrix<double,Eigen::Dynamic,1>& x, Eigen::Matrix<double,Eigen::Dynamic,1>& grad)
{
	grad.setZero();
	const int numNodes = mesh->InitalVertices->rows();
	for(int i=0; i<spring_list.size(); i++)
	{
		int start = spring_list[i].start_index;
		int end = spring_list[i].end_index;
		double dx = x[start+0*numNodes] - x[end+0*numNodes];
		double dy = x[start+1*numNodes] - x[end+1*numNodes];
		double dz = x[start+2*numNodes] - x[end+2*numNodes];
		double len = sqrt(dx*dx+dy*dy+dz*dz);
		grad(start+0*numNodes) += spring_list[i].k*(len-spring_list[i].rest_len)*dx/len;
		grad(start+1*numNodes) += spring_list[i].k*(len-spring_list[i].rest_len)*dy/len;
		grad(start+2*numNodes) += spring_list[i].k*(len-spring_list[i].rest_len)*dz/len;
		grad(end+0*numNodes) -= spring_list[i].k*(len-spring_list[i].rest_len)*dx/len;
		grad(end+1*numNodes) -= spring_list[i].k*(len-spring_list[i].rest_len)*dy/len;
		grad(end+2*numNodes) -= spring_list[i].k*(len-spring_list[i].rest_len)*dz/len;
	}

}

void MassSpring_LIMSolver3D::computeHessian(const Eigen::Matrix<double,Eigen::Dynamic,1>& x, const Eigen::Matrix<double*,Eigen::Dynamic,1>& hess)
{
  // identity
  for(int i=0;i<numVariables;i++)
    *hess[i] = 1;
}