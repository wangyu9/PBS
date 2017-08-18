#include "massSpringSystem.h"
#include "integratorBaseSparse.h"
#include "massSpringSystemForceModel.h"
#include "implicitBackwardEulerSparse.h"
#include <vector>


//#include <Eigen/Dense>
//#include <Eigen/Geometry>

class VegaMassSpringSystem{
public:
	double * vertices;//u[3*index+0] if we want  p(index).x
	double * rest_vertices;
	double * disp_vertices;
	int numVertices;
	int dim;
	double * f;
	double * u;
	double * uvel;//velocity of all vetices
	
	std::vector<int> posConstrainedParticlesIndex;
	SparseMatrix *pMassMatrix;
	MassSpringSystem *pMassSpringSystem;
	MassSpringSystemForceModel *pMassSpringSystemForceModel;
	double timeStep;
	ImplicitBackwardEulerSparse *pIntegratorSparse;
	
	VegaMassSpringSystem(int numParticles, double * masses,
		double * restPositions, int numEdges, int * edges,
		int * edgeGroups, int numMaterialGroups,
		double * groupStiffness, double * groupDamping,
		int addGravity);
	VegaMassSpringSystem(int numParticles, double mass,
		double * restPositions, int numEdges, int * edges,
		double stiffness, double damping, int numFixedVertices=0,int *fixedVertices=NULL,
		int addGravity=0);
	VegaMassSpringSystem();
	//(TetMesh * tetMesh,
	//	MassSpringSystem ** massSpringSystem, double density,
	//	double tensileStiffness, double damping, int addGravity=0);
	~VegaMassSpringSystem();

	void doTimeStep(double time_gap);//time_gap is how long it takes since last call of doTimeStep()

	void setState(int numConstraintDOF, int* indexConstraintDOF, double* valueConstrantDOF);

	//Eigen interface:
	//VegaMassSpringSystem(Eigen::Matrix* verticesMatrix, Eigen::Matrix* facesMatrix);
	//void copyVerticesMatrix(Eigen::Matrix* verticesMatrix);
};