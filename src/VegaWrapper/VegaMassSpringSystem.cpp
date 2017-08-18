#include "VegaMassSpringSystem.h"
#include <assert.h>

VegaMassSpringSystem::VegaMassSpringSystem(int numParticles, double * masses,
	double * restPositions, int numEdges, int * edges,
	int * edgeGroups, int numMaterialGroups,
	double * groupStiffness, double * groupDamping,
	int addGravity):
	dim(3),
	numVertices(numParticles),
	vertices(restPositions),
	timeStep(0.01)
{
	rest_vertices = (double*) malloc (sizeof(double) * numParticles * dim);
	memcpy(rest_vertices, restPositions, sizeof(double) * numParticles * dim);

	disp_vertices = (double*) malloc (sizeof(double) * numParticles * dim);
	memset(disp_vertices,0,sizeof(double) * numParticles * dim);

	f = (double*) malloc(sizeof(double) * dim * numVertices);
	u = (double*) malloc(sizeof(double) * dim * numVertices);
	uvel = (double*) malloc(sizeof(double) * dim * numVertices);//velocity of all vetices

	//double * init_vel = (double*) malloc (sizeof(double) * numParticles * dim);
	//memset(init_vel, 0, sizeof(double) * numParticles * dim);

	pMassSpringSystem = new MassSpringSystem( numParticles,  masses,
		restPositions, numEdges, edges,
		edgeGroups, numMaterialGroups,
		groupStiffness, groupDamping,
		addGravity);
	
	//pMassSpringSystem->GenerateMassMatrix(&pMassMatrix, 1);
	//delete(pMassMatrix);
	pMassSpringSystem->GenerateMassMatrix(&pMassMatrix);

	pMassSpringSystemForceModel = new MassSpringSystemForceModel(pMassSpringSystem);

	pIntegratorSparse = new ImplicitBackwardEulerSparse(numParticles*dim,timeStep,pMassMatrix,pMassSpringSystemForceModel);
	pIntegratorSparse->ResetToRest();
	pIntegratorSparse->SetState(disp_vertices);//Initilize the value in the integrator


	//integratorBase->ResetToRest();
	//integratorBase->SetState(uInitial, velInitial);
	//integratorBase->SetTimestep(timeStep / substepsPerTimeStep);

	//numVertices = pMassSpringSystem->GetNumParticles();
	//vertices = (double*) malloc (sizeof(double) * dim * numVertices);
	//memset(vertices, 0, sizeof(double) * dim * numVertices);
}

VegaMassSpringSystem::VegaMassSpringSystem(int numParticles, double mass,
	double * restPositions, int numEdges, int * edges,
	double stiffness, double damping, int numFixedVertices, int *fixedVertices,
	int addGravity):
dim(3),
numVertices(numParticles),
vertices(restPositions),
timeStep(0.01)
{//set uniform material property
	

	printf("check mesh %d %d",numParticles,numEdges);

	rest_vertices = (double*) malloc (sizeof(double) * numParticles * dim);
	memcpy(rest_vertices, restPositions, sizeof(double) * numParticles * dim);

	disp_vertices = (double*) malloc (sizeof(double) * numParticles * dim);
	memset(disp_vertices,0,sizeof(double) * numParticles * dim);

	f = (double*) malloc(sizeof(double) * dim * numVertices);
	u = (double*) malloc(sizeof(double) * dim * numVertices);
	uvel = (double*) malloc(sizeof(double) * dim * numVertices);//velocity of all vetices

	double * init_vel = (double*) malloc (sizeof(double) * numParticles * dim);
	memset(init_vel, 0, sizeof(double) * numParticles * dim);
	//for(int i=0; i<numParticles; i++)
	//{
	//	if( rest_vertices[3*i+2]>0 )
	//		init_vel[3*i+2] = 0.2;
	//	else
	//		init_vel[3*i+2] = -0.2;
	//}


	double *masses = (double*) malloc (sizeof(double) * numParticles);
	for(int i=0; i<numParticles; i++)
	{
		masses[i] = mass;// we cannot use memset to set initial value
	}
	//double *masses = new double[numVertices];

	int *edgeGroups = (int*) malloc (sizeof(int) * numEdges);
	for(int i=0; i<numEdges; i++)
	{
		edgeGroups[i] = 0;//to assign 0, this is equivalent to memset(edgeGroups, 0, sizeof(int) * numEdges);
	}
	//int *edgeGroups = new int[numEdges];

	int numMaterialGroups = 1;
	//
	double *groupStiffness = (double*) malloc( sizeof(double) * numMaterialGroups );
	for(int i=0; i<numMaterialGroups; i++)
	{
		groupStiffness[i] = stiffness;
	}

	double *groupDamping = (double*) malloc(sizeof(double) * numMaterialGroups);
	for(int i=0; i<numMaterialGroups; i++)
	{
		groupDamping[i] = damping;
	}

	// create 0-indexed fixed DOFs
	int numFixedDOFs = 3 * numFixedVertices;
	int * fixedDOFs = (int*) malloc (sizeof(int) * numFixedDOFs);
	if(numFixedDOFs==0)
	{
		fixedDOFs = NULL;
	}
	for(int i=0; i<numFixedVertices; i++)
	{
		fixedDOFs[3*i+0] = 3*fixedVertices[i];
		fixedDOFs[3*i+1] = 3*fixedVertices[i]+1;
		fixedDOFs[3*i+2] = 3*fixedVertices[i]+2;
	}
	//for(int i=0; i<numFixedVertices; i++)
	//	fixedVertices[i]--;





	pMassSpringSystem = new MassSpringSystem( numParticles,  masses,
		restPositions, numEdges, edges,
		edgeGroups, numMaterialGroups,
		groupStiffness, groupDamping,
		addGravity);

	pMassSpringSystem->SetGravity(true,-9.8);

	pMassSpringSystem->GenerateMassMatrix(&pMassMatrix);

	pMassSpringSystemForceModel = new MassSpringSystemForceModel(pMassSpringSystem);

	pIntegratorSparse = new ImplicitBackwardEulerSparse(numParticles*dim,timeStep,pMassMatrix,pMassSpringSystemForceModel,0,numFixedDOFs, fixedDOFs);
	pIntegratorSparse->SetState(disp_vertices,init_vel);//Initilize the value in the integrator
	pIntegratorSparse->SetTimestep(1/40.0);

	//numVertices = pMassSpringSystem->GetNumParticles();
	//vertices = (double*) malloc (sizeof(double) * dim * numVertices);
	//memset(vertices, 0, sizeof(double) * dim * numVertices);

	//free(masses);
	//free(edgeGroups);
	//free(groupStiffness);
	//free(groupDamping);

	//delete [] masses;
	//delete [] edgeGroups;
}

VegaMassSpringSystem::VegaMassSpringSystem():
dim(3)
{
	int numParticles = 20;        
	double groupStiffness = 5;
	printf("Creating a chain mass-spring system with %d particles...\n", numParticles);

	double * masses = (double*) malloc (sizeof(double) * numParticles);
	for(int i=0; i<numParticles; i++)
		masses[i] = 1.0;

	double * restPositions = (double*) malloc (sizeof(double) * 3 * numParticles);
	for(int i=0; i<numParticles; i++)
	{
		restPositions[3*i+0] = 0;
		restPositions[3*i+1] = (numParticles == 1) ? 0.0 : 1.0 * i / (numParticles-1);
		restPositions[3*i+2] = 0;
	}
	int * edges = (int*) malloc (sizeof(int) * 2 * (numParticles - 1));
	for(int i=0; i<numParticles-1; i++)
	{
		edges[2*i+0] = i;
		edges[2*i+1] = i+1;
	}

	f = (double*) malloc(sizeof(double) * dim * numVertices);
	u = (double*) malloc(sizeof(double) * dim * numVertices);
	uvel = (double*) malloc(sizeof(double) * dim * numVertices);//velocity of all vetices

	int * edgeGroups = (int*) malloc (sizeof(int) * (numParticles - 1));
	for(int i=0; i<numParticles-1; i++)
		edgeGroups[i] = 0;
	double groupDamping = 0;

	pMassSpringSystem = new MassSpringSystem(numParticles, masses, restPositions, numParticles - 1, edges, edgeGroups, 1, &groupStiffness, &groupDamping, false);

	char s[96];
	sprintf(s,"chain-%d.obj", numParticles);
	pMassSpringSystem->CreateObjMesh(s);
	//strcpy(renderingMeshFilename, s);

	free(edgeGroups);
	free(edges);
	free(restPositions);
	free(masses);

	//renderVertices = 1;


	int n = pMassSpringSystem->GetNumParticles();

	// create the mass matrix
	//massSpringSystem->GenerateMassMatrix(&massMatrix, 1);
	//delete(massMatrix);
	pMassSpringSystem->GenerateMassMatrix(&pMassMatrix);
}

VegaMassSpringSystem::~VegaMassSpringSystem()
{
	delete pIntegratorSparse;
	delete pMassSpringSystemForceModel;
	delete pMassSpringSystem;
	
	free(vertices);
	free(rest_vertices);
	free(disp_vertices);
	free(f);
	free(u);
	free(uvel);
}

void VegaMassSpringSystem::doTimeStep(double time_gap)
{
	assert(time_gap>=0.0000001);//threshold of time_gap

	
	//double * f = (double*) malloc(sizeof(double) * dim * numVertices);
	memset(f, 0, sizeof(double) * dim * numVertices);

	//double * u = (double*) malloc(sizeof(double) * dim * numVertices);
	memset(u, 0, sizeof(double) * dim * numVertices);

	for(int i=0; i<dim*numVertices; i++)
	{
		u[i] = vertices[i] - rest_vertices[i];
	}

	//pMassSpringSystem->ComputeForce( u, f, true);

	//double * uvel = (double*) malloc(sizeof(double) * dim * numVertices);//velocity of all vetices
	memset(uvel, 0, sizeof(double) * dim * numVertices);

	//pMassSpringSystem->AddDampingForce(uvel,f,0,pMassSpringSystem->GetNumEdges());

	//pMassSpringSystemForceModel->Reset(u);

	for(int i=0; i<posConstrainedParticlesIndex.size(); i++)
	{
		int cpIndex = posConstrainedParticlesIndex[i];
		//vertices[cpIndex*dim+0]
		//f[cpIndex*dim+0] = 0;
		//f[cpIndex*dim+1] = 0;
		//f[cpIndex*dim+2] = 0;
	}
	//pIntegratorSparse->for
	if(pIntegratorSparse->DoTimestep())//return 0 if successsful
		printf("Fail to do time integration!\n");

	//cppying the value from integrator to vertices:
	double *q = pIntegratorSparse->Getq();
	for(int i=0; i<numVertices*dim; i++)
	{
		vertices[i] =q[i]+rest_vertices[i];
	}

	//free(f);
	//free(u);
	//free(uvel);
}

void VegaMassSpringSystem::setState(int numConstraintDOF, int* indexConstraintDOF, double* valueConstrantDOF)
{
	double *copy_states = new double[dim*numVertices];
	double *q = pIntegratorSparse->Getq();
	memcpy(copy_states, q, dim*numVertices*sizeof(double));
	for(int i=0; i<numConstraintDOF; i++)
	{
		int index = indexConstraintDOF[i];
		copy_states[index] = valueConstrantDOF[i]-rest_vertices[index];
	}
	pIntegratorSparse->SetState(copy_states);
	delete [] copy_states;
}

//Eigen Interface 
//VegaMassSpringSystem::VegaMassSpringSystem(Eigen::MatrixXd* verticesMatrix, Eigen::MatrixXd* facesMatrix)
//{
//	//VegaMassSpringSystem(int numParticles, double * masses,
//	//	double * restPositions, int numEdges, int * edges,
//	//	int * edgeGroups, int numMaterialGroups,
//	//	double * groupStiffness, double * groupDamping,
//	//	 0);
//}

//void VegaMassSpringSystem::copyVerticesMatrix(Eigen::MatrixXd* verticesMatrix)
//{
//
//}