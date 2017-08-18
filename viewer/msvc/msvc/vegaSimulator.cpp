/*************************************************************************
 *                                                                       *
 * Vega FEM Simulation Library Version 2.0                               *
 *                                                                       *
 * "Interactive deformable object simulator" driver application,         *
 *  Copyright (C) 2007 CMU, 2009 MIT, 2013 USC                           *
 *                                                                       *
 * All rights reserved.                                                  *
 *                                                                       *
 * Code authors: Jernej Barbic, Fun Shing Sin, Daniel Schroeder          *
 * http://www.jernejbarbic.com/code                                      *
 *                                                                       *
 * Research: Jernej Barbic, Fun Shing Sin, Daniel Schroeder,             *
 *           Doug L. James, Jovan Popovic                                *
 *                                                                       *
 * Funding: National Science Foundation, Link Foundation,                *
 *          Singapore-MIT GAMBIT Game Lab,                               *
 *          Zumberge Research and Innovation Fund at USC                 *
 *                                                                       *
 * This utility is free software; you can redistribute it and/or         *
 * modify it under the terms of the BSD-style license that is            *
 * included with this library in the file LICENSE.txt                    *
 *                                                                       *
 * This utility is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the file     *
 * LICENSE.TXT for more details.                                         *
 *                                                                       *
 *************************************************************************/

/*****************************************************************************

Interactive deformable object simulator (3D nonlinear solid deformable objects).

Supported integrators:
- implicit backward Euler
- implicit Newmark
- central differences
- Euler
- symplectic Euler

Supported meshes:
- tetrahedral
- cubic

Supported materials:
- linear
- rotated linear (corotational linear FEM)
- Saint-Venant Kirchoff
- invertible Saint-Venant Kirchoff
- invertible neo-Hookean 
- invertible Mooney-Rivlin
- mass-spring system

*******************************************************************************/

#include <stdlib.h>
#include <iostream>
#include <vector>
#include <string>
#include <cstdio>
#include <cassert>
#include <float.h>
using namespace std;

#ifdef WIN32
  #include <windows.h>
#endif

#ifdef __APPLE__
  #include "TargetConditionals.h"
#endif

#include "getopts.h"
//#include "initGraphics.h"
//#include "camera.h"

#include "sceneObjectDeformable.h"
#include "performanceCounter.h"
#include "tetMesh.h"
#include "StVKCubeABCD.h"
#include "StVKTetABCD.h"
#include "StVKTetHighMemoryABCD.h"
#include "implicitBackwardEulerSparse.h"
#include "eulerSparse.h"
#include "centralDifferencesSparse.h"
#include "StVKInternalForces.h"
#include "StVKStiffnessMatrix.h"
#include "StVKInternalForcesMT.h"
#include "StVKStiffnessMatrixMT.h"
#include "StVKForceModel.h"
#include "massSpringSystemForceModel.h"
#include "corotationalLinearFEM.h"
#include "corotationalLinearFEMMT.h"
#include "corotationalLinearFEMForceModel.h"
#include "linearFEMForceModel.h"
#include "isotropicHyperelasticFEM.h"
#include "isotropicHyperelasticFEMMT.h"
#include "isotropicHyperelasticFEMForceModel.h"
#include "isotropicMaterial.h"
#include "StVKIsotropicMaterial.h"
#include "neoHookeanIsotropicMaterial.h"
#include "MooneyRivlinIsotropicMaterial.h"
#include "getIntegratorSolver.h"
#include "volumetricMeshLoader.h"
#include "StVKElementABCDLoader.h"
#include "generateMeshGraph.h"
#include "massSpringSystem.h"
#include "massSpringSystemMT.h"
#include "massSpringSystemFromObjMeshConfigFile.h"
#include "massSpringSystemFromTetMeshConfigFile.h"
#include "massSpringSystemFromCubicMeshConfigFile.h"
#include "graph.h"
#include "renderSprings.h"
#include "configFile.h"
//#include "GL/glui.h"
#include "lighting.h"
#include "loadList.h"
#include "matrixIO.h"

// graphics 
char windowTitleBase[4096] = "Real-time sim";
void displayFunction(void);
int windowID;
int windowWidth = 800;
int windowHeight = 600;
double zNear=0.01;
double zFar=10.0;
double cameraRadius;
double focusPositionX, focusPositionY, focusPositionZ;
double cameraLongitude, cameraLattitude;
//SphericalCamera * camera = NULL;
int g_iLeftMouseButton=0, g_iMiddleMouseButton=0, g_iRightMouseButton=0;
int g_vMousePos[2] = {0,0};
int shiftPressed=0;
int altPressed=0;
int ctrlPressed=0;
int renderWireframe=1;
int renderAxes=0;
int renderDeformableObject=1;
int renderSecondaryDeformableObject=1;
int useRealTimeNormals = 0;
int renderGroundPlane = 0;
int renderFixedVertices = 1;
int renderSprings = 0;
int renderVertices = 0;
int lockScene=0;
int pauseSimulation=0;
int singleStepMode=0;
Lighting * lighting = NULL;
SceneObjectDeformable * deformableObjectRenderingMesh = NULL;
SceneObjectDeformable * secondaryDeformableObjectRenderingMesh = NULL;
SceneObject * extraSceneGeometry = NULL;
char groundPlaneString[128];
double groundPlaneHeight;
double groundPlaneLightHeight = 10.0;
double groundPlaneSize = 15.0;
GLuint displayListGround;

// config file
string configFilename;
char renderingMeshFilename[4096];
char secondaryRenderingMeshFilename[4096];
char secondaryRenderingMeshInterpolationFilename[4096];
char volumetricMeshFilename[4096];
char customMassSpringSystem[4096];
char deformableObjectMethod[4096];
char fixedVerticesFilename[4096];
char massMatrixFilename[4096];
char massSpringSystemObjConfigFilename[4096];
char massSpringSystemTetMeshConfigFilename[4096];
char massSpringSystemCubicMeshConfigFilename[4096];
char invertibleMaterialString[4096] = "__none";
char initialPositionFilename[4096];
char initialVelocityFilename[4096];
char forceLoadsFilename[4096];
char outputFilename[4096];
int corotationalLinearFEM_warp = 1;
const int max_corotationalLinearFEM_warp = 2;
char implicitSolverMethod[4096];
char solverMethod[4096];
char extraSceneGeometryFilename[4096];
char lightingConfigFilename[4096];
float dampingMassCoef;
float dampingStiffnessCoef;
float dampingLaplacianCoef = 0.0;
float deformableObjectCompliance = 1.0;
float baseFrequency = 1.0;
int maxIterations;
double epsilon;
char backgroundColorString[4096] = "255 255 255";
int numInternalForceThreads;
int numSolverThreads;

// simulation
int syncTimestepWithGraphics=1;
float timeStep = 1.0 / 30;
float newmarkBeta = 0.25;
float newmarkGamma = 0.5;
int use1DNewmarkParameterFamily = 1;
int substepsPerTimeStep = 1;
double inversionThreshold;
double fps = 0.0;
const int fpsBufferSize = 5;
int fpsHead = 0;
double fpsBuffer[fpsBufferSize];
double cpuLoad = 0;
double forceAssemblyTime = 0.0;
double forceAssemblyLocalTime = 0.0;
const int forceAssemblyBufferSize = 50;
int forceAssemblyHead = 0;
double forceAssemblyBuffer[forceAssemblyBufferSize];
double systemSolveTime = 0.0;
double systemSolveLocalTime = 0.0;
const int systemSolveBufferSize = 50;
int systemSolveHead = 0;
double systemSolveBuffer[systemSolveBufferSize];
int enableTextures = 0;
int staticSolver = 0;
int graphicFrame = 0;
int lockAt30Hz = 0;
int pulledVertex = -1;
int forceNeighborhoodSize = 5;
int dragStartX, dragStartY;
int explosionFlag = 0;
PerformanceCounter titleBarCounter;
PerformanceCounter explosionCounter;
PerformanceCounter cpuLoadCounter;
int timestepCounter = 0;
int subTimestepCounter = 0;
int numFixedVertices;
int * fixedVertices;
int numForceLoads = 0;
double * forceLoads = NULL;
IntegratorBase * integratorBase = NULL;
ImplicitNewmarkSparse * implicitNewmarkSparse = NULL;
IntegratorBaseSparse * integratorBaseSparse = NULL;
ForceModel * forceModel = NULL;
StVKInternalForces * stVKInternalForces = NULL;
StVKStiffnessMatrix * stVKStiffnessMatrix = NULL;
StVKForceModel * stVKForceModel = NULL;
MassSpringSystemForceModel * massSpringSystemForceModel = NULL;
CorotationalLinearFEMForceModel * corotationalLinearFEMForceModel = NULL;
int enableCompressionResistance = 1;
double compressionResistance = 500;
int centralDifferencesTangentialDampingUpdateMode = 1;
int positiveDefinite = 0;
int addGravity=0;
double g=9.81;
VolumetricMesh * volumetricMesh = NULL;
TetMesh * tetMesh = NULL;
Graph * meshGraph = NULL;
enum massSpringSystemSourceType { OBJ, TETMESH, CUBICMESH, CHAIN, NONE } massSpringSystemSource = NONE;
enum deformableObjectType { STVK, COROTLINFEM, LINFEM, MASSSPRING, INVERTIBLEFEM, UNSPECIFIED } deformableObject = UNSPECIFIED;
enum invertibleMaterialType { INV_STVK, INV_NEOHOOKEAN, INV_MOONEYRIVLIN, INV_NONE } invertibleMaterial = INV_NONE;
enum solverType { IMPLICITNEWMARK, IMPLICITBACKWARDEULER, EULER, SYMPLECTICEULER, CENTRALDIFFERENCES, UNKNOWN } solver = UNKNOWN;
MassSpringSystem * massSpringSystem = NULL;
RenderSprings * renderMassSprings = NULL;
SparseMatrix * massMatrix = NULL;
SparseMatrix * LaplacianDampingMatrix = NULL;
int n;
double * u = NULL;
double * uvel = NULL;
double * uaccel = NULL;
double * f_ext = NULL;
double * f_extBase = NULL;
double * uSecondary = NULL;
double * uInitial = NULL;
double * velInitial = NULL;
// interpolation to secondary mesh
int secondaryDeformableObjectRenderingMesh_interpolation_numElementVertices;
int * secondaryDeformableObjectRenderingMesh_interpolation_vertices = NULL;
double * secondaryDeformableObjectRenderingMesh_interpolation_weights = NULL;


//font is, for example, GLUT_BITMAP_9_BY_15
void print_bitmap_string(float x, float y, float z, void * font, char * s)
{
  glRasterPos3f(x,y,z);
  if (s && strlen(s)) 
  {
    while (*s) 
    {
      glutBitmapCharacter(font, *s);
      s++;
    }
  }
}

// program initialization
void initSimulation()
{
  //// init lighting
  //try
  //{
  //  lighting = new Lighting(lightingConfigFilename);
  //}
  //catch(int exceptionCode)
  //{
  //  printf("Error (%d) reading lighting information from %s .\n",exceptionCode, lightingConfigFilename);
  //  exit(1);
  //}

  // init camera
  //delete(camera);
  double virtualToPhysicalPositionFactor = 1.0;
  //initCamera(cameraRadius, cameraLongitude, cameraLattitude,
  //   focusPositionX, focusPositionY, focusPositionZ,
  //   1.0 / virtualToPhysicalPositionFactor,
  //   &zNear, &zFar, &camera);

  volumetricMesh = NULL;
  massSpringSystem = NULL;

  // set deformable material type
  //if (strcmp(volumetricMeshFilename, "__none") != 0)
  //{
  //  if (strcmp(deformableObjectMethod, "StVK") == 0)
  //    deformableObject = STVK;
  //  if (strcmp(deformableObjectMethod, "CLFEM") == 0)
  //    deformableObject = COROTLINFEM;
  //  if (strcmp(deformableObjectMethod, "LinearFEM") == 0)
  //    deformableObject = LINFEM;
  //  if (strcmp(deformableObjectMethod, "InvertibleFEM") == 0)
  //    deformableObject = INVERTIBLEFEM;
  //}

  //if (strcmp(massSpringSystemObjConfigFilename, "__none") != 0)
  //  massSpringSystemSource = OBJ;
  //else if (strcmp(massSpringSystemTetMeshConfigFilename, "__none") != 0)
  //  massSpringSystemSource = TETMESH;
  //else if (strcmp(massSpringSystemCubicMeshConfigFilename, "__none") != 0)
  //  massSpringSystemSource = CUBICMESH;
  //else if (strncmp(customMassSpringSystem, "chain", 5) == 0)
  //  massSpringSystemSource = CHAIN;

  //if ((massSpringSystemSource == OBJ) || (massSpringSystemSource == TETMESH) || (massSpringSystemSource == CUBICMESH) || (massSpringSystemSource == CHAIN)) 
  //  deformableObject = MASSSPRING;

  massSpringSystemSource = CHAIN;
  deformableObject = MASSSPRING;


  if (deformableObject == UNSPECIFIED)
  {
    printf("Error: no deformable model specified.\n");
    exit(1);
  }

  // load mesh
  if ((deformableObject == STVK) || (deformableObject == COROTLINFEM) || (deformableObject == LINFEM) || (deformableObject == INVERTIBLEFEM))
  {
    printf("Loading volumetric mesh from file %s...\n", volumetricMeshFilename);

    volumetricMesh = VolumetricMeshLoader::load(volumetricMeshFilename);
    if (volumetricMesh == NULL)
    {
      printf("Error: unable to load the volumetric mesh from %s.\n", volumetricMeshFilename);
      exit(1);
    }

    n = volumetricMesh->getNumVertices();
    printf("Num vertices: %d. Num elements: %d\n",n, volumetricMesh->getNumElements());
    meshGraph = GenerateMeshGraph::Generate(volumetricMesh);

    // load mass matrix
    if (strcmp(massMatrixFilename, "__none") == 0)
    {
      printf("Error: mass matrix for the StVK deformable model not specified (%s).\n", massMatrixFilename);
      exit(1);
    }

    printf("Loading the mass matrix from file %s...\n", massMatrixFilename);
    // get the mass matrix
    SparseMatrixOutline * massMatrixOutline;
    try
    {
      massMatrixOutline = new SparseMatrixOutline(massMatrixFilename, 3); // 3 is expansion flag to indicate this is a mass matrix; and does 3x3 identity block expansion
    }
    catch(int exceptionCode)
    {
      printf("Error loading mass matrix %s.\n", massMatrixFilename);
      exit(1);
    }

    massMatrix = new SparseMatrix(massMatrixOutline);
    delete(massMatrixOutline);

    if (deformableObject == STVK || deformableObject == LINFEM)  //LINFEM constructed from stVKInternalForces
    {
      unsigned int loadingFlag = 0; // 0 = use low-memory version, 1 = use high-memory version
      StVKElementABCD * precomputedIntegrals = StVKElementABCDLoader::load(volumetricMesh, loadingFlag);
      if (precomputedIntegrals == NULL)
      {
	printf("Error: unable to load the StVK integrals.\n");
	exit(1);
      }

      printf("Generating internal forces and stiffness matrix models...\n"); fflush(NULL);
      if (numInternalForceThreads == 0)
        stVKInternalForces = new StVKInternalForces(volumetricMesh, precomputedIntegrals, addGravity, g);
      else
        stVKInternalForces = new StVKInternalForcesMT(volumetricMesh, precomputedIntegrals, addGravity, g, numInternalForceThreads);

      if (numInternalForceThreads == 0)
        stVKStiffnessMatrix = new StVKStiffnessMatrix(stVKInternalForces);
      else
        stVKStiffnessMatrix = new StVKStiffnessMatrixMT(stVKInternalForces, numInternalForceThreads);
    }
  }

  // load mass spring system (if any) 
  if (deformableObject == MASSSPRING)
  {
    switch (massSpringSystemSource)
    {
      case OBJ:
	{
	  printf("Loading mass spring system from an obj file...\n");
	  MassSpringSystemFromObjMeshConfigFile massSpringSystemFromObjMeshConfigFile;
	  MassSpringSystemObjMeshConfiguration massSpringSystemObjMeshConfiguration;
	  if (massSpringSystemFromObjMeshConfigFile.GenerateMassSpringSystem(massSpringSystemObjConfigFilename, &massSpringSystem, &massSpringSystemObjMeshConfiguration) != 0) 
	  {
	    printf("Error initializing the mass spring system.\n");
	    exit(1);
	  }
	  strcpy(renderingMeshFilename, massSpringSystemObjMeshConfiguration.massSpringMeshFilename);
	}
	break;

      case TETMESH:
	{
	  printf("Loading mass spring system from a tet mesh file...\n");
	  MassSpringSystemFromTetMeshConfigFile massSpringSystemFromTetMeshConfigFile;
	  MassSpringSystemTetMeshConfiguration massSpringSystemTetMeshConfiguration;
	  if (massSpringSystemFromTetMeshConfigFile.GenerateMassSpringSystem(massSpringSystemTetMeshConfigFilename, &massSpringSystem, &massSpringSystemTetMeshConfiguration) != 0)
	  {
	    printf("Error initializing the mass spring system.\n");
	    exit(1);
	  }
	  strcpy(renderingMeshFilename, massSpringSystemTetMeshConfiguration.surfaceMeshFilename);
	}
	break;

      case CUBICMESH:
	{
	  printf("Loading mass spring system from a cubic mesh file...\n");
	  MassSpringSystemFromCubicMeshConfigFile massSpringSystemFromCubicMeshConfigFile;
	  MassSpringSystemCubicMeshConfiguration massSpringSystemCubicMeshConfiguration;
	  if (massSpringSystemFromCubicMeshConfigFile.GenerateMassSpringSystem(massSpringSystemCubicMeshConfigFilename, &massSpringSystem, &massSpringSystemCubicMeshConfiguration) != 0)
	  {
	    printf("Error initializing the mass spring system.\n");
	    exit(1);
	  }
	  strcpy(renderingMeshFilename, massSpringSystemCubicMeshConfiguration.surfaceMeshFilename);
	}
	break;

      case CHAIN:
	{
	  int numParticles;        
	  double groupStiffness;
	  sscanf(customMassSpringSystem, "chain,%d,%lf", &numParticles, &groupStiffness);
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

	  int * edgeGroups = (int*) malloc (sizeof(int) * (numParticles - 1));
	  for(int i=0; i<numParticles-1; i++)
	    edgeGroups[i] = 0;
	  double groupDamping = 0;

	  massSpringSystem = new MassSpringSystem(numParticles, masses, restPositions, numParticles - 1, edges, edgeGroups, 1, &groupStiffness, &groupDamping, addGravity);

	  char s[96];
	  sprintf(s,"chain-%d.obj", numParticles);
	  massSpringSystem->CreateObjMesh(s);
	  strcpy(renderingMeshFilename, s);

	  free(edgeGroups);
	  free(edges);
	  free(restPositions);
	  free(masses);

	  renderVertices = 1;
	}
	break;

      default:
	printf("Error: mesh spring system configuration file was not specified.\n");
	exit(1);
	break;
    }

    if (addGravity)
      massSpringSystem->SetGravity(addGravity, g);

    if (numInternalForceThreads > 0)
    {
      printf("Launching threaded internal force evaluation: %d threads.\n", numInternalForceThreads);
      MassSpringSystemMT * massSpringSystemMT = new MassSpringSystemMT(*massSpringSystem, numInternalForceThreads);
      delete(massSpringSystem);
      massSpringSystem = massSpringSystemMT;
    }

    n = massSpringSystem->GetNumParticles();

    // create the mass matrix
    massSpringSystem->GenerateMassMatrix(&massMatrix, 1);
    delete(massMatrix);
    massSpringSystem->GenerateMassMatrix(&massMatrix);

    // create the mesh graph (used only for the distribution of user forces over neighboring vertices)
    meshGraph = new Graph(massSpringSystem->GetNumParticles(), massSpringSystem->GetNumEdges(), massSpringSystem->GetEdges());
  }

  int scaleRows = 1;
  meshGraph->GetLaplacian(&LaplacianDampingMatrix, scaleRows);
  LaplacianDampingMatrix->ScalarMultiply(dampingLaplacianCoef);

  // initialize the rendering mesh for the volumetric mesh
  if (strcmp(renderingMeshFilename, "__none") == 0)
  {
    printf("Error: rendering mesh was not specified.\n");
    exit(1);
  }
  deformableObjectRenderingMesh = new SceneObjectDeformable(renderingMeshFilename);
  if (enableTextures)
    deformableObjectRenderingMesh->SetUpTextures(SceneObject::MODULATE, SceneObject::NOMIPMAP);
  deformableObjectRenderingMesh->ResetDeformationToRest();
  deformableObjectRenderingMesh->BuildNeighboringStructure();
  deformableObjectRenderingMesh->BuildNormals(); 
  deformableObjectRenderingMesh->SetMaterialAlpha(0.5);

  // initialize the embedded triangle rendering mesh 
  secondaryDeformableObjectRenderingMesh = NULL;
  if (strcmp(secondaryRenderingMeshFilename, "__none") != 0)
  {
    secondaryDeformableObjectRenderingMesh = new SceneObjectDeformable(secondaryRenderingMeshFilename);
    if (enableTextures)
      secondaryDeformableObjectRenderingMesh->SetUpTextures(SceneObject::MODULATE, SceneObject::NOMIPMAP);
    secondaryDeformableObjectRenderingMesh->ResetDeformationToRest();
    secondaryDeformableObjectRenderingMesh->BuildNeighboringStructure();
    secondaryDeformableObjectRenderingMesh->BuildNormals(); 

    uSecondary = (double*) calloc (3 * secondaryDeformableObjectRenderingMesh->Getn(), sizeof(double));

    // load interpolation structure
    if (strcmp(secondaryRenderingMeshInterpolationFilename, "__none") == 0)
    {
      printf("Error: no secondary rendering mesh interpolation filename specified.\n");
      exit(1);
    }

    secondaryDeformableObjectRenderingMesh_interpolation_numElementVertices = VolumetricMesh::getNumInterpolationElementVertices(secondaryRenderingMeshInterpolationFilename);

    if (secondaryDeformableObjectRenderingMesh_interpolation_numElementVertices < 0)
    {
      printf("Error: unable to open file %s.\n", secondaryRenderingMeshInterpolationFilename);
      exit(1);
    }

    printf("Num interpolation element vertices: %d\n", secondaryDeformableObjectRenderingMesh_interpolation_numElementVertices);

    VolumetricMesh::loadInterpolationWeights(secondaryRenderingMeshInterpolationFilename, secondaryDeformableObjectRenderingMesh->Getn(), secondaryDeformableObjectRenderingMesh_interpolation_numElementVertices, &secondaryDeformableObjectRenderingMesh_interpolation_vertices, &secondaryDeformableObjectRenderingMesh_interpolation_weights);
  }
  else
    renderSecondaryDeformableObject = 0;

  if (!((deformableObject == MASSSPRING) && (massSpringSystemSource == CHAIN)))
  {
    // read the fixed vertices
    // 1-indexed notation
    if (strcmp(fixedVerticesFilename, "__none") == 0)
    {
      numFixedVertices = 0;
      fixedVertices = NULL;
    }
    else
    {
      if (LoadList::load(fixedVerticesFilename, &numFixedVertices,&fixedVertices) != 0)
      {
	printf("Error reading fixed vertices.\n");
	exit(1);
      }
      LoadList::sort(numFixedVertices, fixedVertices);
    }
  }
  else
  {
    numFixedVertices = 1;
    fixedVertices = (int*) malloc (sizeof(int) * numFixedVertices);
    fixedVertices[0] = massSpringSystem->GetNumParticles();
  }

  printf("Loaded %d fixed vertices. They are:\n",numFixedVertices);
  LoadList::print(numFixedVertices,fixedVertices);
  // create 0-indexed fixed DOFs
  int numFixedDOFs = 3 * numFixedVertices;
  int * fixedDOFs = (int*) malloc (sizeof(int) * numFixedDOFs);
  for(int i=0; i<numFixedVertices; i++)
  {
    fixedDOFs[3*i+0] = 3*fixedVertices[i]-3;
    fixedDOFs[3*i+1] = 3*fixedVertices[i]-2;
    fixedDOFs[3*i+2] = 3*fixedVertices[i]-1;
  }
  for(int i=0; i<numFixedVertices; i++)
    fixedVertices[i]--;
  printf("Boundary vertices processed.\n");

  // make room for deformation and force vectors
  u = (double*) calloc (3*n, sizeof(double));
  uvel = (double*) calloc (3*n, sizeof(double));
  uaccel = (double*) calloc (3*n, sizeof(double));
  f_ext = (double*) calloc (3*n, sizeof(double));
  f_extBase = (double*) calloc (3*n, sizeof(double));

  // load initial condition
  if (strcmp(initialPositionFilename, "__none") != 0)
  {
    int m1, n1;
    ReadMatrixFromDisk_(initialPositionFilename, &m1, &n1, &uInitial);
    if ((m1 != 3*n) || (n1 != 1))
    {
      printf("Error: initial position matrix size mismatch.\n");
      exit(1);
    }
  }
  else if ((deformableObject == MASSSPRING) && (massSpringSystemSource == CHAIN))
  {
    uInitial = (double*) calloc (3*n, sizeof(double));
    int numParticles = massSpringSystem->GetNumParticles(); 
    for(int i=0; i<numParticles; i++)
    {
      uInitial[3*i+0] = 1.0 - ((numParticles == 1) ? 1.0 : 1.0 * i / (numParticles - 1));
      uInitial[3*i+1] = 1.0 - ((numParticles == 1) ? 0.0 : 1.0 * i / (numParticles-1));
      uInitial[3*i+2] = 0.0;
    }
  }
  else
    uInitial = (double*) calloc (3*n, sizeof(double));

  // load initial velocity
  if (strcmp(initialVelocityFilename, "__none") != 0)
  {
    int m1, n1;
    ReadMatrixFromDisk_(initialVelocityFilename, &m1, &n1, &velInitial);
    if ((m1 != 3*n) || (n1 != 1))
    {
      printf("Error: initial position matrix size mismatch.\n");
      exit(1);
    }
  }

  // load force loads
  if (strcmp(forceLoadsFilename, "__none") != 0)
  {
    int m1;
    ReadMatrixFromDisk_(forceLoadsFilename, &m1, &numForceLoads, &forceLoads);
    if (m1 != 3*n)
    {
      printf("Mismatch in the dimension of the force load matrix.\n");
      exit(1);
    }
  }

  // create force models, to be used by the integrator
  printf("Creating force models...\n");
  if (deformableObject == STVK)
  {
    stVKForceModel = new StVKForceModel(stVKInternalForces, stVKStiffnessMatrix);
    forceModel = stVKForceModel;
    stVKForceModel->GetInternalForce(uInitial, u);
  }

  if (deformableObject == COROTLINFEM)
  {
    TetMesh * tetMesh = dynamic_cast<TetMesh*>(volumetricMesh);
    if (tetMesh == NULL)
    {
      printf("Error: the input mesh is not a tet mesh (CLFEM deformable model).\n");
      exit(1);
    }

    CorotationalLinearFEM * corotationalLinearFEM;

    if (numInternalForceThreads == 0)
      corotationalLinearFEM = new CorotationalLinearFEM(tetMesh);
    else
      corotationalLinearFEM = new CorotationalLinearFEMMT(tetMesh, numInternalForceThreads);

    corotationalLinearFEMForceModel = new CorotationalLinearFEMForceModel(corotationalLinearFEM, corotationalLinearFEM_warp);
    forceModel = corotationalLinearFEMForceModel;
  }

  if (deformableObject == LINFEM)
  {
    LinearFEMForceModel * linearFEMForceModel = new LinearFEMForceModel(stVKInternalForces);
    forceModel = linearFEMForceModel;
  }

  if (deformableObject == INVERTIBLEFEM)
  {
    TetMesh * tetMesh = dynamic_cast<TetMesh*>(volumetricMesh);
    if (tetMesh == NULL)
    {
      printf("Error: the input mesh is not a tet mesh (Invertible FEM deformable model).\n");
      exit(1);
    }

    IsotropicMaterial * isotropicMaterial = NULL;

    // create the invertible material model
    if (strcmp(invertibleMaterialString, "StVK") == 0)
      invertibleMaterial = INV_STVK;
    if (strcmp(invertibleMaterialString, "neoHookean") == 0)
      invertibleMaterial = INV_NEOHOOKEAN;
    if (strcmp(invertibleMaterialString, "MooneyRivlin") == 0)
      invertibleMaterial = INV_MOONEYRIVLIN;

    switch (invertibleMaterial)
    {
      case INV_STVK:
      {
      
	isotropicMaterial = new StVKIsotropicMaterial(tetMesh, enableCompressionResistance, compressionResistance);
	printf("Invertible material: StVK.\n");
	break;
      }

      case INV_NEOHOOKEAN:
	isotropicMaterial = new NeoHookeanIsotropicMaterial(tetMesh, enableCompressionResistance, compressionResistance);
	printf("Invertible material: neo-Hookean.\n");
	break;

      case INV_MOONEYRIVLIN:
	isotropicMaterial = new MooneyRivlinIsotropicMaterial(tetMesh, enableCompressionResistance, compressionResistance);
	printf("Invertible material: Mooney-Rivlin.\n");
	break;

      default:
	printf("Error: invalid invertible material type.\n");
	exit(1);
	break;
    }

    // create the invertible FEM deformable model
    IsotropicHyperelasticFEM * isotropicHyperelasticFEM;
    if (numInternalForceThreads == 0)
      isotropicHyperelasticFEM = new IsotropicHyperelasticFEM(tetMesh, isotropicMaterial, inversionThreshold, addGravity, g);
    else
      isotropicHyperelasticFEM = new IsotropicHyperelasticFEMMT(tetMesh, isotropicMaterial, inversionThreshold, addGravity, g, numInternalForceThreads);

    // create force model for the invertible FEM class
    IsotropicHyperelasticFEMForceModel * isotropicHyperelasticFEMForceModel = new IsotropicHyperelasticFEMForceModel(isotropicHyperelasticFEM);
    forceModel = isotropicHyperelasticFEMForceModel;
  }

  if (deformableObject == MASSSPRING)
  {
    massSpringSystemForceModel = new MassSpringSystemForceModel(massSpringSystem);
    forceModel = massSpringSystemForceModel;

    renderMassSprings = new RenderSprings();
  }

  // initialize the integrator
  printf("Initializing the integrator, n = %d...\n", n);
  printf("Solver type: %s\n", solverMethod);

  integratorBaseSparse = NULL;
  if (solver == IMPLICITNEWMARK)
  {
    implicitNewmarkSparse = new ImplicitNewmarkSparse(3*n, timeStep, massMatrix, forceModel, positiveDefinite, numFixedDOFs, fixedDOFs,
       dampingMassCoef, dampingStiffnessCoef, maxIterations, epsilon, newmarkBeta, newmarkGamma, numSolverThreads);
    integratorBaseSparse = implicitNewmarkSparse;
  }
  else if (solver == IMPLICITBACKWARDEULER)
  {
    implicitNewmarkSparse = new ImplicitBackwardEulerSparse(3*n, timeStep, massMatrix, forceModel, positiveDefinite, numFixedDOFs, fixedDOFs,
       dampingMassCoef, dampingStiffnessCoef, maxIterations, epsilon, numSolverThreads);
    integratorBaseSparse = implicitNewmarkSparse;
  }
  else if (solver == EULER)
  {
    int symplectic = 0;
    integratorBaseSparse = new EulerSparse(3*n, timeStep, massMatrix, forceModel, symplectic, numFixedDOFs, fixedDOFs, dampingMassCoef);
  }
  else if (solver == SYMPLECTICEULER)
  {
    int symplectic = 1;
    integratorBaseSparse = new EulerSparse(3*n, timeStep, massMatrix, forceModel, symplectic, numFixedDOFs, fixedDOFs, dampingMassCoef);
  }
  else if (solver == CENTRALDIFFERENCES)
  {
    integratorBaseSparse = new CentralDifferencesSparse(3*n, timeStep, massMatrix, forceModel, numFixedDOFs, fixedDOFs, dampingMassCoef, dampingStiffnessCoef, centralDifferencesTangentialDampingUpdateMode, numSolverThreads);
  }

  integratorBase = integratorBaseSparse;

  if (integratorBase == NULL)
  {
    printf("Error: failed to initialize numerical integrator.\n");
    exit(1);
  }

  // set integration parameters
  integratorBaseSparse->SetDampingMatrix(LaplacianDampingMatrix);
  integratorBase->ResetToRest();
  integratorBase->SetState(uInitial, velInitial);
  integratorBase->SetTimestep(timeStep / substepsPerTimeStep);

  if (implicitNewmarkSparse != NULL)
  {
    implicitNewmarkSparse->UseStaticSolver(staticSolver);
    if (velInitial != NULL)
      implicitNewmarkSparse->SetState(implicitNewmarkSparse->Getq(), velInitial);
  }

  // clear fps buffer
  for(int i=0; i<fpsBufferSize; i++)
    fpsBuffer[i] = 0.0;

  for(int i=0; i<forceAssemblyBufferSize; i++)
    forceAssemblyBuffer[i] = 0.0;

  for(int i=0; i<systemSolveBufferSize; i++)
    systemSolveBuffer[i] = 0.0;

  // load any external geometry file (e.g. some static scene for decoration; usually there will be none)
  if (strcmp(extraSceneGeometryFilename,"__none") != 0)
  {
    extraSceneGeometry = new SceneObject(extraSceneGeometryFilename);
    extraSceneGeometry->BuildNormals(85.0);
  }
  else
    extraSceneGeometry = NULL;

  // set up the ground plane (for rendering)
  renderGroundPlane = (strcmp(groundPlaneString, "__none") != 0);
  if (renderGroundPlane)
  {
    double groundPlaneR, groundPlaneG, groundPlaneB;
    double groundPlaneAmbient, groundPlaneDiffuse, groundPlaneSpecular, groundPlaneShininess;
    sscanf(groundPlaneString,"%lf,%lf,%lf,r%lf,g%lf,b%lf,a%lf,d%lf,s%lf,sh%lf", &groundPlaneHeight, &groundPlaneLightHeight, &groundPlaneSize, &groundPlaneR, &groundPlaneG, &groundPlaneB, &groundPlaneAmbient, &groundPlaneDiffuse, &groundPlaneSpecular, &groundPlaneShininess);
    displayListGround = glGenLists(1);
    //glNewList(displayListGround, GL_COMPILE);
//    RenderGroundPlane(groundPlaneHeight, groundPlaneR, groundPlaneG, groundPlaneB, groundPlaneAmbient, groundPlaneDiffuse, groundPlaneSpecular, groundPlaneShininess);
    //glEndList();
  }

  // set background color
  int colorR, colorG, colorB;
  sscanf(backgroundColorString, "%d %d %d", &colorR, &colorG, &colorB);
  //glClearColor(1.0 * colorR / 255, 1.0 * colorG / 255, 1.0 * colorB / 255, 0.0);

//  callAllUICallBacks();

  titleBarCounter.StartCounter();
}



