// Copyright 2013 - Christian Schüller 2013, schuellc@inf.ethz.ch
// Interactive Geometry Lab - ETH Zurich

#ifdef __APPLE__
#   include <OpenGL/gl.h>
#   include <OpenGL/glu.h>
#   include <GLUT/glut.h>
#else
#   ifdef _WIN32
#       include <windows.h>
#       include <GL/glew.h>
#       include <GL/glut.h>
#   endif
#   include <GL/gl.h>
#   include <GL/glu.h>
#endif

#include <unsupported/Eigen/OpenGLSupport>

#include "DeformLocallyInjective.h"
#include "PluginManager.h"
#include "PickingPlugin.h"

#include "FileDialog.h"

#include "path_anttweak.h"

#ifdef USING_IGL_HEADER_ONLY_MODE
#define IGL_HEADER_ONLY 
#endif

#include "igl/tetgen/tetrahedralize.h"
#include "igl/boundary_faces.h"
#include "igl/writeMESH.h"
#include "igl/writeOBJ.h"
#include "igl/writeOFF.h"
#include "igl/writeTGF.h"
#include "igl/writeDMAT.h"

#include "LIMSolver2D.h"
#include "LIMSolver3D.h"

#include "DeformableMesh.h"
#include "TriangleMesh.h"
#include "TetrahedronMesh.h"

#include "Identity_LIMSolver2D.h"
#include "Dirichlet_LIMSolver2D.h"
#include "UniformLaplacian_LIMSolver2D.h"
#include "Laplacian_LIMSolver2D.h"
#include "GreenStrain_LIMSolver2D.h"
#include "LGARAP_LIMSolver2D.h"
#include "LSConformal_LIMSolver2D.h"
#include "Poisson_LIMSolver2D.h"
#include "MassSpring_LIMSolver2D.h"

#include "Identity_LIMSolver3D.h"
#include "Dirichlet_LIMSolver3D.h"
#include "UniformLaplacian_LIMSolver3D.h"
#include "Laplacian_LIMSolver3D.h"
#include "GreenStrain_LIMSolver3D.h"
#include "LGARAP_LIMSolver3D.h"
#include "MassSpring_LIMSolver3D.h"

#include "LIMSolverInterface.h"

#include "DeformSkinning.h"

#include "TimerWrapper.h"

using namespace std;
using namespace Eigen;

// Declare the single entity of the plugin
// This should be in all the plugin headers
DeformLocallyInjective  DeformLocallyInjectiveInstance;
DeformLocallyInjective&  DeformLocallyInjectiveInstance_(){ return DeformLocallyInjectiveInstance; }

DeformLocallyInjective& DeformLocallyInjective::GetReference()
{
  return DeformLocallyInjectiveInstance;
}

DeformLocallyInjective::DeformLocallyInjective():
enable_deform_locally_injective(INIT_ENABLE_DEFORM_LOCALLY_INJECTIVE),
connected_to_skinning(INIT_CONNECT_LIM_TO_SKINNING)
{
  //check in with the manager
  PluginManager().register_plugin(this);
  
  bar = NULL;

  solver = NULL;
  mesh = NULL;
  triMesh = NULL;
  tetMesh = NULL;

  energyType = ARAP;

  isMeshLoaded = false;
  showInvertedElements = true;
  runSolver= false;
  isTetMesh = false;

  enableBarriers = true;
  enableSubStepping = true;
  enableAlphaUpdate = true;
  enableOutput = true;

  positionalConstraintError = 0;
  barrierWeight = 0;
  alpha = 0;

  numElements = 0;
};

DeformLocallyInjective::~DeformLocallyInjective()
{

};

void DeformLocallyInjective::UpdateConstraintVertexPositions(const vector<IndexType>& constraintVertices, const Matrix<double,Dynamic,3>& positions)
{
	if(!enable_deform_locally_injective)
		return;

  int dim = isTetMesh ? 3 : 2;
    
  for(int i=0;i<constraintVertices.size();i++)
  {
    int idx = constraintVertices[i];
    for(int d=0;d<dim;d++)
      mesh->ConstraintTargets->coeffRef(idx*dim+d) = positions.coeff(i,d);
  }
  
  solver->Restart();
}

void DeformLocallyInjective::UpdatePositionalConstraints(const vector<IndexType>& constraintVertices)
{
	if(!enable_deform_locally_injective)
		return;

  int dim = isTetMesh ? 3 : 2;

  // free all constraint vertices
  for(int i=0;i<mesh->InitalVertices->rows()*dim;i++)
    mesh->ConstraintMatrix->coeffRef(i,i) = 0;
  mesh->ConstraintTargets->setZero();

  // set new constraint vertices
  for(int i=0;i<constraintVertices.size();i++)
  {
    int idx = constraintVertices[i];
    for(int c=0;c<dim;c++)
    {
      mesh->ConstraintMatrix->coeffRef(idx*dim+c,idx*dim+c) = 1;
      mesh->ConstraintTargets->coeffRef(idx*dim+c) = mesh->DeformedVertices->coeff(idx,c);
    }
  }

  solver->UpdatePositionalConstraintMatrix();

  solver->Restart();
}

// initialization (runs every time a mesh is loaded or cleared)
void DeformLocallyInjective::init(Preview3D *preview)
{

  PreviewPlugin::init(preview);

  if(CONNECT_LIM_TO_SKINNING)
  {
	  import_vertices = &DeformSkinning::GetReference().Handles;
	  import_faces = &DeformSkinning::GetReference().HandleFaces;
  }
  else
  {
	  import_vertices = m_preview->vertices;
	  import_faces = m_preview->faces;
  }


  isMeshLoaded = import_vertices->rows() > 0;
  
  // init menu bar
  if(bar == NULL)
  {
    // Create a tweak bar
    bar = new igl::ReTwBar;
    bar->TwNewBar("LIMDeformation");
    TwDefine(" LIMDeformation size='200 300' color='76 76 127' position='235 300' label='LIM Deformation' "); // change default tweak bar size and color
    bar->TwAddVarRW("Enable Deformer", TW_TYPE_BOOLCPP, &enable_deform_locally_injective, "");
	bar->TwAddVarRO("#Elements", TW_TYPE_INT32, &numElements, "");

	bar->TwAddVarCB("Connect to Skinning",TW_TYPE_BOOLCPP,SetConnectSkinningCB,GetConnectSkinningCB,this," label='Connect SKinning (Init)'");
	bar->TwAddButton("Connect Skinning (Init)", prepare_dialog, this,
		" label='Connect Skinning (Init)'");
	bar->TwAddButton("Save Deformed Mesh", save_dialog_deformed_mesh, this, 
		" label='Save Deformed Mesh'");

    #define EnergyCount 9
	TwEnumVal energyEV[EnergyCount] = {{IDENTITY, "Identity"}, {DIRICHLET, "Dirichlet"}, {UNILAP, "Uniform Laplacian"}, {COTLAP, "Cotan Laplacian"}, {GREEN, "Green Strain"}, {ARAP, "ARAP"}, {LSC, "LSC (2D only)"}, {POISSON, "Poisson (2D only)"}, {MASSSPRING,"Mass Spring"}};
    TwType energy = TwDefineEnum("Energy", energyEV, EnergyCount);
    bar->TwAddVarCB("Energy", energy, SetEnergyCB, GetEnergyCB, this, "");
    bar->TwAddVarRW("ShowInvertions", TW_TYPE_BOOLCPP, &showInvertedElements, "label='Show Invertions'");
    bar->TwAddVarRW("RunSolver", TW_TYPE_BOOLCPP, &runSolver, "label='Run Solver'");

    bar->TwAddVarRW("Barriers", TW_TYPE_BOOLCPP, &enableBarriers, "group='Solver Options'");
    bar->TwAddVarRW("BarrierWeights", TW_TYPE_DOUBLE, &barrierWeight, "group='Solver Options'");
    bar->TwAddVarRW("SubStepping", TW_TYPE_BOOLCPP, &enableSubStepping, "group='Solver Options'");
    bar->TwAddVarRW("AlphaUpdate", TW_TYPE_BOOLCPP, &enableAlphaUpdate, "group='Solver Options'");
    bar->TwAddVarRW("Alpha/Ratio", TW_TYPE_DOUBLE, &alpha, "group='Solver Options'");
    bar->TwAddVarRW("Output", TW_TYPE_BOOLCPP, &enableOutput, "group='Solver Options'");

    bar->TwAddVarRO("PCError", TW_TYPE_DOUBLE, &error, "group='Solver Options'");

  }

  if(!enable_deform_locally_injective)
	  return;

  if(isMeshLoaded)
  {
    if(createTetMesh())
    {
      isTetMesh = true;

      m_preview->set_toggle_ortho(false);
      m_preview->enable_rotation = true;
    }
    else
    {
      isTetMesh = false;
      
      m_preview->set_toggle_ortho(true);
      m_preview->view_xy_plane();
      m_preview->enable_rotation = false;

      createTriMesh();
    }

    int dim = isTetMesh ? 3 : 2;

    mesh->ConstraintMatrix->resize(mesh->InitalVertices->rows()*dim,mesh->InitalVertices->rows()*dim);
    mesh->ConstraintTargets->resize(mesh->InitalVertices->rows()*dim);
    mesh->ConstraintTargets->setZero();

    // init with identity matrix in order to reserve single vertex constraints
    vector<Eigen::Triplet<double> > triplets;
    for(int i=0;i<mesh->InitalVertices->rows()*dim;i++)
      triplets.push_back(Triplet<double>(i,i,1));
    mesh->ConstraintMatrix->setFromTriplets(triplets.begin(),triplets.end());

    initEnergy();

    // free all constraint vertices as now hessian structure is already reserved
    for(int i=0;i<mesh->InitalVertices->rows()*dim;i++)
      mesh->ConstraintMatrix->coeffRef(i,i) = 0;

    solver->UpdatePositionalConstraintMatrix();
  }
}

bool DeformLocallyInjective::Serialize(tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element)
{
  return igl::save_ReAntTweakBar(bar,doc);
}

bool DeformLocallyInjective::Deserialize(tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element)
{
  return igl::load_ReAntTweakBar(bar,doc);
}

// keyboard callback
LIMData* data = NULL;
bool DeformLocallyInjective::keyDownEvent(unsigned char key, int modifiers, int mouse_x, int mouse_y)
{
	if(!enable_deform_locally_injective)
		return false;

  switch (key)
  {
    case '1':
    {
      TriangleMesh* triMesh = NULL;
      TetrahedronMesh* tetMesh = NULL;

      vector<int> borderVertices;

      Matrix<int,Dynamic,Dynamic>* elements;
      Matrix<double,Dynamic,1> gradients;
      if(isTetMesh)
      {
        tetMesh = static_cast<TetrahedronMesh*>(mesh);
        elements = new Matrix<int,Dynamic,Dynamic>(*tetMesh->Tetrahedra);
      }
      else
      {
        triMesh = static_cast<TriangleMesh*>(mesh);
        elements = new Matrix<int,Dynamic,Dynamic>(*triMesh->Triangles);
        for(int i=0;i<triMesh->BorderVertices->rows();i++)
        borderVertices.push_back(triMesh->BorderVertices->coeff(i));

        gradients.resize(triMesh->Triangles->rows()*2*2,1);
        for(int i=0;i<triMesh->Triangles->rows()*2*2;i++)
        {
          if(i<triMesh->Triangles->rows()*2)
            if(i%2 == 0)
              gradients(i) = 10;
            else
              gradients(i) = 0;
          else
            if(i%2 == 0)
              gradients(i) = 0;
            else
              gradients(i) = 1;
        }
      }

      int res = ComputeLIM(
        *mesh->DeformedVertices,
        *mesh->InitalVertices,
        *elements,
        borderVertices,
        gradients,
        *mesh->ConstraintMatrix,
        *mesh->ConstraintTargets,
        5,
        1e-8,
        100,
        true,
        true,
        true);

      delete elements;

      // copy vertex state
      for(int r=0;r<import_vertices->rows();r++)
        for(int c=0;c<import_vertices->cols();c++)
          import_vertices->coeffRef(r,c) = mesh->DeformedVertices->coeff(r,c);

      // update face normals, 
      m_preview->recompute_all_normals();

      //m_preview->is_compiled = false;//disable by wangyu

      return true;
    }
    break;

    case '2':
    {
      TriangleMesh* triMesh = NULL;
      TetrahedronMesh* tetMesh = NULL;

      vector<int> borderVertices;

      Matrix<int,Dynamic,Dynamic>* elements;
      Matrix<double,Dynamic,1> gradients;
      if(isTetMesh)
      {
        tetMesh = static_cast<TetrahedronMesh*>(mesh);
        elements = new Matrix<int,Dynamic,Dynamic>(*tetMesh->Tetrahedra);
      }
      else
      {
        triMesh = static_cast<TriangleMesh*>(mesh);
        elements = new Matrix<int,Dynamic,Dynamic>(*triMesh->Triangles);
        for(int i=0;i<triMesh->BorderVertices->rows();i++)
        borderVertices.push_back(triMesh->BorderVertices->coeff(i));

        gradients.resize(triMesh->Triangles->rows()*2*2,1);
        for(int i=0;i<triMesh->Triangles->rows()*2*2;i++)
        {
          if(i<triMesh->Triangles->rows()*2)
            if(i%2 == 0)
              gradients(i) = 10;
            else
              gradients(i) = 0;
          else
            if(i%2 == 0)
              gradients(i) = 0;
            else
              gradients(i) = 1;
        }
      }

      LIMData* data =  InitLIM(
        *mesh->DeformedVertices,
        *mesh->InitalVertices,
        *elements,
        borderVertices,
        gradients,
        *mesh->ConstraintMatrix,
        *mesh->ConstraintTargets,
        5,
        true,
        true);

      Matrix<double,Dynamic,3> m(triMesh->DeformedVertices->rows(),3);
      for(int i=0;i<2;i++)
        ComputeLIM_Step(data,m);
      *triMesh->DeformedVertices = m;

      // copy vertex state
      for(int r=0;r<import_vertices->rows();r++)
        for(int c=0;c<import_vertices->cols();c++)
          import_vertices->coeffRef(r,c) = triMesh->DeformedVertices->coeff(r,c);

      // update face normals 
      m_preview->recompute_all_normals();

      //m_preview->is_compiled = false;//disable by wangyu

      return true;
    }
    break;
  }

  return false;
}
  
//mouse callback
bool DeformLocallyInjective::mouseDownEvent(int mouse_x, int mouse_y, int button, int modifiers)
{
  return false;
}

bool DeformLocallyInjective::mouseUpEvent(int mouse_x, int mouse_y, int button, int modifiers)
{
  return false;
}

bool DeformLocallyInjective::mouseMoveEvent(int mouse_x, int mouse_y)
{
  return false;
}

bool DeformLocallyInjective::mouseScrollEvent(int mouse_x, int mouse_y, float delta)
{
  return false;
}
  
//stuff that is drawn by the plugin before the previewer has displayed the mesh
//first draw 3d, then 2d
void DeformLocallyInjective::preDraw(int currentTime)
{
	static bool old_enable_deform_locally_injective = enable_deform_locally_injective;
	if(!enable_deform_locally_injective)
		return;
	else
		if(old_enable_deform_locally_injective!=enable_deform_locally_injective)
		{
			init(m_preview);
		}


  if(isMeshLoaded && runSolver)
  {
    if( solver->EnableBarriers != enableBarriers
      || solver->EnableSubstepping != enableSubStepping
      || solver->EnableAlpaUpdate != enableAlphaUpdate
      || (solver->EnableAlpaUpdate && solver->AlphaRatio != alpha)
      || (!solver->EnableAlpaUpdate && solver->Alpha != alpha)
      || solver->Beta != barrierWeight)
    {
      if(solver->EnableAlpaUpdate != enableAlphaUpdate)
      {
        if(enableAlphaUpdate)
          alpha = solver->AlphaRatio;
        else
          alpha = 1e8;
      }
      
      solver->EnableBarriers = enableBarriers;
      solver->EnableSubstepping = enableSubStepping;
      solver->EnableAlpaUpdate = enableAlphaUpdate;

      if(solver->EnableAlpaUpdate)
        solver->AlphaRatio = alpha;
      else
        solver->Alpha = alpha;

      solver->Beta = barrierWeight;

      solver->Restart();
    }
    solver->EnableOutput = enableOutput;
    
	TimerWrapper timeWrapper;
	timeWrapper.Tic();

	solver->Solve();

	timeWrapper.Toc();
	printf("LIM Compuring time: %f\n",timeWrapper.Duration());

    error = solver->CurrentPositionalEnergy;

    // switch vertex buffers
    Matrix<double,Dynamic,3>* temp = mesh->DeformedVertices;
    mesh->DeformedVertices = mesh->PredictedVertices;
    mesh->PredictedVertices = temp;
      
    // copy vertex state
    for(int r=0;r<import_vertices->rows();r++)
      for(int c=0;c<import_vertices->cols();c++)
        import_vertices->coeffRef(r,c) = mesh->DeformedVertices->coeff(r,c);

    // update face normals 
    m_preview->recompute_all_normals();
	//m_preview->is_compiled = false;//wangyu disables this
  }

  

  if(CONNECT_LIM_TO_SKINNING)
  {
	  //DeformSkinning::GetReference().applySkinning();
	  DeformSkinning::GetReference().update_skinning = true;
  }

}

//stuff that is drawn by the plugin after the previewer has displayed the mesh
//first draw 3d, then 2d
void DeformLocallyInjective::postDraw(int currentTime)
{
	if(!enable_deform_locally_injective)
		return;

  if(isMeshLoaded && showInvertedElements)
  {
    glPushMatrix();

    float mat[4*4];
    Preview3D::ConvertQuaternionToMatrix(m_preview->g_Rotation, mat);
    glMultMatrixf(mat);
    glScaled(m_preview->g_Zoom, m_preview->g_Zoom, m_preview->g_Zoom);
    glScaled(m_preview->zoom, m_preview->zoom, m_preview->zoom);
    glTranslatef(m_preview->g_Translation[0],m_preview->g_Translation[1],m_preview->g_Translation[2]);
    
    // Render flipped tets
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glColor3f(1,1,0);
    if(isTetMesh)
    {
      for(int t=0;t<tetMesh->Tetrahedra->rows();t++)
      {
        Vector4i index = tetMesh->Tetrahedra->row(t);

        Vector3 A = tetMesh->DeformedVertices->row(index[0]);
        Vector3 B = tetMesh->DeformedVertices->row(index[1]);
        Vector3 C = tetMesh->DeformedVertices->row(index[2]);
        Vector3 D = tetMesh->DeformedVertices->row(index[3]);

        Vector3 a = A-D;
        Vector3 b = B-D;
        Vector3 c = C-D;

        double det = a.dot(c.cross(b));

        if(det < 0)
        {
          glBegin(GL_TRIANGLES);
          glVertex(A);
          glVertex(B);
          glVertex(C);
          glEnd();

          glBegin(GL_TRIANGLES);
          glVertex(B);
          glVertex(C);
          glVertex(D);
          glEnd();

          glBegin(GL_TRIANGLES);
          glVertex(A);
          glVertex(C);
          glVertex(D);
          glEnd();

          glBegin(GL_TRIANGLES);
          glVertex(A);
          glVertex(B);
          glVertex(D);
          glEnd();
        }
      }
    }
    else
    {
      // Draw flipped triangles
      for(int t=0;t<triMesh->Triangles->rows();t++)
      {
        Eigen::Vector3i index = triMesh->Triangles->row(t);

        Vector2 A = triMesh->DeformedVertices->row(index[0]).block<1,2>(0,0);;
        Vector2 B = triMesh->DeformedVertices->row(index[1]).block<1,2>(0,0);;
        Vector2 C = triMesh->DeformedVertices->row(index[2]).block<1,2>(0,0);;

        Matrix2d V;
        V.row(0) = A-C;
        V.row(1) = B-C;

        double det = V.determinant();

        if(det < 0)
        {
          glBegin(GL_TRIANGLES);
          glVertex(A);
          glVertex(B);
          glVertex(C);
          glEnd();
        }
      }
    }

    glPopMatrix();
  
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
  }
}

bool DeformLocallyInjective::createTriMesh()
{
  // init triangle mesh
  triMesh = new TriangleMesh();  
  
  PickingPlugin& p =  PickingPlugin::GetReference();
  
  triMesh->InitalVertices = new Matrix<double,Dynamic,3>(*import_vertices);
  triMesh->DeformedVertices = new Matrix<double,Dynamic,3>(*import_vertices);
  triMesh->PredictedVertices = new Matrix<double,Dynamic,3>(*import_vertices);

  const int numVertices = import_vertices->rows();
  const int numTriangles = import_faces->rows();

  triMesh->Triangles = new Matrix<int,Dynamic,3>();
  triMesh->Triangles->resize(numTriangles,3);
  for(int t=0;t<numTriangles;t++)
    for(int i=0;i<3;i++)
      triMesh->Triangles->coeffRef(t,i) = import_faces->coeff(t,i);

  vector<vector<int> > verticesVec;
  for(int r=0;r<numTriangles;r++)
  {
    vector<int> v;
    for(int c=0;c<3;c++)
      v.push_back(triMesh->Triangles->coeff(r,c));

    verticesVec.push_back(v);
  }

  vector<vector<int> > borderEdges;
  igl::boundary_faces(verticesVec,borderEdges);

  map<int,int> edges;
  for(int i=0;i<borderEdges.size();i++)
    edges[borderEdges[i][0]] = borderEdges[i][1];

  triMesh->BorderVertices = new Matrix<int,Dynamic,1>();
  triMesh->BorderVertices->resize(borderEdges.size());
  
  map<int,int>::iterator iter = edges.begin();
  int count = 0;
  do
  {
    triMesh->BorderVertices->coeffRef(count++) = iter->first;
    iter = edges.find(iter->second);
  } while(iter != edges.begin());

  triMesh->ConstraintMatrix = new SparseMatrix<double>();
  triMesh->ConstraintTargets = new Matrix<double,Dynamic,1>();

  triMesh->InitMesh();

  numElements = numTriangles;
  mesh = triMesh;

  return true;
}

bool DeformLocallyInjective::createTetMesh()
{
  vector<vector<REAL> > verticesVec;
  int numVertices = import_vertices->rows();
  for(int r=0;r<numVertices;r++)
  {
    vector<REAL> v;
    for(int c=0;c<3;c++)
      v.push_back(import_vertices->coeff(r,c));

    verticesVec.push_back(v);
  }

  vector<vector<int> > faces;
  int numTriangles = import_faces->rows();
  for (int r=0;r<numTriangles;r++)
  {
    vector<int> v;
    for(int c=0;c<3;c++)
      v.push_back(import_faces->coeff(r,c));

    faces.push_back(v);
  }

  vector<vector<REAL> > verticesNew;
  vector<vector<int> > tetrahydra;
  vector<vector<int> > tetFaces;

  // Tetrahydralize volunme
  const std::string switches = "pq1.414a0.01";
  if(igl::tetrahedralize(verticesVec, faces, switches, verticesNew, tetrahydra, tetFaces) != 0)
    return false;

  // To load a simple tet
  /*tetrahydra.push_back(vector<int>());
  tetrahydra[0].push_back(0);
  tetrahydra[0].push_back(1);
  tetrahydra[0].push_back(2);
  tetrahydra[0].push_back(3);

  tetFaces = faces;
  verticesNew = verticesVec;*/

  tetMesh = new TetrahedronMesh();
  
  numVertices = verticesNew.size();

  // Create new object
  tetMesh->InitalVertices = new Matrix<double,Dynamic,3>();
  tetMesh->DeformedVertices = new Matrix<double,Dynamic,3>();
  tetMesh->PredictedVertices = new Matrix<double,Dynamic,3>();
  tetMesh->InitalVertices->resize(numVertices,3);
  tetMesh->DeformedVertices->resize(numVertices,3);
  tetMesh->PredictedVertices->resize(numVertices,3);

  for(int i=0;i<numVertices;i++)
  {
    Vector3d point;
    for(int c=0;c<3;c++)
      point[c] = verticesNew[i][c];

    tetMesh->InitalVertices->row(i) = point;
  }
  *tetMesh->DeformedVertices << *tetMesh->InitalVertices;
  *tetMesh->PredictedVertices << *tetMesh->InitalVertices;

  tetMesh->Tetrahedra = new Matrix<int,Dynamic,4>();
  tetMesh->Tetrahedra->resize(tetrahydra.size(),4);
  for(int i=0;i<tetrahydra.size();i++)
  {
    Vector4i tet;
    for(int c=0;c<4;c++)
      tet[c] = tetrahydra[i][c];

    tetMesh->Tetrahedra->row(i) = tet;
  }

  tetMesh->ConstraintMatrix = new SparseMatrix<double>();
  tetMesh->ConstraintTargets = new Matrix<double,Dynamic,1>();

  tetMesh->InitMesh();
  
  numElements = tetMesh->Tetrahedra->rows();
  mesh = tetMesh;

  return true;
}

void DeformLocallyInjective::initEnergy()
{
  delete solver;
  
  if(isTetMesh)
  {
    switch(energyType)
    {
      case IDENTITY:
      {
        solver = new Identity_LIMSolver3D();
      }
      break;

      case DIRICHLET:
      {
        solver = new Dirichlet_LIMSolver3D();
      }
      break;

      case UNILAP:
      {
        solver = new UniformLaplacian_LIMSolver3D();
      }
      break;

      case COTLAP:
      {
        solver = new Laplacian_LIMSolver3D();
      }
      break;

      case GREEN:
      {
        solver = new GreenStrain_LIMSolver3D();
      }
      break;

      case ARAP:
      {
        solver = new LGARAP_LIMSolver3D();
      }
      break;

      case LSC:
      {
        solver = new GreenStrain_LIMSolver3D();
        energyType = ARAP;
      }
      break;

      case POISSON:
      {
        solver = new GreenStrain_LIMSolver3D();
        energyType = ARAP;
      }
      break;

	  case MASSSPRING:
		{
			solver = new MassSpring_LIMSolver3D();
			energyType = MASSSPRING;
		}
		break;
    }
  }
  else
  {
    switch(energyType)
    {
      case IDENTITY:
      {
        solver = new Identity_LIMSolver2D();
      }
      break;

      case DIRICHLET:
      {
        solver = new Dirichlet_LIMSolver2D();
      }
      break;

      case UNILAP:
      {
        solver = new UniformLaplacian_LIMSolver2D();
      }
      break;

      case COTLAP:
      {
        solver = new Laplacian_LIMSolver2D();
      }
      break;

      case GREEN:
      {
        solver = new GreenStrain_LIMSolver2D();
      }
      break;

      case ARAP:
      {
        solver = new LGARAP_LIMSolver2D();
      }
      break;

      case LSC:
      {
        solver = new LSConformal_LIMSolver2D();
      }
      break;

      case POISSON:
      {
        Poisson_LIMSolver2D* poissonSolver = new Poisson_LIMSolver2D();
        Matrix<double,Dynamic,1> gradients(triMesh->Triangles->rows()*2*2,1);
        for(int i=0;i<triMesh->Triangles->rows()*2*2;i++)
        {
          if(i<triMesh->Triangles->rows()*2)
            if(i%2 == 0)
              gradients(i) = 10;
            else
              gradients(i) = 0;
          else
            if(i%2 == 0)
              gradients(i) = 0;
            else
              gradients(i) = 1;
        }
        poissonSolver->b = gradients;
        solver = poissonSolver;
      }
	  break;
	  case MASSSPRING:
		  {
			  solver = new MassSpring_LIMSolver2D();
		  }
      break;
    }
  }

  if(solver->EnableAlpaUpdate)
    alpha = solver->AlphaRatio;
  else
    alpha = solver->Alpha;
  
  barrierWeight = solver->Beta;

  if(mesh != NULL)
  {
    solver->Init(mesh);
  }
}

void DeformLocallyInjective::SetEnergy(EnergyType energy)
{
  energyType = energy;
  initEnergy();
}

EnergyType DeformLocallyInjective::GetEnergy() const
{
  return energyType;
}

void DeformLocallyInjective::prepare_connect()
{
	init(m_preview);
	PickingPlugin::GetReference().init_Picking();


	//if(!enable_deform_locally_injective)
	//	return;

	//if(isMeshLoaded)
	//{
	//	if(createTetMesh())
	//	{
	//		isTetMesh = true;

	//		m_preview->set_toggle_ortho(false);
	//		m_preview->enable_rotation = true;
	//	}
	//	else
	//	{
	//		isTetMesh = false;

	//		m_preview->set_toggle_ortho(true);
	//		m_preview->view_xy_plane();
	//		m_preview->enable_rotation = false;

	//		createTriMesh();
	//	}

	//	int dim = isTetMesh ? 3 : 2;

	//	mesh->ConstraintMatrix->resize(mesh->InitalVertices->rows()*dim,mesh->InitalVertices->rows()*dim);
	//	mesh->ConstraintTargets->resize(mesh->InitalVertices->rows()*dim);
	//	mesh->ConstraintTargets->setZero();

	//	// init with identity matrix in order to reserve single vertex constraints
	//	vector<Eigen::Triplet<double> > triplets;
	//	for(int i=0;i<mesh->InitalVertices->rows()*dim;i++)
	//		triplets.push_back(Triplet<double>(i,i,1));
	//	mesh->ConstraintMatrix->setFromTriplets(triplets.begin(),triplets.end());

	//	initEnergy();

	//	// free all constraint vertices as now hessian structure is already reserved
	//	for(int i=0;i<mesh->InitalVertices->rows()*dim;i++)
	//		mesh->ConstraintMatrix->coeffRef(i,i) = 0;

	//	solver->UpdatePositionalConstraintMatrix();
	//}

}

void DeformLocallyInjective::save_dialog_deformed_mesh(void *clientData)
{
	char fname[2048];

	fname[0] = 0;
	get_save_file_path(fname);

	if(fname[0] == 0)
		return;

	static_cast<DeformLocallyInjective *>(clientData)->save_deformed_mesh_to_file(fname);
}

bool DeformLocallyInjective::save_deformed_mesh_to_file(const char* mesh_file_name)
{
	//igl::writeMESH(mesh_file_name,*mesh->DeformedVertices,,*import_faces);
	igl::writeOBJ(mesh_file_name,*mesh->DeformedVertices,*import_faces);
	return true;
}