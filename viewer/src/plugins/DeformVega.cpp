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

#include "DeformVega.h"
#include "PluginManager.h"
#include "PickingPlugin.h"

#include "igl/xml/ReAntTweakBarXMLSerialization.h"
#include "igl/ReAntTweakbar.h"
#define IGL_HEADER_ONLY 
#include "igl/tetgen/tetrahedralize.h"
#include "igl/boundary_faces.h"


#include "DeformableMesh.h"
#include "TriangleMesh.h"
#include "TetrahedronMesh.h"

//#include "LIMSolverInterface.h"

#include "DeformSkinning.h"

using namespace std;
//using namespace Eigen;

// Declare the single entity of the plugin
// This should be in all the plugin headers
DeformVega  DeformVegaInstance;
DeformVega&  DeformVegaInstance_(){ return DeformVegaInstance; }

DeformVega& DeformVega::GetReference()
{
  return DeformVegaInstance;
}

DeformVega::DeformVega():
enable_deform_locally_injective(INIT_ENABLE_DEFORM_VEGA),
connected_to_skinning(INIT_CONNECT_VEGA_TO_SKINNING)
{
  //check in with the manager
  PluginManager().register_plugin(this);
  
  bar = NULL;

//  solver = NULL;
  mesh = NULL;
  triMesh = NULL;
  tetMesh = NULL;


  isMeshLoaded = false;
  showInvertedElements = false;
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

DeformVega::~DeformVega()
{

};

void DeformVega::UpdateConstraintVertexPositions(const vector<IndexType>& constraintVertices, const Matrix<double,Dynamic,3>& positions)
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
  

  //write constraints into vega solver:
  int num_of_constraint_dofs = constraintVertices.size()*dim;
  int* pIndexConstraintDofIndices = new int[num_of_constraint_dofs];
  double* pConstraintCoordinates = new double[num_of_constraint_dofs]; 
  for(int i=0;i<constraintVertices.size();i++)
  {
	  int idx = constraintVertices[i];
	  for(int c=0;c<dim;c++)
	  {		   
		  pIndexConstraintDofIndices[i*dim+c] = idx*dim+c;
		  pConstraintCoordinates[i*dim+c] = positions.coeff(i,c);
	  }
  }
  vegaSolver->setState(num_of_constraint_dofs,pIndexConstraintDofIndices,pConstraintCoordinates);
  delete [] pIndexConstraintDofIndices;
  delete [] pConstraintCoordinates;


//  solver->Restart();
}

void DeformVega::UpdatePositionalConstraints(const vector<IndexType>& constraintVertices)
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



//  solver->UpdatePositionalConstraintMatrix();

//  solver->Restart();

  //write constraints into vega solver:
  int num_of_constraint_dofs = constraintVertices.size()*dim;
  int* pIndexConstraintDofIndices = new int[num_of_constraint_dofs];
  double* pConstraintCoordinates = new double[num_of_constraint_dofs]; 
  for(int i=0;i<constraintVertices.size();i++)
  {
	  int idx = constraintVertices[i];
	  for(int c=0;c<dim;c++)
	  {		   
		  pIndexConstraintDofIndices[i*dim+c] = idx*dim+c;
		  pConstraintCoordinates[i*dim+c] = mesh->DeformedVertices->coeff(idx,c);//positions.coeff(i,c);
	  }
  }
  vegaSolver->setState(num_of_constraint_dofs,pIndexConstraintDofIndices,pConstraintCoordinates);
  delete [] pIndexConstraintDofIndices;
  delete [] pConstraintCoordinates;


}

// initialization (runs every time a mesh is loaded or cleared)
void DeformVega::init(Preview3D *preview)
{

  PreviewPlugin::init(preview);

  if(CONNECT_VEGA_TO_SKINNING)
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
    bar->TwNewBar("Vega");
    TwDefine(" Vega size='200 300' color='76 76 127' position='235 300' label='Vega Deformation' "); // change default tweak bar size and color
    bar->TwAddVarRW("Enable Deformer", TW_TYPE_BOOLCPP, &enable_deform_locally_injective, "");
	bar->TwAddVarRO("#Elements", TW_TYPE_INT32, &numElements, "");

	bar->TwAddVarCB("Connect to Skinning",TW_TYPE_BOOLCPP,SetConnectSkinningCB,GetConnectSkinningCB,this," label='Connect SKinning (Init)'");
	bar->TwAddButton("Connect Skinning (Init)", prepare_dialog, this,
		" label='Connect SKinning (Init)'");


    bar->TwAddVarRW("ShowInvertions", TW_TYPE_BOOLCPP, &showInvertedElements, "label='Show Invertions'");
    bar->TwAddVarRW("RunSolver", TW_TYPE_BOOLCPP, &runSolver, "label='Run Solver'");

	//bar->TwAddButton("Init Deformer", dialog_init_deformer, this," label='Init Deformer'");

    //bar->TwAddVarRW("Barriers", TW_TYPE_BOOLCPP, &enableBarriers, "group='Solver Options'");
    //bar->TwAddVarRW("BarrierWeights", TW_TYPE_DOUBLE, &barrierWeight, "group='Solver Options'");
    //bar->TwAddVarRW("SubStepping", TW_TYPE_BOOLCPP, &enableSubStepping, "group='Solver Options'");
    //bar->TwAddVarRW("AlphaUpdate", TW_TYPE_BOOLCPP, &enableAlphaUpdate, "group='Solver Options'");
    //bar->TwAddVarRW("Alpha/Ratio", TW_TYPE_DOUBLE, &alpha, "group='Solver Options'");
    //bar->TwAddVarRW("Output", TW_TYPE_BOOLCPP, &enableOutput, "group='Solver Options'");

    bar->TwAddVarRO("PCError", TW_TYPE_DOUBLE, &error, "group='Solver Options'");

    m_preview->enable_autoRefresh = true;
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

	initDeformer();
 
  }
}

void DeformVega::initDeformer()
{
	dim = isTetMesh ? 3 : 2;

	mesh->ConstraintMatrix->resize(mesh->InitalVertices->rows()*dim,mesh->InitalVertices->rows()*dim);
	mesh->ConstraintTargets->resize(mesh->InitalVertices->rows()*dim);
	mesh->ConstraintTargets->setZero();

	// init with identity matrix in order to reserve single vertex constraints
	vector<Eigen::Triplet<double> > triplets;
	for(int i=0;i<mesh->InitalVertices->rows()*dim;i++)
		triplets.push_back(Triplet<double>(i,i,1));
	mesh->ConstraintMatrix->setFromTriplets(triplets.begin(),triplets.end());


	// free all constraint vertices as now hessian structure is already reserved
	for(int i=0;i<mesh->InitalVertices->rows()*dim;i++)
		mesh->ConstraintMatrix->coeffRef(i,i) = 0;

	//    solver->UpdatePositionalConstraintMatrix();




#if 0
	//put an spring at each triangle edge
	int numParticles = import_vertices->rows();	
	double *particles = new double[dim*numParticles];
	for(int i=0; i<numParticles; i++)
	{
		if(dim==3)
		{
			particles[i*dim+0] = (*import_vertices)(i,0);
			particles[i*dim+1] = (*import_vertices)(i,1);
			particles[i*dim+2] = (*import_vertices)(i,2);

			//printf("%f %f\n",(*import_vertices)(i,0),particles[i*dim+0]);
		}
		else
		{
			assert(dim==2);
			particles[i*dim+0] = (*import_vertices)(i,0);
			particles[i*dim+1] = (*import_vertices)(i,1);
		}
	}

	int numEdges = import_faces->rows()*3;
	int *edges = new int[numEdges*2];
	printf("Mesh Size: %d %d\n",numParticles,numEdges);
	for(int i=0; i<import_faces->rows(); i++)
	{
		edges[6*i+0] = (*import_faces)(i,0);
		edges[6*i+1] = (*import_faces)(i,1);
		edges[6*i+2] = (*import_faces)(i,1);
		edges[6*i+3] = (*import_faces)(i,2);
		edges[6*i+4] = (*import_faces)(i,2);
		edges[6*i+5] = (*import_faces)(i,0);
	}
#else
	//put an spring at each tet edge
	int numParticles = export_vertices->rows();	
	double *particles = new double[dim*numParticles];
	for(int i=0; i<numParticles; i++)
	{
		if(dim==3)
		{
			particles[i*dim+0] = (*export_vertices)(i,0);
			particles[i*dim+1] = (*export_vertices)(i,1);
			particles[i*dim+2] = (*export_vertices)(i,2);

			//printf("%f %f\n",(*import_vertices)(i,0),particles[i*dim+0]);
		}
		else
		{
			assert(dim==2);
			particles[i*dim+0] = (*export_vertices)(i,0);
			particles[i*dim+1] = (*export_vertices)(i,1);
		}
	}

	int numEdges = export_tets->rows()*6;
	int *edges = new int[numEdges*2];
	printf("Mesh Size: %d %d\n",numParticles,numEdges);
	for(int i=0; i<export_tets->rows(); i++)
	{
		edges[12*i+0] = (*export_tets)(i,0);
		edges[12*i+1] = (*export_tets)(i,1);
		edges[12*i+2] = (*export_tets)(i,1);
		edges[12*i+3] = (*export_tets)(i,2);
		edges[12*i+4] = (*export_tets)(i,2);
		edges[12*i+5] = (*export_tets)(i,0);

		edges[12*i+6+0] = (*export_tets)(i,0);
		edges[12*i+6+1] = (*export_tets)(i,3);
		edges[12*i+6+2] = (*export_tets)(i,1);
		edges[12*i+6+3] = (*export_tets)(i,3);
		edges[12*i+6+4] = (*export_tets)(i,2);
		edges[12*i+6+5] = (*export_tets)(i,3);
	}
#endif
	

	//write constraints into vega solver:
	std::vector<IndexType>& constraintVertices = PickingPlugin::GetReference().m_constrained_vertices;
	Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic>& positions =  PickingPlugin::GetReference().m_constrained_vertex_positions;
	int num_of_constraint_vertices = constraintVertices.size();
	int* pIndexConstraintVertex = new int[num_of_constraint_vertices];
	for(int i=0;i<constraintVertices.size();i++)
	{
		int idx = constraintVertices[i];		   
		pIndexConstraintVertex[i] = idx;
	}

	vegaSolver = new VegaMassSpringSystem( numParticles, 2, particles, numEdges, edges, 40000.0, 0, num_of_constraint_vertices, pIndexConstraintVertex, true);


	delete [] pIndexConstraintVertex;

	//delete [] particles;// do not delete!
	//delete [] edges;//do not delete!
}



bool DeformVega::Serialize(tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element)
{
  return igl::save_ReAntTweakBar(bar,doc);
}

bool DeformVega::Deserialize(tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element)
{
  return igl::load_ReAntTweakBar(bar,doc);
}

// keyboard callback
//LIMData* data = NULL;
bool DeformVega::keyDownEvent(unsigned char key, int modifiers, int mouse_x, int mouse_y)
{
	if(!enable_deform_locally_injective)
		return false;

  switch (key)
  {
    case '1':
    {
      return true;
    }
    break;

    case '2':
    {
      return true;
    }
    break;
  }

  return false;
}
  
//mouse callback
bool DeformVega::mouseDownEvent(int mouse_x, int mouse_y, int button, int modifiers)
{
  return false;
}

bool DeformVega::mouseUpEvent(int mouse_x, int mouse_y, int button, int modifiers)
{
  return false;
}

bool DeformVega::mouseMoveEvent(int mouse_x, int mouse_y)
{
  return false;
}

bool DeformVega::mouseScrollEvent(int mouse_x, int mouse_y, float delta)
{
  return false;
}
  
//stuff that is drawn by the plugin before the previewer has displayed the mesh
//first draw 3d, then 2d
void DeformVega::preDraw(int currentTime)
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
	  vegaSolver->doTimeStep(0.0001);
	  for(int i=0; i<mesh->DeformedVertices->rows(); i++)
	  {
		  if(dim==3)
		  {
			  //printf("%f %f\n",(*mesh->DeformedVertices)(i,0),vegaSolver->vertices[i*dim+0]);

			  (*mesh->DeformedVertices)(i,0) = vegaSolver->vertices[i*dim+0];
			  (*mesh->DeformedVertices)(i,1) = vegaSolver->vertices[i*dim+1];
			  (*mesh->DeformedVertices)(i,2) = vegaSolver->vertices[i*dim+2];

			  //(*mesh->DeformedVertices)(i,0) = (*mesh->InitalVertices)(i,0);
			  //(*mesh->DeformedVertices)(i,1) = (*mesh->InitalVertices)(i,1);
			  //(*mesh->DeformedVertices)(i,2) = (*mesh->InitalVertices)(i,2);
		  }
		  else
		  {
			  assert(dim==2);
			  (*mesh->DeformedVertices)(i,0) = vegaSolver->vertices[i*dim+0];
			  (*mesh->DeformedVertices)(i,1) = vegaSolver->vertices[i*dim+1];
		  }
	  }

    //if( solver->EnableBarriers != enableBarriers
    //  || solver->EnableSubstepping != enableSubStepping
    //  || solver->EnableAlpaUpdate != enableAlphaUpdate
    //  || (solver->EnableAlpaUpdate && solver->AlphaRatio != alpha)
    //  || (!solver->EnableAlpaUpdate && solver->Alpha != alpha)
    //  || solver->Beta != barrierWeight)
    //{
    //  if(solver->EnableAlpaUpdate != enableAlphaUpdate)
    //  {
    //    if(enableAlphaUpdate)
    //      alpha = solver->AlphaRatio;
    //    else
    //      alpha = 1e8;
    //  }
    //  
    //  solver->EnableBarriers = enableBarriers;
    //  solver->EnableSubstepping = enableSubStepping;
    //  solver->EnableAlpaUpdate = enableAlphaUpdate;

    //  if(solver->EnableAlpaUpdate)
    //    solver->AlphaRatio = alpha;
    //  else
    //    solver->Alpha = alpha;

    //  solver->Beta = barrierWeight;

    //  solver->Restart();
    //}
    //solver->EnableOutput = enableOutput;
    
    //solver->Solve();

    //error = solver->CurrentPositionalEnergy;

    // switch vertex buffers
    //Matrix<double,Dynamic,3>* temp = mesh->DeformedVertices;
    //mesh->DeformedVertices = mesh->PredictedVertices;
    //mesh->PredictedVertices = temp;
      
    // copy vertex state
    for(int r=0;r<import_vertices->rows();r++)
      for(int c=0;c<import_vertices->cols();c++)
        import_vertices->coeffRef(r,c) = mesh->DeformedVertices->coeff(r,c);

    // update face normals
	// wangyu turn off this
    //m_preview->recompute_face_normals();
    //m_preview->is_compiled = true;
  }

  if(CONNECT_VEGA_TO_SKINNING)
  {
	  DeformSkinning::GetReference().applySkinning();
  }

}

//stuff that is drawn by the plugin after the previewer has displayed the mesh
//first draw 3d, then 2d
void DeformVega::postDraw(int currentTime)
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

bool DeformVega::createTriMesh()
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

  triMesh->ConstraintMatrix = new Eigen::SparseMatrix<double>();
  triMesh->ConstraintTargets = new Matrix<double,Dynamic,1>();

  triMesh->InitMesh();

  numElements = numTriangles;
  mesh = triMesh;

  return true;
}

bool DeformVega::createTetMesh()
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

  tetMesh->ConstraintMatrix = new Eigen::SparseMatrix<double>();
  tetMesh->ConstraintTargets = new Matrix<double,Dynamic,1>();

  tetMesh->InitMesh();
  
  numElements = tetMesh->Tetrahedra->rows();
  mesh = tetMesh;

  export_vertices = tetMesh->InitalVertices;
  export_tets = tetMesh->Tetrahedra;

  return true;
}

void DeformVega::prepare_connect()
{
	init(m_preview);
	PickingPlugin::GetReference().init_Picking();
}

