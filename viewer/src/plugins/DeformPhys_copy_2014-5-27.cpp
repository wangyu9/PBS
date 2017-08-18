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

#include "DeformPhys.h"
#include "PluginManager.h"
#include "PickingPlugin.h"

#include "igl/xml/ReAntTweakBarXMLSerialization.h"
#include "igl/ReAntTweakbar.h"
#define IGL_HEADER_ONLY 
#include "igl/tetgen/tetrahedralize.h"
#include "igl/boundary_faces.h"

// Used in solver
#include <igl/svd3x3/arap.h>
#include <igl/partition.h>
#include <igl/harmonic.h>

#include "DeformableMesh.h"
#include "TriangleMesh.h"
#include "TetrahedronMesh.h"

#include "DeformSkinning.h"



using namespace std;
//using namespace Eigen;

// Declare the single entity of the plugin
// This should be in all the plugin headers
DeformPhys  DeformPhysInstance;
DeformPhys&  DeformPhysInstance_(){ return DeformPhysInstance; }

igl::ARAPData arap_data;

DeformPhys& DeformPhys::GetReference()
{
  return DeformPhysInstance;
}

DeformPhys::DeformPhys():
enable_deform_locally_injective(INIT_ENABLE_DEFORM_PHYS),
connected_to_skinning(INIT_CONNECT_PHYS_TO_SKINNING)
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

  restart_solver = true;
  with_dynamics = false;
  max_iteration = 15;
  time_step = 0.1;
  add_gravity = false;
  gravity_factor = 5e-5;
  gravity_direction[0] = 0;
  gravity_direction[1] = -1;
  gravity_direction[2] = 0;

  enableBarriers = true;
  enableSubStepping = true;
  enableAlphaUpdate = true;
  enableOutput = true;

  export_vertices = new Eigen::Matrix<double,Eigen::Dynamic,3>;
  export_faces = new Eigen::Matrix<int,Eigen::Dynamic,3>;
  export_tets = new Eigen::Matrix<int,Eigen::Dynamic,4>;

  positionalConstraintError = 0;
  barrierWeight = 0;
  alpha = 0;

  numElements = 0;
};

DeformPhys::~DeformPhys()
{

};

void DeformPhys::UpdateConstraintVertexPositions(const vector<IndexType>& constraintVertices, const Matrix<double,Dynamic,3>& positions)
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
  
  //write constraints into solver
  //S.setZero();
  S.setConstant(-1);
  for(int i=0;i<constraintVertices.size();i++)
  {
	  int idx = constraintVertices[i];
	  S(idx) = 1;
	  for(int d=0;d<dim;d++)
	  {
		  U(idx,d) = positions.coeff(i,d);	  
	  }
  }

}

void DeformPhys::UpdatePositionalConstraints(const vector<IndexType>& constraintVertices)
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

  //write constraints into solver
  //S.setZero();
  S.setConstant(-1);
  for(int i=0;i<constraintVertices.size();i++)
  {
	  int idx = constraintVertices[i];
	  S(idx) = 1;
  }
}

void DeformPhys::ConstraintChanged()
{
	  restart_solver = true;
}

// initialization (runs every time a mesh is loaded or cleared)
void DeformPhys::init(Preview3D *preview)
{

  PreviewPlugin::init(preview);

  if(CONNECT_PHYS_TO_SKINNING)
  {
	  import_vertices = &DeformSkinning::GetReference().Handles;
	  import_faces = &DeformSkinning::GetReference().HandleFaces;
	  import_tets = &DeformSkinning::GetReference().HandleTets;
  }
  else
  {
	  import_vertices = m_preview->vertices;
	  import_faces = m_preview->faces;
	  import_tets = new Eigen::Matrix<unsigned,Eigen::Dynamic,4>;
  }

  isMeshLoaded = import_vertices->rows() > 0;
  
  // init menu bar
  if(bar == NULL)
  {
    // Create a tweak bar
    bar = new igl::ReTwBar;
    bar->TwNewBar("Phys");
    TwDefine(" Phys size='200 300' color='76 76 127' position='235 300' label='Phys Deformation' "); // change default tweak bar size and color
    bar->TwAddVarRW("Enable Deformer", TW_TYPE_BOOLCPP, &enable_deform_locally_injective, "");
	bar->TwAddVarRO("#Elements", TW_TYPE_INT32, &numElements, "");

	bar->TwAddVarCB("Connect to Skinning",TW_TYPE_BOOLCPP,SetConnectSkinningCB,GetConnectSkinningCB,this," label='Connect SKinning (Init)'");
	bar->TwAddButton("Connect Skinning (Init)", prepare_dialog, this,
		" label='Connect Skinning (Init)'");


    bar->TwAddVarRW("RunSolver", TW_TYPE_BOOLCPP, &runSolver, "label='Run Solver'");
	bar->TwAddVarRW("With Dynamics", TW_TYPE_BOOLCPP, &with_dynamics, "label='Dynamic or Quasi-static'");
	bar->TwAddVarRW("Max Iterations", TW_TYPE_INT32, &max_iteration, "label='Max Iterations' min=1 step=1");
	bar->TwAddVarRW("Time Step", TW_TYPE_DOUBLE, &time_step, "label='Time Step' min=0.0000000001");
	bar->TwAddVarRW("Add Gravity", TW_TYPE_BOOLCPP, &add_gravity, "label='Add Gravity'");
	bar->TwAddVarRW("Gravity Factor", TW_TYPE_DOUBLE, &gravity_factor, "label='Gravity Factor'");
	bar->TwAddVarRW( "LightDir", TW_TYPE_DIR3F, &gravity_direction, 
		" label='Gravity direction' open help='Change the Gravity direction.' ");

	bar->TwAddButton("Restart Solver", RestartSolver, this, " label='Restart Solver'");

    m_preview->enable_autoRefresh = true;
  }

  if(!enable_deform_locally_injective)
	  return;

  if(isMeshLoaded)
  {
	  if(import_tets->rows()!=0)
	  {
		  // simply coyp existing tet mesh
		  *export_vertices = *import_vertices;
		  *export_tets = (*import_tets).cast<int>();
		  *export_faces = (*import_faces).cast<int>();

		  isTetMesh = true;
		  //m_preview->set_toggle_ortho(false);
		  m_preview->enable_rotation = true;
	  }
	  else
	  {
		  if(createTetMesh())
		  {
			  isTetMesh = true;

			  //m_preview->set_toggle_ortho(false);
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
	  }

	initDeformer();
 
  }
}

void DeformPhys::initDeformer()
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
	
	//Init Solver here
	V = *export_vertices;
	S.resize(V.rows(),1);
	S.setConstant(-1);
	//S(0) = 1;
	//S(1) = 1;
	//S(2) = 1;
	//S(3) = 1;
	T = *export_tets;
	Faces = *export_faces;
	U = V;
	if(isTetMesh)
	{
		TF = T;
	}
	else
	{
		TF = Faces;
	}
	init_solver(V,TF,S);
}



bool DeformPhys::Serialize(tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element)
{
  return igl::save_ReAntTweakBar(bar,doc);
}

bool DeformPhys::Deserialize(tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element)
{
  return igl::load_ReAntTweakBar(bar,doc);
}

bool DeformPhys::keyDownEvent(unsigned char key, int modifiers, int mouse_x, int mouse_y)
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
bool DeformPhys::mouseDownEvent(int mouse_x, int mouse_y, int button, int modifiers)
{
  return false;
}

bool DeformPhys::mouseUpEvent(int mouse_x, int mouse_y, int button, int modifiers)
{
  return false;
}

bool DeformPhys::mouseMoveEvent(int mouse_x, int mouse_y)
{
  return false;
}

bool DeformPhys::mouseScrollEvent(int mouse_x, int mouse_y, float delta)
{
  return false;
}
  
//stuff that is drawn by the plugin before the previewer has displayed the mesh
//first draw 3d, then 2d
void DeformPhys::preDraw(int currentTime)
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
	  if(restart_solver)
		  init_solver(V,TF,S);
	  update_solver(S,U);
	  *mesh->DeformedVertices = U;

      
    // copy vertex state
    for(int r=0;r<import_vertices->rows();r++)
      for(int c=0;c<import_vertices->cols();c++)
        import_vertices->coeffRef(r,c) = mesh->DeformedVertices->coeff(r,c);

    // update face normals
	// wangyu turn off this
    //m_preview->recompute_face_normals();
    //m_preview->is_compiled = true;
  }

  if(CONNECT_PHYS_TO_SKINNING)
  {
	  //DeformSkinning::GetReference().applySkinning();
	  DeformSkinning::GetReference().update_skinning = true;
  }

}

//stuff that is drawn by the plugin after the previewer has displayed the mesh
//first draw 3d, then 2d
void DeformPhys::postDraw(int currentTime)
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

bool DeformPhys::createTriMesh()
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


  *export_vertices = *triMesh->InitalVertices;
  export_tets = new Matrix<int,Dynamic,4>();
  *export_faces = *triMesh->Triangles;

  return true;
}

bool DeformPhys::createTetMesh()
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

  *export_vertices = *tetMesh->InitalVertices;
  *export_tets = *tetMesh->Tetrahedra;
  export_faces = new Matrix<int,Dynamic,3>();
  export_faces->resize(tetFaces.size(),3);
  for(int i=0;i<tetFaces.size();i++)
  {
	  Eigen::Vector3i oneface;
	  for(int c=0;c<3;c++)
		  oneface[c] = tetFaces[i][c];

	  export_faces->row(i) = oneface;
  }

  return true;
}

void DeformPhys::prepare_connect()
{
	init(m_preview);
	PickingPlugin::GetReference().init_Picking();
}

int num_in_selection(const Eigen::VectorXi & S)
{
	int count = 0;
	for(int v = 0;v<S.rows(); v++)
	{
		if(S(v) >= 0)
		{
			count++;
		}
	}
	return count;
}

bool DeformPhys::init_solver(Eigen::MatrixXd& V, Eigen::MatrixXi &F, Eigen::VectorXi& S)
{	
	//using namespace Eigen;

	Eigen::VectorXi b(num_in_selection(S));
	assert(S.rows() == V.rows());
	Eigen::MatrixXd bc = Eigen::MatrixXd::Zero(b.size(),S.maxCoeff()+1);
	// get b from S
	{
		int bi = 0;
		for(int v = 0;v<S.rows(); v++)
		{
			if(S(v) >= 0)
			{
				b(bi) = v;
				bc(bi,S(v)) = 1;
				bi++;
			}
		}
	}
	// Store current mesh
	//U = V;
	Eigen::VectorXi _S;
	Eigen::VectorXd _D;
	Eigen::MatrixXd W;
	if(!igl::harmonic(V,F,b,bc,1,W))
	{
		return false;
	}
	igl::partition(W,100,arap_data.G,_S,_D);
	arap_data.with_dynamics = with_dynamics;
	arap_data.max_iter = max_iteration;
	arap_data.h = time_step;
	if(add_gravity)
	{
		f_ext.resize(V.rows(),3);
		//f_ext.setConstant(gravity_factor);
		Eigen::MatrixXd ones(V.rows(),1);
		ones.setOnes();
		f_ext.col(0).array() = gravity_direction[0]*gravity_factor*ones;
		f_ext.col(1).array() = gravity_direction[1]*gravity_factor*ones;
		f_ext.col(2).array() = gravity_direction[2]*gravity_factor*ones;
		//f_ext.setOnes();
		arap_data.f_ext = f_ext;
	}
	igl::arap_precomputation(V,F,b,arap_data);

	restart_solver = false;
	return true;
}


bool DeformPhys::update_solver(Eigen::VectorXi& S, Eigen::MatrixXd& V)
{

	Eigen::MatrixXd bc(num_in_selection(S),V.cols());
	// get b from S
	{
		int bi = 0;
		for(int v = 0;v<S.rows(); v++)
		{
			if(S(v) >= 0)
			{
				bc.row(bi) = V.row(v);
				bi++;
			}
		}
	}
	if(add_gravity)
	{
		arap_data.f_ext = f_ext;
	}
	if(!igl::arap_solve(bc, arap_data, U))
	{
		cerr<<"arap_solve failed."<<endl;
		return false;
	}
}
