//
//  PickingPlugin.h
//  Preview3D
//
// Copyright 2013 - Christian Sch�ller 2013, schuellc@inf.ethz.ch
// Interactive Geometry Lab - ETH Zurich
//

#ifndef DEFORM_VEGA_H
#define DEFORM_VEGA_H

#include "ViewerPlugin.h"
#include "DeformerPicking.h"

#include "VegaMassSpringSystem.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

//class LIMSolver;
class DeformableMesh;
class TriangleMesh;
class TetrahedronMesh;

//defined by wangyu:
//typedef Eigen::Matrix<IndexType,  Eigen::Dynamic, 4, Eigen::RowMajor> TetMatrixType;

class DeformVega : public PreviewPlugin
{
public:
  bool enable_deform_locally_injective;
  bool connected_to_skinning;

  DeformVega();
  ~DeformVega();

  static DeformVega& GetReference();

  // implement Serializable interface
  bool Serialize(tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element);
  bool Deserialize(tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element);
  //void InitWithDefaultValue();

  void UpdateConstraintVertexPositions(const std::vector<IndexType>& constraintVertices, const Eigen::Matrix<double,Eigen::Dynamic,3>& positions);
  void UpdatePositionalConstraints(const std::vector<IndexType>& constraintVertices);
  
  // initialization (runs every time a mesh is loaded or cleared)
  void init(Preview3D *preview);
  
  // keyboard callback
  bool keyDownEvent(unsigned char key, int modifiers, int mouse_x, int mouse_y) ;
  
  //mouse callback
  bool mouseDownEvent(int mouse_x, int mouse_y, int button, int modifiers);
  bool mouseUpEvent(int mouse_x, int mouse_y, int button, int modifiers);
  bool mouseMoveEvent(int mouse_x, int mouse_y);
  bool mouseScrollEvent(int mouse_x, int mouse_y, float delta);
  
  //stuff that is drawn by the plugin before the previewer has displayed the mesh
  //first draw 3d, then 2d
  void preDraw(int currentTime);
  //stuff that is drawn by the plugin after the previewer has displayed the mesh
  //first draw 3d, then 2d
  void postDraw(int currentTime);

  void initDeformer();
protected: 
	int dim;

	//added by wangyu
	PointMatrixType *import_vertices;//used for the initial import of vertices from external mesh only
	FaceMatrixType *import_faces;//used for the initial import of vertices from external mesh only
	
	Eigen::Matrix<double,Eigen::Dynamic,3> *export_vertices;
	Eigen::Matrix<double,Eigen::Dynamic,3> *export_faces;
	Eigen::Matrix<int,Eigen::Dynamic,4> *export_tets;

	//Vega simulator
	VegaMassSpringSystem* vegaSolver;


  // Pointer to the tweak bar
  igl::ReTwBar* bar;

  //LIMSolver* solver;
  DeformableMesh* mesh;
  TriangleMesh* triMesh;
  TetrahedronMesh* tetMesh;

  int numElements;


  bool isMeshLoaded;
  bool showInvertedElements;
  bool runSolver;
  bool isTetMesh;
  bool enableBarriers;
  bool enableSubStepping;
  bool enableAlphaUpdate;
  bool enableOutput;

  double positionalConstraintError;
  double barrierWeight;
  double alpha;
  double error;
  
  bool createTriMesh();
  bool createTetMesh();
  
  static void TW_CALL prepare_dialog(void *clientData)
  {
	  static_cast<DeformVega *>(clientData)->prepare_connect();
  }
  void prepare_connect();// prepare to be connected to skinning deformer

  static void TW_CALL SetConnectSkinningCB(const void *value, void *clientData)
  {
	  static_cast<DeformVega*>(clientData)->SetConnectSkinning(*static_cast<const bool *>(value));
  }
  static void TW_CALL GetConnectSkinningCB(void *value, void *clientData)
  {
	  *static_cast<bool *>(value) = static_cast<const DeformVega*>(clientData)->GetConnectSkinning();
  }
  void SetConnectSkinning(bool connect)
  {
	  connected_to_skinning = connect;
	  prepare_connect();
  }
  bool GetConnectSkinning() const
  {
	  return connected_to_skinning;
  }

};

#endif