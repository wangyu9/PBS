#include <igl/svd3x3/arap.h>
#include <igl/writeDMAT.h>
#include <igl/partition.h>
#include <igl/harmonic.h>
#include <igl/cotmatrix.h>
#include <igl/massmatrix.h>
#include <igl/invert_diag.h>
#include <igl/OpenGL_convenience.h>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <igl/two_axis_valuator_fixed_up.h>
#include <igl/normalize_row_lengths.h>
#include <igl/draw_mesh.h>
#include <igl/draw_floor.h>
#include <igl/quat_to_mat.h>
#include <igl/report_gl_error.h>
#include <igl/readOBJ.h>
#include <igl/readDMAT.h>
#include <igl/readOFF.h>
#include <igl/readMESH.h>
#include <igl/jet.h>
#include <igl/readWRL.h>
#include <igl/trackball.h>
#include <igl/list_to_matrix.h>
#include <igl/snap_to_canonical_view_quat.h>
#include <igl/snap_to_fixed_up.h>
#include <igl/triangulate.h>
#include <igl/material_colors.h>
#include <igl/barycenter.h>
#include <igl/matlab_format.h>
#include <igl/material_colors.h>
#include <igl/ReAntTweakBar.h>
#include <igl/pathinfo.h>
#include <igl/Camera.h>
#include <igl/get_seconds.h>
#include <igl/PI.h>
#include <igl/STR.h>
#include <YImage.hpp>

#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif

#include <Eigen/Core>

#include <vector>
#include <iostream>
#include <algorithm>

struct State
{
  igl::Camera camera;
} s;
enum RotationType
{
  ROTATION_TYPE_IGL_TRACKBALL = 0,
  ROTATION_TYPE_TWO_AXIS_VALUATOR_FIXED_UP = 1,
  NUM_ROTATION_TYPES = 2,
} rotation_type;
bool is_rotating = false;
int down_x,down_y;
igl::Camera down_camera;
bool is_animating = false;
double animation_start_time = 0;
double ANIMATION_DURATION = 0.5;
Eigen::Quaterniond animation_from_quat;
Eigen::Quaterniond animation_to_quat;
// Use vector for range-based `for`
std::vector<State> undo_stack;
std::vector<State> redo_stack;

void push_undo()
{
  undo_stack.push_back(s);
  // Clear
  redo_stack = std::vector<State>();
}

void undo()
{
  using namespace std;
  if(!undo_stack.empty())
  {
    redo_stack.push_back(s);
    s = undo_stack.front();
    undo_stack.pop_back();
  }
}

void redo()
{
  using namespace std;
  if(!redo_stack.empty())
  {
    undo_stack.push_back(s);
    s = redo_stack.front();
    redo_stack.pop_back();
  }
}

void TW_CALL set_rotation_type(const void * value, void * clientData)
{
  using namespace Eigen;
  using namespace std;
  using namespace igl;
  const RotationType old_rotation_type = rotation_type;
  rotation_type = *(const RotationType *)(value);
  if(rotation_type == ROTATION_TYPE_TWO_AXIS_VALUATOR_FIXED_UP && 
    old_rotation_type != ROTATION_TYPE_TWO_AXIS_VALUATOR_FIXED_UP)
  {
    push_undo();
    animation_from_quat = s.camera.m_rotation_conj;
    snap_to_fixed_up(animation_from_quat,animation_to_quat);
    // start animation
    animation_start_time = get_seconds();
    is_animating = true;
  }
}

void TW_CALL get_rotation_type(void * value, void *clientData)
{
  RotationType * rt = (RotationType *)(value);
  *rt = rotation_type;
}

// Width and height of window
int width,height;
// Position of light
float light_pos[4] = {0.1,0.1,-0.9,0};
// Vertex positions, normals, colors and centroid
Eigen::MatrixXd V,U,N,C,mid;
Eigen::VectorXi S;
igl::ARAPData arap_data;
Eigen::MatrixXi F;
int selected_col = 0;
// Faces
// Bounding box diagonal length
double bbd;
int tot_num_samples = 0;
#define REBAR_NAME "temp.rbr"
igl::ReTwBar rebar; // Pointer to the tweak bar

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

bool init_arap()
{
  using namespace igl;
  using namespace Eigen;
  using namespace std;
  VectorXi b(num_in_selection(S));
  assert(S.rows() == V.rows());
  C.resize(S.rows(),3);
  MatrixXd bc = MatrixXd::Zero(b.size(),S.maxCoeff()+1);
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
        switch(S(v))
        {
          case 0:
            C.row(v) = RowVector3d(0.039,0.31,1);
            break;
          case 1:
            C.row(v) = RowVector3d(1,0.41,0.70);
            break;
          default:
            C.row(v) = RowVector3d(0.4,0.8,0.3);
            break;
        }
      }else
      {
        C.row(v) = RowVector3d(
          GOLD_DIFFUSE[0],
          GOLD_DIFFUSE[1],
          GOLD_DIFFUSE[2]);
      }
    }
  }
  // Store current mesh
  U = V;
  VectorXi _S;
  VectorXd _D;
  MatrixXd W;
  if(!igl::harmonic(V,F,b,bc,1,W))
  {
    return false;
  }
  igl::partition(W,100,arap_data.G,_S,_D);
  return igl::arap_precomputation(V,F,b,arap_data);
}

bool update_arap()
{
  using namespace Eigen;
  using namespace igl;
  using namespace std;
  MatrixXd bc(num_in_selection(S),V.cols());
  // get b from S
  {
    int bi = 0;
    for(int v = 0;v<S.rows(); v++)
    {
      if(S(v) >= 0)
      {
        bc.row(bi) = V.row(v);
        switch(S(v))
        {
          case 0:
          {
            const double r = mid(0)*0.25;
            bc(bi,0) += r*cos(0.5*get_seconds()*2.*PI);
            bc(bi,1) -= r+r*sin(0.5*get_seconds()*2.*PI);
            break;
          }
          case 1:
          {
            const double r = mid(1)*0.15;
            bc(bi,1) += r+r*cos(0.15*get_seconds()*2.*PI);
            bc(bi,2) -= r*sin(0.15*get_seconds()*2.*PI);

            //// Pull-up
            //bc(bi,0) += 0.42;//mid(0)*0.5;
            //bc(bi,1) += 0.55;//mid(0)*0.5;
            //// Bend
            //Vector3d t(-1,0,0);
            //Quaterniond q(AngleAxisd(PI/1.5,Vector3d(0,1.0,0.1).normalized()));
            //const Vector3d a = bc.row(bi);
            //bc.row(bi) = (q*(a-t) + t) + Vector3d(1.5,0.1,0.9);
                

            break;
          }
          default:
          break;
        }
        bi++;
      }
    }
  }
  if(!arap_solve(bc,arap_data,U))
  {
    cerr<<"arap_solve failed."<<endl;
    return false;
  }
  per_face_normals(U,F,N);
  return true;
}