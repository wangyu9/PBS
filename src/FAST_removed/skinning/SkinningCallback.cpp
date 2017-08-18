#ifndef IGL_HEADER_ONLY
#define IGL_HEADER_ONLY
#endif

#include "SkinningCallback.h"
#include "Skinning.h"
#include "gather_bones.h"
#include "Bone.h"
#include "Quat.h"
#include "matrix_from_string.h"
#include "ColorOption.h"
#include "gather_transformations.h"
#include "columnize.h"
#include "destroy_bone_roots.h"

// Standard library
#include <algorithm>
#include <iostream>
#include <cstdio>
#include <string>
#include <sstream>

// IGL library
#include <igl/canonical_quaternions.h>
#include <igl/snap_to_canonical_view_quat.h>
#include <igl/get_seconds.h>
#include <igl/transpose_blocks.h>
#include <igl/per_corner_normals.h>

#include <Eigen/Dense>

using namespace std;
using namespace igl;
using namespace Eigen;


inline Skinning * SkinningCallback::scast(void *clientData)
{
  return static_cast<Skinning *>(clientData);
}

void TW_CALL SkinningCallback::no_op(const void *value, void *clientData)
{
  // do nothing
}

void TW_CALL SkinningCallback::no_op(void *value, void *clientData)
{
  // do nothing
}

void TW_CALL SkinningCallback::view_xy_plane(void *clientData)
{
  Skinning * skinning = scast(clientData);
  copy(XY_PLANE_QUAT_F,XY_PLANE_QUAT_F+4,skinning->rotation);
}

void TW_CALL SkinningCallback::view_xz_plane(void *clientData)
{
  Skinning * skinning = scast(clientData);
  copy(XZ_PLANE_QUAT_F,XZ_PLANE_QUAT_F+4,skinning->rotation);
}

void TW_CALL SkinningCallback::view_yz_plane(void *clientData)
{
  Skinning * skinning = scast(clientData);
  copy(YZ_PLANE_QUAT_F,YZ_PLANE_QUAT_F+4,skinning->rotation);
}

// Check zoom lock before seting new zoom value
void TW_CALL SkinningCallback::set_zoom(const void *value, void *clientData)
{
  Skinning * skinning = scast(clientData);
  if(!skinning->zoom_locked)
  {
    skinning->zoom = *(const double*)(value);
  }
}
// Get current zoom value
void TW_CALL SkinningCallback::get_zoom(void *value, void *clientData)
{
  Skinning * skinning = scast(clientData);
  *(double *)(value) = skinning->zoom;
}

// Check angle lock before seting new angle value
void TW_CALL SkinningCallback::set_angle(const void *value, void *clientData)
{
  Skinning * skinning = scast(clientData);
  if(!skinning->angle_locked)
  {
    skinning->angle = *(const double*)(value);
  }
}
// Get current angle value
void TW_CALL SkinningCallback::get_angle(void *value, void *clientData)
{
  Skinning * skinning = scast(clientData);
  *(double *)(value) = skinning->angle;
}

// Get current number of vertices
void TW_CALL SkinningCallback::get_num_vertices(void * value, void *clientData)
{
  Skinning * skinning = scast(clientData);
  *(int *)(value) = skinning->V.rows();
}
// Get current number of faces
void TW_CALL SkinningCallback::get_num_faces(void * value, void *clientData)
{
  Skinning * skinning = scast(clientData);
  *(int *)(value) = skinning->F.rows();
}

void TW_CALL SkinningCallback::set_spinning_about_up_axis(
  const void *value, 
  void *clientData)
{
  Skinning * skinning = scast(clientData);
  skinning->spinning_about_up_axis = *(const bool*)(value);
  if(skinning->spinning_about_up_axis)
  {
    // If just started spinning then reset spin timer
    skinning->spin_start_seconds = get_seconds();
    copy(skinning->rotation,skinning->rotation+4,skinning->spin_start_rotation);
  }
}
void TW_CALL SkinningCallback::get_spinning_about_up_axis(void *value, void *clientData)
{
  Skinning * skinning = scast(clientData);
  *(bool *)(value) = skinning->spinning_about_up_axis;
}

void TW_CALL SkinningCallback::snap_rotation_to_canonical_view_quat(void *clientData)
{
  Skinning * skinning = scast(clientData);
  snap_to_canonical_view_quat<float>(
    skinning->rotation,
    1.0,
    skinning->rotation);
}

void TW_CALL SkinningCallback::clear(void *clientData)
{
  Skinning * skinning = scast(clientData);
  skinning->clear();
}

// Predicate to return bone is_selected field
static bool is_selected(Bone * b)
{
  return b->is_selected;
}

// Get weight index of selected bone
void TW_CALL SkinningCallback::get_selected_bone_wi(
  void *value, 
  void *clientData)
{
  Skinning * skinning = scast(clientData);
  // Get selected bone
  vector<Bone*> B = gather_bones(skinning->BR);
  vector<Bone*>::iterator bi = find_if(B.begin(),B.end(),is_selected);
  if(bi != B.end())
  {
    *(int *)(value) = (*bi)->get_wi();
  }

}

// Set and get the selected bone's rotation
void TW_CALL SkinningCallback::set_selected_bone_rotation(
  const void *value, 
  void *clientData)
{
  Skinning * skinning = scast(clientData);
  // Get selected bone
  vector<Bone*> B = gather_bones(skinning->BR);
  vector<Bone*>::iterator bi = find_if(B.begin(),B.end(),is_selected);
  if(bi != B.end())
  {
    // Deselect all others
    for_each((bi+1),B.end(),
        bind2nd(mem_fun(&Bone::set_is_tip_selected),false));
    for_each((bi+1),B.end(),
      bind2nd(mem_fun(&Bone::set_is_line_segment_selected),false));
    // case current value to double
    const double * quat = (const double *)(value);
    // Remove object rotation by left multiplying stored quaternion by object
    // rotation's inverse
    Quat obj_rot(
      skinning->rotation[3],
      skinning->rotation[0],
      skinning->rotation[1],
      skinning->rotation[2]);
    // Offset *ALL* rotations so that rotations are about the parent (self in
    // case of roots)
    Vec3 tail = (*bi)->offset;
    if(!(*bi)->is_root())
    {
      tail = (*bi)->get_parent()->rest_tip();
    }
    // Keep track of old translation
    Vec3 ot = ((*bi)->rotation*((*bi)->translation + tail))-tail;
    // Adjust rotation
    (*bi)->rotation = obj_rot.inverse() * Quat(quat[3],quat[0],quat[1],quat[2]);
    //if(skinning->center_root_rotations && (*bi)->is_root())
    (*bi)->translation = 
      (((*bi)->rotation.inverse() * (tail+ot)) - tail);
  }
}

void TW_CALL SkinningCallback::get_selected_bone_rotation(
  void *value, 
  void *clientData)
{
  Skinning * skinning = scast(clientData);
  // Get selected bone
  vector<Bone*> B = gather_bones(skinning->BR);
  vector<Bone*>::iterator bi = find_if(B.begin(),B.end(),is_selected);
  if(bi != B.end())
  {
    // case current value to double
    double * quat = (double *)(value);
    // Left multiply by object rotation
    Quat obj_rot(
      skinning->rotation[3],
      skinning->rotation[0],
      skinning->rotation[1],
      skinning->rotation[2]);
    Quat rot = obj_rot * (*bi)->rotation;
    quat[0] = rot.x();
    quat[1] = rot.y();
    quat[2] = rot.z();
    quat[3] = rot.w();
  }
}

// Set and get the selected bone's translation
void TW_CALL SkinningCallback::set_selected_bone_translation_x(
  const void *value, 
  void *clientData)
{
  Skinning * skinning = scast(clientData);
  // Get selected bone
  vector<Bone*> B = gather_bones(skinning->BR);
  vector<Bone*>::iterator bi = find_if(B.begin(),B.end(),is_selected);
  if(bi != B.end())
  {
    // Deselect all others
    for_each((bi+1),B.end(),
        bind2nd(mem_fun(&Bone::set_is_tip_selected),false));
    for_each((bi+1),B.end(),
      bind2nd(mem_fun(&Bone::set_is_line_segment_selected),false));
    // case current value to double
    const double * t = (const double *)(value);
    (*bi)->translation[0] = t[0];
  }
}
void TW_CALL SkinningCallback::get_selected_bone_translation_x(
  void *value, 
  void *clientData)
{
  Skinning * skinning = scast(clientData);
  // Get selected bone
  vector<Bone*> B = gather_bones(skinning->BR);
  vector<Bone*>::iterator bi = find_if(B.begin(),B.end(),is_selected);
  if(bi != B.end())
  {
    // case current value to double
    double * t = (double *)(value);
    t[0] = (*bi)->translation[0];
  }
}

// Set and get the selected bone's translation
void TW_CALL SkinningCallback::set_selected_bone_translation_y(
  const void *value, 
  void *clientData)
{
  Skinning * skinning = scast(clientData);
  // Get selected bone
  vector<Bone*> B = gather_bones(skinning->BR);
  vector<Bone*>::iterator bi = find_if(B.begin(),B.end(),is_selected);
  if(bi != B.end())
  {
    // Deselect all others
    for_each((bi+1),B.end(),
        bind2nd(mem_fun(&Bone::set_is_tip_selected),false));
    for_each((bi+1),B.end(),
      bind2nd(mem_fun(&Bone::set_is_line_segment_selected),false));
    // case current value to double
    const double * t = (const double *)(value);
    (*bi)->translation[1] = t[0];
  }
}
void TW_CALL SkinningCallback::get_selected_bone_translation_y(
  void *value, 
  void *clientData)
{
  Skinning * skinning = scast(clientData);
  // Get selected bone
  vector<Bone*> B = gather_bones(skinning->BR);
  vector<Bone*>::iterator bi = find_if(B.begin(),B.end(),is_selected);
  if(bi != B.end())
  {
    // case current value to double
    double * t = (double *)(value);
    t[0] = (*bi)->translation[1];
  }
}

// Set and get the selected bone's translation
void TW_CALL SkinningCallback::set_selected_bone_translation_z(
  const void *value, 
  void *clientData)
{
  Skinning * skinning = scast(clientData);
  // Get selected bone
  vector<Bone*> B = gather_bones(skinning->BR);
  vector<Bone*>::iterator bi = find_if(B.begin(),B.end(),is_selected);
  if(bi != B.end())
  {
    // Deselect all others
    for_each((bi+1),B.end(),
        bind2nd(mem_fun(&Bone::set_is_tip_selected),false));
    for_each((bi+1),B.end(),
      bind2nd(mem_fun(&Bone::set_is_line_segment_selected),false));
    // case current value to double
    const double * t = (const double *)(value);
    (*bi)->translation[2] = t[0];
  }
}
void TW_CALL SkinningCallback::get_selected_bone_translation_z(
  void *value, 
  void *clientData)
{
  Skinning * skinning = scast(clientData);
  // Get selected bone
  vector<Bone*> B = gather_bones(skinning->BR);
  vector<Bone*>::iterator bi = find_if(B.begin(),B.end(),is_selected);
  if(bi != B.end())
  {
    // case current value to double
    double * t = (double *)(value);
    t[0] = (*bi)->translation[2];
  }
}

void TW_CALL SkinningCallback::set_selected_bone_dof_type(
  const void *value, 
  void *clientData)
{
  Skinning * skinning = scast(clientData);
  // Get selected bone
  vector<Bone*> B = gather_bones(skinning->BR);
  vector<Bone*>::iterator bi = find_if(B.begin(),B.end(),is_selected);
  bool changed = false;
  while(bi != B.end())
  {
    // case current value to bool
    (*bi)->tip_dof_type = *(const DegreeOfFreedomType *)(value);
    changed = true;
    bi = find_if((bi+1),B.end(),is_selected);
  }
  // trigger reinitialization of arap dof
  if(changed && skinning->auto_dof)
  {
    skinning->reinitialize_auto_dof();
  }
}

void TW_CALL SkinningCallback::get_selected_bone_dof_type(
  void *value, 
  void *clientData)
{
  Skinning * skinning = scast(clientData);
  // Get selected bone
  vector<Bone*> B = gather_bones(skinning->BR);
  vector<Bone*>::iterator bi = find_if(B.begin(),B.end(),is_selected);
  if(bi != B.end())
  {
    *(DegreeOfFreedomType *)(value) = (*bi)->tip_dof_type;
  }
}

void TW_CALL SkinningCallback::set_selected_bone_last_T(
  const void *value, void *clientData)
{
  // Set: copy the value of s from AntTweakBar
  const std::string *srcPtr = static_cast<const std::string *>(value);
  string T_str = *srcPtr;


  MatrixXd T;
  T.resize(3,4);
  // Parse string into transform
  bool success = matrix_from_string(3,4,T_str,T);

  // return early if transform didn't make any sense
  if(!success)
  {
    return;
  }

  Skinning * skinning = scast(clientData);
  // Get selected bone
  vector<Bone*> B = gather_bones(skinning->BR);
  vector<Bone*>::iterator bi = find_if(B.begin(),B.end(),is_selected);
  bool changed = false;
  while(bi != B.end())
  {
    changed = true;
    (*bi)->last_T.affine() = T;
    // increment
    bi = find_if((bi+1),B.end(),is_selected);
  }
}

void TW_CALL SkinningCallback::get_selected_bone_last_T(
  void *value, 
  void *clientData)
{
  using namespace std;
  Skinning * skinning = scast(clientData);
  // Get selected bone
  vector<Bone*> B = gather_bones(skinning->BR);
  vector<Bone*>::iterator bi = find_if(B.begin(),B.end(),is_selected);
  stringstream T_stream;
  if(bi != B.end())
  {
    Tform3 T = (*bi)->last_T;
    // convert T into a string
    T_stream << 
      T(0,0) << "," << T(0,1) << "," << T(0,2) << "," << T(0,3) << ";" <<
      T(1,0) << "," << T(1,1) << "," << T(1,2) << "," << T(1,3) << ";" <<
      T(2,0) << "," << T(2,1) << "," << T(2,2) << "," << T(2,3);
  }

  // Get: copy the value of s to AntTweakBar
  std::string *destPtr = static_cast<std::string *>(value);
  // the use of TwCopyStdStringToLibrary is required here
//!!!  TwCopyStdStringToLibrary(*destPtr, T_stream.str());
}
// Snap current selection bone's rotation to canonical view quaternion
void TW_CALL 
  SkinningCallback::snap_selected_bone_rotation_to_canonical_view_quat(
  void *clientData)
{

  Skinning * skinning = scast(clientData);
  // Get selected bone
  vector<Bone*> B = gather_bones(skinning->BR);
  vector<Bone*>::iterator bi = find_if(B.begin(),B.end(),is_selected);
  while(bi != B.end())
  {
    double q[4];
    q[0] = (*bi)->rotation.x();
    q[1] = (*bi)->rotation.y();
    q[2] = (*bi)->rotation.z();
    q[3] = (*bi)->rotation.w();
    snap_to_canonical_view_quat<double>( q, 1.0, q);
    Quat new_rot(q[3],q[0],q[1],q[2]);

    // Offset *ALL* rotations so that rotations are about the parent (self in
    // case of roots)
    Vec3 tail = (*bi)->offset;
    if(!(*bi)->is_root())
    {
      tail = (*bi)->get_parent()->rest_tip();
    }
    // Keep track of old translation
    Vec3 ot = ((*bi)->rotation*((*bi)->translation + tail))-tail;
    // Adjust rotation
    (*bi)->rotation = new_rot;
    //if(skinning->center_root_rotations && (*bi)->is_root())
    (*bi)->translation = 
      (((*bi)->rotation.inverse() * (tail+ot)) - tail);
    bi = find_if((bi+1),B.end(),is_selected);
  }

}

void TW_CALL SkinningCallback::select_all(void *clientData)
{
  Skinning * skinning = scast(clientData);
  // Get selected bone
  vector<Bone*> B = gather_bones(skinning->BR);
  for_each(B.begin(),B.end(),
      bind2nd(mem_fun(&Bone::set_is_tip_selected),true));
}

void TW_CALL SkinningCallback::reset_selected_bone_transformation(void *clientData)
{
  Skinning * skinning = scast(clientData);
  // Get selected bone
  vector<Bone*> B = gather_bones(skinning->BR);
  vector<Bone*>::iterator bi = find_if(B.begin(),B.end(),is_selected);
  while(bi != B.end())
  {
    (*bi)->reset();
    // increment iterator and keep looking for selected bones
    bi = find_if((bi+1),B.end(),is_selected);
  }
}

void TW_CALL SkinningCallback::reset_extra_transformations(void * clientData)
{
  Skinning * skinning = scast(clientData);
  // Zero out extra transformations, by zeroing out all
  skinning->T *= 0;
  // Re-gather T of non extras 
  bool gather_success = 
    gather_transformations(skinning->BR,skinning->dial_in_each_T,skinning->T);
  if(!gather_success)
  {
    return;
  }
  if(skinning->auto_dof)
  {
    int m = skinning->arap_dof.m;
    if(m != skinning->T.cols()/4)
    {
      fprintf(stderr,
        "Error: transformations()" 
        "number of handles in T (%d) doesn't match that in arap_dof data (%d)\n" 
        "you must reinitialize.\n",
        (int)(skinning->T.cols()/4),
        m);
      skinning->auto_dof = false;
      return;
    }
    MatrixXf Tcol;
    // number of dimensions
    columnize<float,Dynamic>(
      skinning->T.block(
        0,0,skinning->arap_dof.dim,skinning->T.cols()).eval(),m,2,Tcol);
    // Replace last solution with current T
    skinning->L = Tcol.cast<double>().eval();
  }
}

void TW_CALL SkinningCallback::reset_all_bone_transformations(void *clientData)
{
  Skinning * skinning = scast(clientData);
  // Get selected bone
  vector<Bone*> B = gather_bones(skinning->BR);
  // call reset() for each bone
  for_each(B.begin(),B.end(),mem_fun(&Bone::reset));
  if(skinning->auto_dof)
  {
    skinning->initialize_transformations();
  }
}

void TW_CALL SkinningCallback::set_auto_dof(const void *value, void *clientData)
{
  Skinning * skinning = scast(clientData);
  bool old_value = skinning->auto_dof;
  skinning->auto_dof = *(const bool*)(value);
  // Only try to initialize if value changed from false to true
  if(skinning->auto_dof && !old_value)
  {
    skinning->initialize_auto_dof();
  }
}
void TW_CALL SkinningCallback::get_auto_dof(void *value, void *clientData)
{
  Skinning * skinning = scast(clientData);
  *(bool *)(value) = skinning->auto_dof;
}

void TW_CALL SkinningCallback::print_transformations(void *clientData)
{
  Skinning * skinning = scast(clientData);
  int m = skinning->T.cols()/4;
  int rows = skinning->T.rows();
  cout<<"T=["<<endl;
  for(int i = 0;i<m;i++)
  {
    cout<<endl<<skinning->T.block(0,i*4,rows,4)<<endl;
  }
  cout<<"];"<<endl;

  cout<<"d=["<<endl;
  for(int i = 0;i<m;i++)
  {
    cout<<endl<<skinning->T.block(0,i*4,3,3).determinant()<<endl;
  }
  cout<<"];"<<endl;
}

void TW_CALL SkinningCallback::clear_extra_weights(void *clientData)
{
  Skinning * skinning = scast(clientData);
  skinning->EW.resize(skinning->V.rows(),0);
  skinning->initialize_weights();
}

void TW_CALL SkinningCallback::set_draw_bones_according_to_T(const void *value, void *clientData)
{
  Skinning * skinning = scast(clientData);
  skinning->draw_bones_according_to_T = *(const bool*)(value);
  // Tell all bones the new value
  vector<Bone *> B = gather_bones(skinning->BR);
  for_each(B.begin(),B.end(),
    bind2nd(
      mem_fun(&Bone::set_draw_according_to_last_T),
      skinning->draw_bones_according_to_T));
}

void TW_CALL SkinningCallback::get_draw_bones_according_to_T(void *value, void *clientData)
{
  Skinning * skinning = scast(clientData);
  *(bool *)(value) = skinning->draw_bones_according_to_T;
}

void TW_CALL SkinningCallback::set_draw_connected_skeleton(
  const void *value, void *clientData)
{
  Skinning * skinning = scast(clientData);
  skinning->draw_connected_skeleton = *(const bool*)(value);
  // Tell all bones the new value
  vector<Bone *> B = gather_bones(skinning->BR);
  for_each(B.begin(),B.end(),
    bind2nd(
      mem_fun(&Bone::set_draw_connected_to_parent),
      skinning->draw_connected_skeleton));
}

void TW_CALL SkinningCallback::get_draw_connected_skeleton(
  void *value, void *clientData)
{
  Skinning * skinning = scast(clientData);
  *(bool *)(value) = skinning->draw_connected_skeleton;
}

void TW_CALL 
  SkinningCallback::set_average_children_tails_to_draw_non_weighted_roots(
  const void *value, void *clientData)
{
  Skinning * skinning = scast(clientData);
  skinning->average_children_tails_to_draw_non_weighted_roots = 
    *(const bool*)(value);
  // Tell all bones the new value
  vector<Bone *> B = gather_bones(skinning->BR);
  for_each(B.begin(),B.end(),
    bind2nd(
      mem_fun(&Bone::set_average_children_tails_to_draw_non_weighted_roots),
      skinning->average_children_tails_to_draw_non_weighted_roots));
}

void TW_CALL 
  SkinningCallback::get_average_children_tails_to_draw_non_weighted_roots(
  void *value, void *clientData)
{
  Skinning * skinning = scast(clientData);
  *(bool *)(value) = skinning->average_children_tails_to_draw_non_weighted_roots;
}

void TW_CALL 
  SkinningCallback::set_dial_in_each_T( const void *value, void *clientData)
{
  Skinning * skinning = scast(clientData);
  skinning->set_dial_in_each_T(*(const bool*)(value));
}

void TW_CALL 
  SkinningCallback::get_dial_in_each_T( void *value, void *clientData) 
{
  Skinning * skinning = scast(clientData);
  *(bool *)(value) = skinning->dial_in_each_T;
}


void TW_CALL 
  SkinningCallback::set_view_vector(const void *value, void *clientData)
{
  Skinning * skinning = scast(clientData);
  skinning->view_vector =  *(const ViewVector*)(value);
  // Tell all bones the new value
  vector<Bone *> B = gather_bones(skinning->BR);
  for_each(B.begin(),B.end(),
    bind2nd(mem_fun(&Bone::set_view_vector),skinning->view_vector));
}

void TW_CALL 
  SkinningCallback::get_view_vector(void *value, void *clientData)
{
  Skinning * skinning = scast(clientData);
  *(ViewVector *)(value) = skinning->view_vector;
}

void TW_CALL SkinningCallback::set_animate(const void * value, void * clientData)
{
  Skinning * skinning = scast(clientData);
  if(value && !skinning->animating)
  {
    skinning->start_animating();
  }else if(skinning->animating)
  {
    skinning->stop_animating();
  }
}

void TW_CALL SkinningCallback::get_animate(void * value, void * clientData)
{
  Skinning * skinning = scast(clientData);
  *(bool *)(value) = skinning->animating;
}

void TW_CALL SkinningCallback::get_animation_size(void * value, void * clientData)
{
  Skinning * skinning = scast(clientData);
  *(int *)(value) = skinning->animation.size();
}

void TW_CALL SkinningCallback::push_keyframe(void * clientData)
{
  Skinning * skinning = scast(clientData);
  if(!skinning->animating)
  {
    skinning->animation.push_back(
      KeyFrame<BoneBoneCopyMap>(
        BoneBoneCopyMap(skinning->BR,false),
        skinning->animation_interp_secs,
        skinning->transition_type));
  }
}

void TW_CALL SkinningCallback::pop_keyframe(void * clientData)
{
  Skinning * skinning = scast(clientData);
  if(!skinning->animating)
  {
    if(skinning->animation.size() > 0)
    {
      skinning->animation.pop_back();
    }
  }
}

void TW_CALL SkinningCallback::set_color_option(const void * value, void * clientData)
{
  Skinning * skinning = scast(clientData);
  skinning->color_option =  *(const ColorOption*)(value);
  if(skinning->color_option == COLOR_OPTION_ONE_SETS &&
    skinning->RC.rows()!= skinning->F.rows()*3)
  {
    verbose("RC.rows() %d != F.rows()*3 %d\n",
      skinning->RC.rows(),skinning->F.rows()*3);
    skinning->color_option = COLOR_OPTION_MATCH_DIFFUSE;
  }
  skinning->display_list_damage = true;
}

void TW_CALL SkinningCallback::get_color_option(void * value, void * clientData)
{
  Skinning * skinning = scast(clientData);
  *(ColorOption *)(value) = skinning->color_option;
}

void TW_CALL SkinningCallback::set_bypass_auto_dof(const void * value, void * clientData)
{
  Skinning * skinning = scast(clientData);
  skinning->bypass_auto_dof =  *(const bool*)(value);
  if(skinning->bypass_auto_dof)
  {
    // keep track of current T
    skinning->T_at_bypass = skinning->T;
    // Zero out extra transformations, by zeroing out all
    skinning->T *= 0;
    // Re-gather T of non extras 
    bool gather_success = 
      gather_transformations(skinning->BR,skinning->dial_in_each_T,skinning->T);
    if(!gather_success)
    {
      return;
    }
  }else
  {
    // Restore old transformations
    if(
      skinning->T.rows() == skinning->T_at_bypass.rows() && 
      skinning->T.cols() == skinning->T_at_bypass.cols())
    {
      verbose("Restored old transformations\n");
      skinning->T = skinning->T_at_bypass;
    }else
    {
      verbose("ERROR: Could not restored old transformations\n");
    }
  }
}

void TW_CALL SkinningCallback::get_bypass_auto_dof(void * value, void * clientData)
{
  Skinning * skinning = scast(clientData);
  *(bool *)(value) = skinning->bypass_auto_dof;
}

void TW_CALL SkinningCallback::set_corner_threshold(const void * value, void * clientData)
{
  Skinning * skinning = scast(clientData);
  skinning->corner_threshold =  *(const double*)(value);
  // Recompute corner normals
  per_corner_normals(
    skinning->V,
    skinning->F,
    skinning->corner_threshold,
    skinning->CN);
  skinning->display_list_damage = true;
}

void TW_CALL SkinningCallback::get_corner_threshold(void * value, void * clientData)
{
  Skinning * skinning = scast(clientData);
  *(double *)(value) = skinning->corner_threshold;
}

void TW_CALL SkinningCallback::set_use_corner_normals(const void * value, void * clientData)
{
  Skinning * skinning = scast(clientData);
  skinning->use_corner_normals =  *(const bool*)(value);
  skinning->display_list_damage = true;
}

void TW_CALL SkinningCallback::get_use_corner_normals(void * value, void * clientData)
{
  Skinning * skinning = scast(clientData);
  *(bool *)(value) = skinning->use_corner_normals;
}

void TW_CALL SkinningCallback::set_use_texture_mapping(const void * value, void * clientData)
{
  Skinning * skinning = scast(clientData);
  skinning->use_texture_mapping =  *(const bool*)(value);
  skinning->display_list_damage = true;
}

void TW_CALL SkinningCallback::get_use_texture_mapping(void * value, void * clientData)
{
  Skinning * skinning = scast(clientData);
  *(bool *)(value) = skinning->use_texture_mapping;
}

void TW_CALL SkinningCallback::clear_bone_roots(void *clientData)
{
  Skinning * skinning = scast(clientData);
  verbose("Clearing bone roots...\n");
  destroy_bone_roots(skinning->BR);
}
