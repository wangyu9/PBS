#ifndef IGL_HEADER_ONLY
#define IGL_HEADER_ONLY
#endif

#define VERBOSE
//#define EXTREME_VERBOSE
#include "Skinning.h"
#include "SkinningCallback.h"
using namespace SkinningCallback;

// Standard library
#include <cmath>
#include <algorithm>

// System library
// For making directories
#ifndef _WIN32
#  include <sys/stat.h>
#endif

// OpenGL related
#ifdef __APPLE__
#  include <OpenGL/gl.h>
// For functions like gluLookAt
#  include <OpenGL/glu.h>
// For glutSolidTeapot, maybe now this can be removed?
#  include <GLUT/glut.h>
#else //__APPLE__
#  include <GL/glu.h>
#  ifdef _WIN32
#    define NOMINMAX
#    include <Windows.h>
#    undef NOMINMAX
//#define FREEGLUT_STATIC
#    include <GL/freeglut.h>
#  else
#    include <GL/glut.h>
#  endif
#endif

// IGL library (all in 'igl' namespace)
// IGL functions
#include <igl/verbose.h>
#include <igl/get_seconds.h>
#include <igl/get_seconds_hires.h>
#include <igl/is_dir.h>
#include <igl/quat_to_mat.h>
#include <igl/trackball.h>
#include <igl/cocoa_key_to_anttweakbar_key.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/writeOBJ.h>
#include <igl/per_vertex_normals.h>
#include <igl/per_corner_normals.h>
#include <igl/quat_conjugate.h>
#include <igl/quat_mult.h>
#include <igl/rotate_by_quat.h>
#include <igl/axis_angle_to_quat.h>
#include <igl/print_gl_get.h>
#include <igl/pathinfo.h>
#include <igl/is_readable.h>
#include <igl/is_writable.h>
#include <igl/create_shader_program.h>
#include <igl/destroy_shader_program.h>
#include <igl/file_contents_as_string.h>
#include <igl/report_gl_error.h>
#include <igl/readDMAT.h>
#include <igl/writeDMAT.h>
#include <igl/sort.h>
#include <igl/transpose_blocks.h>
#include <igl/print_ijv.h>
#include <igl/slice.h>
#include <igl/slice_into.h>
#include <igl/writeMESH.h>
#include <igl/readMESH.h>
#include <igl/writeOFF.h>
#include <igl/texture_from_tga.h>
#include <igl/repmat.h>
// IGL constants
#include <igl/PI.h>
#include <igl/EPS.h>
#include <igl/ZERO.h>
#include <igl/ONE.h>
#include <igl/material_colors.h>
#include <igl/canonical_quaternions.h>
#include <igl/snap_to_canonical_view_quat.h>
// IGL classes
#include <igl/ReAntTweakBar.h>

// Use eigen for mesh data and weights for example as well as matrix
// computation
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Eigen/Sparse>

// Skinning project
#include "draw_mesh.h"
#include "draw_floor.h"
#include "eigen_glUniformMatrixXfv.h"
#include "build_anttweakbar_modifier.h"
#include "is_planar.h"
#include "optimize_index_buffer.h"
#include "append_to_filename_before_extension.h"
// Bones
#include "Bone.h"
#include "gather_transformations.h"
#include "read_BF.h"
#include "write_BF.h"
#include "gather_bones.h"
#include "gather_rest_positions.h"
#include "gather_displacements.h"
#include "gather_free.h"
#include "gather_fixed.h"
#include "intersection.h"
#include "gather_positional_constraints_system.h"
#include "gather_positional_constraints_rhs.h"
#include "distribute_transformations.h"
#include "lerp.h"
#include "bone_roots_match.h"
#include "copy_bone_roots_data.h"
#include "load_tgf_and_dmat_pair_as_bone_roots_and_weights.h"
#include "destroy_bone_roots.h"
// Weights
#include "sort_weights.h"
// Auto dof
#include "lbs_matrix.h"
#include "partition.h"
#include "columnize.h"
#include "uncolumnize.h"
#include "pseudocolors_from_weights.h"
#include "region_colors_from_weights.h"
#include "project_extra_compact.h"
#include "iso_extra.h"

//#define SKINNING_PROFILING

#define OPTIMIZE_INDEX_BUFFER

using namespace std;
// All singleton header files will be using igl namespace
using namespace igl;
using namespace Eigen;

/////////////////////////////////////////////////////////////////////////////
// Skinning Shader related constants
/////////////////////////////////////////////////////////////////////////////
// The variable name in the shader associated with the handle transformations
#define HANDLE_TRANSFORMATION_NAME "handle_transformation[0]"
#define NUM_HANDLES_NAME "num_handles"
#define SELECTED_WEIGHT_NAME "selected_weight"
// The maximum number of weights per mesh vertex allowed by the LBS shader, and
// maximum number of handles in shader
#define MAX_NUM_HANDLES 100
#define MAX_NUM_WEIGHTS_PER_VERTEX 16

// The variable name in the shader associated with the first four skinning
// weights (0 through 3)
#define WEIGHTS0_ATTRIBUTE_NAME "weights0"
// The variable name in the shader associated with the next four skinning
// weights (4 through 7)
#define WEIGHTS4_ATTRIBUTE_NAME "weights4"
#define WEIGHTS8_ATTRIBUTE_NAME "weights8"
#define WEIGHTS12_ATTRIBUTE_NAME "weights12"
#define WEIGHTS0_ATTRIBUTE_LOCATION 8
// The location of WEIGHTS4 etc will be WEIGHTS0_ATTRIBUTE_LOCATION + 1, etc

// The variable name in the shader associated with the first for skinning
// weight indices
#define WEIGHT_INDICES0_ATTRIBUTE_NAME "weight_indices0"
// The variable name in the shader associated with the second for skinning
// weight indices
#define WEIGHT_INDICES4_ATTRIBUTE_NAME "weight_indices4"
#define WEIGHT_INDICES8_ATTRIBUTE_NAME "weight_indices8"
#define WEIGHT_INDICES12_ATTRIBUTE_NAME "weight_indices12"
#define WEIGHT_INDICES0_ATTRIBUTE_LOCATION 12
// The location of WEIGHT_INDICES4 etc will be
// WEIGHT_INDICES0_ATTRIBUTE_LOCATION + 1, etc

#define MAT4
// Allow exactly one of MAT4, MAT4x3 or MAT3x4 to be defined, defaulting to
// MAT4
// Note: This definition should match the definition in the lbs shader
#if defined( MAT4x3 )
#  ifdef MAT3x4
#    error
#  endif
#  ifdef MAT4
#    error
#  endif
#elif defined( MAT3x4 )
#  ifdef MAT4x3
#    error
#  endif
#  ifdef MAT4
#    error
#  endif
#elif defined( MAT4 )
#  ifdef MAT4x3
#    error
#  endif
#  ifdef MAT3x4
#    error
#  endif
#else
#  define MAT4
#endif

// number of rows in handle transformations in skinning shader
// (4 if using mat4, 3 if using mat3x4 or mat4x3)
#ifdef HANDLE_TRANSFORMATION_ROWS
#  error(HANDLE_TRANSFORMATION_ROWS IS CHOSEN BASED ON MAT* MACRO)
#endif
#if defined( MAT4x3)
#  define HANDLE_TRANSFORMATION_ROWS  3
#elif defined( MAT3x4)
#  define HANDLE_TRANSFORMATION_ROWS  3
#else 
#  define HANDLE_TRANSFORMATION_ROWS  4
#endif

#ifdef __APPLE__
  #define TEMP_DIR "/var/tmp/.skinning_temp"
  #define SHADER_DIR "/Users/ajx/Documents/AutoDOF/Code/skinning/skinning/GLSL"
#else
  #define TEMP_DIR "C:/prg/tmp"
  #define SHADER_DIR "C:/prg/GLSL"
#endif

Skinning::Skinning():
  texture_id(0),
  use_cpu(false),
  // Default values for const fields
  min_zoom(1e-5),
  max_zoom(20),
  min_angle(0),
  // Default values for fields that may not get initialized
  color_option(COLOR_OPTION_MATCH_DIFFUSE),
  draw_bones(true),
  draw_bones_according_to_T(false),
  draw_connected_skeleton(false),
  average_children_tails_to_draw_non_weighted_roots(false),
  dial_in_each_T(false),
  animating(false),
  phony_animation(false),
  animation_interp_secs(1.0),
  transition_type(EASE_TRANSITION),
  auto_dof(false),
  bypass_auto_dof(false),
  max_iters(1),
  tol(0),
  sort_weights_epsilon_power(-1),
  ew_type(EXTRAWEIGHT_ISO),
  ease(EASE_CUBIC_BSPLINE),
  push(2),
  num_abrupt_weight_space(0),
  num_groups(0),
  num_extra_weights(0),
  display_list_id(0),
  // Default values for fields that must be set to a certain value for the
  // program to work correctly
  virgin_display(true),
  shaders_loaded(false),
  shader_mode(NO_SHADER),
  selected_weight(0)
{
  // Print out any "weird" settings
#ifdef EIGEN_DEFAULT_TO_ROW_MAJOR
  verbose("EIGEN_DEFAULT_TO_ROW_MAJOR\n");
#endif
#if defined(MAT4x3)
  verbose("MAT4x3\n");
#elif defined(MAT3x4)
  verbose("MAT3x4\n");
#endif

  anim_timer = 0.0f;
  initialize_display();
  initialize_anttweakbar();
  initialize_mouse_and_keyboard();
  initialize_rotation_animation();
  // Try to load last session
  load(TEMP_DIR);
}

Skinning::~Skinning()
{
  // Save current session to temp folder
  save(TEMP_DIR);
  delete rebar;
  deinitialize_shaders();
}

// Default names
#define REBAR_NAME "rebar"
#define MESH_NAME "mesh"
#define WEIGHTS_NAME "weights"
#define BONE_ROOTS_NAME "bone_roots"

bool Skinning::save(const string folder_name)
{
  // Make folder (if it doesn't already exist)
  bool folder_exists = is_dir(folder_name.c_str());
  if(!folder_exists)
  {
    verbose("Creating new directory %s...",folder_name.c_str());
#ifdef _WIN32
    int folder_created = false;
#else
    int folder_created = mkdir(folder_name.c_str(),0755);
#endif
    verbose("%s\n",(folder_created==0?"succeeded":"failed"));
    if(folder_created!=0)
    {
      fprintf(
        stderr,
        "^Skinning::save: IOERROR (%d): could not create folder: %s\n",
        errno,
        folder_name.c_str());
      return false;
    }
  }else
  {
    verbose("Directory %s already exists\n",folder_name.c_str());
  }

  // Try to save everything to this folder with a default name
  bool everything_saved = true;
  // Try to save ReAntTweakBar
  verbose("Saving ReantTweakBar...");
  bool rebar_saved = 
    rebar->save((folder_name +"/" + REBAR_NAME + ".rbr").c_str());
  verbose("%s\n",(rebar_saved?"succeeded":"failed"));
  everything_saved &= rebar_saved;

  // Try to save mesh
  bool mesh_saved = false;
  verbose("Saving Mesh...");
  string obj_file_name(folder_name+"/"+MESH_NAME+".obj");
  string mesh_file_name(folder_name+"/"+MESH_NAME+".mesh");
  if(Tets.size() == 0)
  {
    remove(mesh_file_name.c_str());
    mesh_saved = writeOBJ(obj_file_name.c_str(),V,F);
  }else
  {
    remove(obj_file_name.c_str());
    mesh_saved = writeMESH(folder_name+"/"+MESH_NAME+".mesh",V,Tets,F);
  }
  verbose("%s\n",(mesh_saved?"succeeded":"failed"));
  everything_saved &= mesh_saved;

  // Try to save weights
  verbose("Saving weights...");
  bool weights_saved = 
    writeDMAT((folder_name+"/"+WEIGHTS_NAME+".dmat").c_str(),OW);
  verbose("%s\n",(weights_saved?"succeeded":"failed"));
  everything_saved &= weights_saved;

  // Try to save bones
  if(animation.size() > 0)
  {
    bool bone_roots_saved = 
      save_bone_roots_animation((folder_name+"/"+BONE_ROOTS_NAME+".bf").c_str());
    everything_saved &= bone_roots_saved;
  }else
  {
    bool bone_roots_saved = 
      save_bone_roots((folder_name+"/"+BONE_ROOTS_NAME+".bf").c_str());
    everything_saved &= bone_roots_saved;
  }

  return everything_saved;
}

bool Skinning::load(const string folder_name)
{
  bool everything_loaded = true;
  // Try to load ReAntTweakBar
  bool rebar_loaded = rebar->load((folder_name+"/"+REBAR_NAME+".rbr").c_str());
  everything_loaded &= rebar_loaded;
  // Try to load OBJ mesh
  bool mesh_loaded = 
    load_mesh_from_file(folder_name+"/"+MESH_NAME+".obj");
  if(!mesh_loaded)
  {
    // try to load OFF mesh
    mesh_loaded = load_mesh_from_file(folder_name+"/"+MESH_NAME+".off");
    if(!mesh_loaded)
    {
      // try to load MESH mesh
      mesh_loaded = load_mesh_from_file(folder_name+"/"+MESH_NAME+".mesh");
    }
  }
  everything_loaded &= mesh_loaded;
  //Try to load DMAT weights matrix
  bool weights_loaded = load_weights(folder_name+"/"+WEIGHTS_NAME+".dmat");
  everything_loaded &= weights_loaded;

  //Try to load BF bone roots file
  clear_bone_roots(this);
  verbose("Loading bone roots... ");
  bool bone_roots_loaded =
    load_bone_roots_animation(folder_name+"/"+BONE_ROOTS_NAME+".bf");
  verbose("%s\n",(bone_roots_loaded?"succeeded":"failed"));
  if(animation.size() == 1)
  {
    animation.clear();
  }
  everything_loaded &= bone_roots_loaded;

  return everything_loaded;
}

void Skinning::initialize_display()
{
  damage = true;
  display_list_damage = true;
  use_display_list = true;
  // Slightly off straight on light
  light_direction[0] = -0.1;
  light_direction[1] = -0.1;
  light_direction[2] = -1.0;

  light2_direction[0] = 1.0;
  light2_direction[1] = 0.0;
  light2_direction[2] = 0.0;
  // 1.0 --> normal exposure
  light_multiplier = 1.0;
  // Silver
  copy(GOLD_AMBIENT,GOLD_AMBIENT+4,mat_ambient);
  copy(GOLD_DIFFUSE,GOLD_DIFFUSE+4,mat_diffuse);
  copy(GOLD_SPECULAR,GOLD_SPECULAR+4,mat_specular);
  copy(DENIS_PURPLE_DIFFUSE,DENIS_PURPLE_DIFFUSE+4,region_color);
  // not-so-shiny
  mat_shininess = 128;
  show_faces = true;
  show_lines = false;
  // canonical (X,Y) view
  copy(XY_PLANE_QUAT_F,XY_PLANE_QUAT_F+4,rotation);
  // Always scale and shift on load
  scale_and_shift_on_load = true;
  // No shift
  shift[0] = 0; shift[1] = 0; shift[2] = 0;
  // No scale
  scale = 1;
  // No snapping
  snap_threshold = 0.0;
  // Identity zoom
  zoom = 1;
  // allow zooming
  zoom_locked = false;
  // no panning
  pan[0] = 0; pan[1] = 0; pan[2] = 0;
  // allow panning
  pan_locked = false;
  // Good default settings
  angle = 45;
  // allow angle/perspective changes
  angle_locked = false;
  // Always clear the screen
  always_clear = true;
  // time dependent color clearing
  time_clear_color = false;
  // Dark blue background color
  background[0] = 0.3;
  background[1] = 0.3;
  background[2] = 0.5;
  background[3] = 0.0;
  // use corner normals
  use_corner_normals = false;
  corner_threshold = 20;
}

void Skinning::initialize_anttweakbar()
{
  draw_anttweakbar = true;
  // Initialize anttweakbar library
  TwInit(TW_OPENGL, NULL);
  // make new reanttweakbar
  rebar = new ReTwBar();
#ifdef _RELEASE
  rebar->TwNewBar("bar");
  TwDefine("bar label='Release' size='200 550' text=light alpha='200' color='10 10 10'");
  rebar->TwGetParam(NULL, "color", TW_PARAM_INT32, 3, old_bar_color);
#else
  rebar->TwNewBar("bar");
  TwDefine("bar label='Debug' size='200 550' text=light alpha='200' color='250 0 0'");
  rebar->TwGetParam(NULL, "color", TW_PARAM_INT32, 3, old_bar_color);
#endif
  TwDefine(" GLOBAL help='This is a prototyping tool for skinning-based "
    " deformation.' ");

  // Start adding items
  add_info_anttweakbar_group();
  add_3d_view_anttweakbar_group();
  add_transformation_editor_group();
  add_auto_dof_group();
  add_animation_group();
}

void Skinning::add_info_anttweakbar_group()
{
  rebar->TwAddVarRW("Draw bar",TW_TYPE_BOOLCPP,&draw_anttweakbar,
    " visible=true key=B help='Enable/Disable all AntTweakBar draw calls'");
  rebar->TwAddVarCB( "NumVertices", TW_TYPE_INT32, 
      no_op,
      get_num_vertices,
      this,
      " group='Info' label='Vertices' help='Lists number of"
      " mesh vertices of current mesh' readonly=true");
  rebar->TwAddVarCB( "NumFaces", TW_TYPE_INT32,
      no_op,
      get_num_faces,
      this,
      " group='Info' label='Faces' help='Lists number of"
      " mesh faces of current mesh' readonly=true");
  rebar->TwAddVarRO("fps",TW_TYPE_DOUBLE, &fps,
    " group='Info' label='Frames per second'"
    " help='Displays current number of frames drawn per second.'");
  rebar->TwAddVarRO("mspf",TW_TYPE_DOUBLE, &mspf,
    " group='Info' label='ms per frame'"
    " help='Displays current number per frame drawn.'");
}

void Skinning::add_transformation_editor_group()
{
  rebar->TwAddVarCB("sb_wi",TW_TYPE_INT32,
      no_op,
      get_selected_bone_wi,
      this,
    " label='wi' group='Transformation editor' readonly=true"
    " help='Weight index of selected bone'",false);
  rebar->TwAddVarCB("sb_rotation",TW_TYPE_QUAT4D,
      set_selected_bone_rotation,
      get_selected_bone_rotation,
      this,
    " label='rotation' group='Transformation editor' "
    " help='Rotation of selected bone' open",false);
  rebar->TwAddButton("sb_snap_rotation", 
    snap_selected_bone_rotation_to_canonical_view_quat, this,
    " group='Transformation editor'"
    " label='Snap rotation' key='CTRL+z'"
    " help='Snap selected bones rotation to nearest canonical quat.'");
  rebar->TwAddVarCB("sb_translation_x",TW_TYPE_DOUBLE,
      set_selected_bone_translation_x,
      get_selected_bone_translation_x,
      this,
    " label='X' step=0.1 group='Transformation editor' "
    " help='X coordinate of translation selected bone' ",false);
  rebar->TwAddVarCB("sb_translation_y",TW_TYPE_DOUBLE,
      set_selected_bone_translation_y,
      get_selected_bone_translation_y,
      this,
    " label='Y' step=0.1 group='Transformation editor' "
    " help='Y coordinate of translation selected bone' ",false);
  rebar->TwAddVarCB("sb_translation_z",TW_TYPE_DOUBLE,
      set_selected_bone_translation_z,
      get_selected_bone_translation_z,
      this,
    " label='Z' step=0.1 group='Transformation editor' "
    " help='Z coordinate of translation selected bone' ",false);
  rebar->TwAddVarCB("dial_in_each_T",TW_TYPE_BOOLCPP,
      SkinningCallback::set_dial_in_each_T,
      SkinningCallback::get_dial_in_each_T,
      this,
    " label='Dial in each T' group='Transformation editor' "
    " help='Dail in each transformation T, ignoring stored rotations and"
    " translations' ");
  rebar->TwAddVarCB("sb_last_T",TW_TYPE_STDSTRING,
      set_selected_bone_last_T,
      get_selected_bone_last_T,
      this,
      " label='last_T' group='Transformation editor' "
      " help='String representation of transformation of selected bone' ",
      false);
  // To be sure readonly gets set correctly
  set_dial_in_each_T(dial_in_each_T);

  rebar->TwAddButton("sb_reset",
    reset_selected_bone_transformation,
    this,
    " label='Reset selected' key='R' group='Transformation editor' "
    " help='Reset transformation of selected bone' ");
  rebar->TwAddButton("reset extra",
    reset_extra_transformations,
    this,
    " label='Reset just extra' key='CTRL+r' group='Transformation editor' "
    " help='Reset transformation of extra weights' ");
  rebar->TwAddButton("reset",
    reset_all_bone_transformations,
    this,
    " label='Reset all' key='r' group='Transformation editor' "
    " help='Reset transformation of all bones' ");
  TwEnumVal DegreeOfFreedomTypeEV[NUM_DEGREE_OF_FREEDOM_TYPE] = 
  {
    {NORMAL_DOF,"Normal"},
    {FREE_DOF, "Free"},
    {FIXED_DOF, "Fixed"}
  };
  TwType DegreeOfFreedomTypeTW = 
    ReTwDefineEnum(
      "DegreeOfFreedomType", 
      DegreeOfFreedomTypeEV, 
      NUM_DEGREE_OF_FREEDOM_TYPE);
  rebar->TwAddVarCB("sb_dof_type",DegreeOfFreedomTypeTW,
      set_selected_bone_dof_type,
      get_selected_bone_dof_type,
      this,
    " label='DOF type' group='Transformation editor' "
    " help='Sets whether bones associated weight function should be treated"
    " as a normal, free or fixed handle' key=f ",false);
  rebar->TwAddButton("select_all",select_all,this,
    " label='select all' group='Transformation editor' "
    " help='Selects all bone tips' key=CTRL+a");
  TwEnumVal ViewVectorEV[NUM_VIEW_VECTOR] = 
  {
    {VIEW_VECTOR,"View"},
    {UP_VECTOR, "Up"},
    {RIGHT_VECTOR, "Right"}
  };
  TwType ViewVectorTW = 
    ReTwDefineEnum("ViewVector", ViewVectorEV, NUM_VIEW_VECTOR);
  rebar->TwAddVarCB("view_vector",ViewVectorTW,
      set_view_vector,get_view_vector,this,
    " label='View vector' group='Transformation editor' key=V"
    " help='Vector about which to rotate when using right-click drag'");
  rebar->TwAddButton("print_transformations",print_transformations,this,
    " group='Transformation editor'"
    " label='Print T' key=T help='print current transformations held in T'");
  rebar->TwAddButton("clear_bone_roots",clear_bone_roots,this,
    " label='Clear bone roots' group='Transformation editor' "
    " help='Clear current skeleton'");
}


void Skinning::add_3d_view_anttweakbar_group()
{
  rebar->TwAddButton("Clear", SkinningCallback::clear, this,
    " group='3D View'"
    " label='Clear' help='Force clear the screen'");
  rebar->TwAddVarRW("AlwaysClear", TW_TYPE_BOOLCPP, &always_clear,
    " group='3D View'"
    " label='Clear' help='clear the screen on display()'");
  rebar->TwAddVarRW("time_clear_color", TW_TYPE_BOOLCPP, &time_clear_color,
    " group='3D View'"
    " label='Time color clear' help='clear the screen on display() with"
    " time varying color'");
  // 3D View Group
  rebar->TwAddVarRW( "Background", TW_TYPE_COLOR3F, &background,
      " group='3D View' colormode=hls ");
  rebar->TwAddVarRW( "Ambient", TW_TYPE_COLOR3F, &mat_ambient, 
      " group='3D View' ");
  rebar->TwAddVarRW( "Diffuse", TW_TYPE_COLOR3F, &mat_diffuse, 
      " group='3D View' ");
  rebar->TwAddVarRW( "Specular", TW_TYPE_COLOR3F, &mat_specular, 
      " group='3D View' ");
  rebar->TwAddVarRW("Shininess",TW_TYPE_FLOAT,&mat_shininess," group='3D View'"
      " min=0 max=128");
  rebar->TwAddVarRW("show_faces",TW_TYPE_BOOLCPP,&show_faces,
    " group='3D View' key=t label='Faces' help='Show filled faces of mesh'");
  rebar->TwAddVarRW("show_lines",TW_TYPE_BOOLCPP,&show_lines,
    " group='3D View' key=l label='lines' help='Show lines of mesh wireframe'");
  rebar->TwAddVarCB("use_corner_normals",TW_TYPE_BOOLCPP,
      set_use_corner_normals, get_use_corner_normals, this,
    " group='3D View' key=n label='sharp edges' "
    " help='Use per corner normals instaed of per vertex'");
  rebar->TwAddVarCB("corner_threshold",TW_TYPE_DOUBLE,
      set_corner_threshold, get_corner_threshold, this,
    " group='3D View' min=0 max=180 label='corner max angle' "
    " help='Angle threshold in degrees used to declare edges sharp'");
  rebar->TwAddVarCB("use_texture_mapping",TW_TYPE_BOOLCPP,
      set_use_texture_mapping, get_use_texture_mapping, this,
    " group='3D View' key=u label='texture map' "
    " help='Texture map using X Y as U V'");
  rebar->TwAddVarRW("reverse_depth_offsets",TW_TYPE_BOOLCPP,
      &reverse_depth_offsets,
    " group='3D View' label='reverse depth offsets' "
    " help='Reverse depth offsets'");
  rebar->TwAddVarRW( "Multiplier", TW_TYPE_FLOAT, &light_multiplier, 
    " group='3D View' label='Light booster' min=0.1 max=4 step=0.02 "
    " help='Increase/decrease the light power.' ");
  rebar->TwAddVarRW( "LightDir", TW_TYPE_DIR3F, &light_direction, 
    " group='3D View' label='Light direction' open"
    " help='Change the light direction.' ");
  rebar->TwAddVarRW( "Light2Dir", TW_TYPE_DIR3F, &light2_direction, 
    " group='3D View' label='Light2 direction' open"
    " help='Change the light2 direction.' ");
  rebar->TwAddVarRW("Rot",TW_TYPE_QUAT4F,&rotation,
    " group='3D View' label='Rotation' open"
    " help='Rotate the object (0 0 0 1 = identity)'");
  rebar->TwAddVarCB("SpinUp",TW_TYPE_BOOLCPP,
    set_spinning_about_up_axis,
    get_spinning_about_up_axis,
    this,
    " group='3D View' label='Spin around up axis' "
    " help='Spin around the current up axis' key=SPACE");
  rebar->TwAddVarRW( "SpinPeriod", TW_TYPE_DOUBLE, &spin_period, 
    " group='3D View' label='Spin period' min=1 max=100 step=1 "
    " help='Time it takes to complete one spin (in seconds).' ");
  rebar->TwAddButton("ViewXYPlane", view_xy_plane, this,
    " group='3D View'"
    " label='View XY Plane' key=z help='View the XY plane.'");
  rebar->TwAddButton("ViewXZPlane", view_xz_plane, this,
    " group='3D View'"
    " label='View XZ Plane' key=y help='View the XZ plane.'");
  rebar->TwAddButton("ViewYZPlane", view_yz_plane, this,
    " group='3D View'"
    " label='View YZ Plane' key=x help='View the XY plane.'");
  rebar->TwAddButton("SnapRot", 
    snap_rotation_to_canonical_view_quat, this,
    " group='3D View'"
    " label='Snap rotation' key=Z"
    " help='Snap view to nearest canonical view.'");
  rebar->TwAddVarRW("TrackBallSpeedFactor",TW_TYPE_DOUBLE,
      &trackball_speed_factor,
      " group='3D View' min=0.05 max=10 help='Controls how fast the trackball"
      " feels' label='Trackball Speed'");
  rebar->TwAddVarRW("SnapThreshold",TW_TYPE_DOUBLE,
      &snap_threshold,
      " group='3D View' min=0 max=1 step=0.1"
      " help='Threshold for snapping to nearest canonical view while using"
      " track ball' label='Snap threshold'");
  rebar->TwAddVarRW( "ZoomLock", TW_TYPE_BOOLCPP, &zoom_locked, 
    " group='3D View' label='Zoom lock' "
    " help='Enable zooming.' ");
  rebar->TwAddVarCB("Zoom",TW_TYPE_DOUBLE,
    set_zoom,get_zoom,this,
    " group='3D View' step=0.01"
    " help='Scale the object (1=original size)'");
  rebar->TwSetParam("Zoom", "min", TW_PARAM_DOUBLE, 1, &min_zoom);
  rebar->TwSetParam("Zoom", "max", TW_PARAM_DOUBLE, 1, &max_zoom);
  rebar->TwAddVarRW("pan_x",TW_TYPE_DOUBLE,&pan[0],
    " group='3D View' label='pan x' step=0.01 min=-3 max=3 "
    " help='Pan in x direction'");
  rebar->TwAddVarRW("pan_y",TW_TYPE_DOUBLE,&pan[1],
    " group='3D View' label='pan y' step=0.01 min=-3 max=3 "
    " help='Pan in y direction'");
  rebar->TwAddVarRW("pan_z",TW_TYPE_DOUBLE,&pan[2],
    " group='3D View' label='pan z' step=0.01 min=-3 max=3 "
    " help='Pan in z direction'");
  rebar->TwAddVarRW( "AngleLock", TW_TYPE_BOOLCPP, &angle_locked, 
    " group='3D View' label='Angle lock' "
    " help='Enable angle/perspective changes.' ");
  rebar->TwAddVarCB("angle",TW_TYPE_DOUBLE,
    set_angle,get_angle,this,
    " group='3D View' max='135' step='1' help='change from"
    " orthographic (~0) to 135 degrees perspective (135)'");
  rebar->TwSetParam("angle", "min", TW_PARAM_DOUBLE, 1, &min_angle);
  rebar->TwAddVarRW("use_display_list",TW_TYPE_BOOLCPP,&use_display_list,
    " group='3D View' key='d' label='Use display list'"
    " help='Use a display list around OpenGL calls'");
  rebar->TwAddVarRW("use_cpu",TW_TYPE_BOOLCPP,&use_cpu,
    " group='3D View' key='C' label='CPU LBS'"
    " help='Use cpu to deform and draw with recomputed normals'",false);
  TwEnumVal ShaderModeEV[NUM_SHADER_MODE] = 
  {
    {NO_SHADER,"OFF"},
    {DIRECTIONAL_PER_PIXEL, "Phong"},
    {DIRECTIONAL_PER_PIXEL_COLOR, "Phong per-vertex color"},
    {WEIGHTCOLORS, "WEIGHTCOLORS"},
    {SCALAR, "Select weight"},
    {LBS, "LBS"}
  };
  TwType ShaderModeTW = ReTwDefineEnum("ShaderMode", ShaderModeEV, NUM_SHADER_MODE);
  rebar->TwAddVarRW("Shader",ShaderModeTW, &shader_mode,
      " group='3D View' help='Select shader in use' keyIncr='>' keyDecr='<'");
  rebar->TwAddVarRW("selected_weight",TW_TYPE_INT32,&selected_weight,
    "label='selected weight' step=1 min=0 max=100 keyIncr=. keyDecr=, "
    " group='3D View' "
    " help='selected weight shown by Select weight shader'");
  rebar->TwAddVarRW("draw_bones",TW_TYPE_BOOLCPP,&draw_bones,
    "label='draw bones' key=s "
    " group='3D View' "
    " help='Draw skeleton vertices and edges'");
  TwEnumVal ColorOptionEV[NUM_COLOROPTION] = 
  {
    {COLOR_OPTION_MATCH_DIFFUSE, "Match diffuse"},
    {COLOR_OPTION_ORIGINAL_WEIGHTS, "Weights"},
    {COLOR_OPTION_ONE_SETS, "Regions"}
  };
  TwType ColorOptionTW = ReTwDefineEnum("ColorOption", ColorOptionEV, NUM_COLOROPTION);
  rebar->TwAddVarCB("color_option",ColorOptionTW,
    set_color_option,get_color_option,this,
    " label='color option' key=c "
    " group='3D View' "
    " help='Use colors defining handle regions when displaying mesh, etc'");
  rebar->TwAddVarCB("draw_bones_according_to_T",TW_TYPE_BOOLCPP,
    set_draw_bones_according_to_T,get_draw_bones_according_to_T,this,
    "label='draw according to T' "
    " group='3D View' "
    " help='Ignore transformations stored at bones when drawing, use whatever"
    " is in T instead'");
  rebar->TwAddVarCB("draw_connected_skeleton",TW_TYPE_BOOLCPP,
    set_draw_connected_skeleton,get_draw_connected_skeleton,this,
    "label='draw connected skeleton' "
    " group='3D View' "
    " help='Ignore transformations of bones when drawing bone segments, "
    " just connect children to parents'");
  rebar->TwAddVarCB("average_children_tails_to_draw_non_weighted_roots",
    TW_TYPE_BOOLCPP,
    set_average_children_tails_to_draw_non_weighted_roots,
    get_average_children_tails_to_draw_non_weighted_roots,
    this,
    "label='average tails for non weight roots' "
    " group='3D View' "
    " help='Draw non-weighted roots as average of childrens tails'");
  rebar->TwAddVarRW("floor depth",TW_TYPE_FLOAT,&floor_depth,
    " group='3D View' label='floor depth' step=0.01 min=-3 max=3 "
    " help='Y coordinate of floor plane'");
  rebar->TwAddVarRW("lead_armadillo_distance",TW_TYPE_FLOAT,&lead_armadillo_distance,
    " group='3D View' label='lead_armadillo_distance' step=0.01 min=-3 max=3 "
    " help='lead_armadillo_distance'");
  play_animation = true;
  rebar->TwAddVarRW("play_animation",TW_TYPE_BOOLCPP,&play_animation,
    " group='3D View' label='play animation'");

  // Collapse group
  rebar->TwSetParam( "3D View", "opened", TW_PARAM_INT32, 1, &INT_ZERO);

  // Advanced view tweaking
  rebar->TwAddVarRW("scale_and_shift_on_load",TW_TYPE_BOOLCPP,&scale_and_shift_on_load,
    " group='Advanced 3D View' help='Scale and shift the object on load'",false);
  rebar->TwAddVarRW("scale",TW_TYPE_DOUBLE,&scale,
    " group='Advanced 3D View' min=0 help='Scale the object, set at mesh load '",false);
  rebar->TwAddVarRW("shift_x",TW_TYPE_DOUBLE,&shift[0],
    " group='Advanced 3D View' help='Shift object, set at mesh load'",false);
  rebar->TwAddVarRW("shift_y",TW_TYPE_DOUBLE,&shift[1],
    " group='Advanced 3D View' help='Shift object, set at mesh load'",false);
  rebar->TwAddVarRW("shift_z",TW_TYPE_DOUBLE,&shift[2],
    " group='Advanced 3D View' help='Shift object, set at mesh load'",false);
}

void Skinning::add_auto_dof_group()
{
  rebar->TwAddVarCB("auto_dof",TW_TYPE_BOOLCPP,
    set_auto_dof,get_auto_dof,this,
    " group='AutoDOF' key='D' label='Auto DOF'"
    " help='Compute missing degrees of freedom automatically'");
  rebar->TwAddVarCB("bypass_auto_dof",
    TW_TYPE_BOOLCPP,
    set_bypass_auto_dof, get_bypass_auto_dof,this,
    " group='AutoDOF' key='b' label='Bypass auto dof' "
    " help='Bypass auto dof when computing T, sets extras temporarily to 0'",
    false);
  TwEnumVal ArapEnergyEV[NUM_ARAPENERGY] = 
  {
    {ARAP_SPOKES,"Spokes"},
    {ARAP_SPOKES_AND_RIMS, "Spokes and rims"},
    {ARAP_ELEMENTS, "Elements"}
  };
  TwType ArapEnergyTW = ReTwDefineEnum("ArapEnergy", ArapEnergyEV, NUM_ARAPENERGY);
  rebar->TwAddVarRW("auto_dof_energy",ArapEnergyTW, &arap_dof.energy,
    " group='AutoDOF' key='e' label='Energy' help='Arap energy to use'");
  rebar->TwAddVarRW("num_groups",TW_TYPE_INT32,&num_groups,
    " group='AutoDOF' min=1 step=1 label='Number of groups'"
    " help='number of rotation cluster groups in arap auto dof optimization'");
  TwEnumVal ExtraWeightEV[NUM_EXTRAWEIGHT] = 
  {
    {EXTRAWEIGHT_PROJECT_COMPACT, "Project compact"},
    {EXTRAWEIGHT_ISO,"iso"}
  };
  TwType ExtraWeightTW = ReTwDefineEnum("ExtraWeight", ExtraWeightEV, NUM_EXTRAWEIGHT);
  rebar->TwAddVarRW("Extra weights type",ExtraWeightTW, &ew_type,
      " group='AutoDOF' help='Select Extra weight type to use' key=E");
  TwEnumVal EaseEV[NUM_EASE] = 
  {
    {EASE_CUBIC, "Cubic"},
    {EASE_QUINTIC,"Quintic"},
    {EASE_CUBIC_BSPLINE, "Cubic B-spline"}
  };
  TwType EaseTW = ReTwDefineEnum("Ease", EaseEV, NUM_EASE);
  rebar->TwAddVarRW("Ease filter",EaseTW, &ease,
      " group='AutoDOF' help='Select ease filter to use for certain extra"
      " weight techniques (e.g. iso_extra)' ");
  rebar->TwAddVarRW("push",TW_TYPE_DOUBLE,&push,
    " group='AutoDOF' min=0.5 max=10 step=0.5 label='Push'"
    " help='Push factor to use for certain extra weight methods'");
  rebar->TwAddVarRW("num_abrupt_weight_space",TW_TYPE_INT32,
      &num_abrupt_weight_space,
    " group='AutoDOF' min=0 max=10 label='Abrupt W'"
    " help='Prefilter weight space by an abrupt filter, this many times'");
  rebar->TwAddVarRW("num_extra_weights",TW_TYPE_INT32,&num_extra_weights,
    " group='AutoDOF' min=0 step=1 label='Number of extra weights'"
    " help='number of extra weights in arap auto dof optimization'");
  rebar->TwAddVarRW("sort_weights_epsilon_power",TW_TYPE_INT32,
      &sort_weights_epsilon_power,
    " group='AutoDOF' min=-16 max=-1 step=1 label='Sort epsilon power'"
    " help='Power of epsilon used for sorting weights'");
  rebar->TwAddVarRW("max_iters",TW_TYPE_INT32,&max_iters,
    " group='AutoDOF' min=1 max=10000 step=1 label='Max iterations'"
    " help='maximum number of iterations per frame'");
  rebar->TwAddVarRW("tol",TW_TYPE_DOUBLE,&tol,
    " group='AutoDOF' min=0 max=1 step=0.1 label='Tolerance'"
    " help='Stopping tolerance parameter: 0 strong, 1 weak'");
  rebar->TwAddButton("clear_EW",clear_extra_weights,this,
    " group='AutoDOF' label='Clear extra weights' help='Reset extra weights"
    " to  emtpy matrix'");
  rebar->TwAddVarRW("print_timings",TW_TYPE_BOOLCPP,
    &arap_dof.print_timings,
    " group='AutoDOF' label='Print timings per update'"
    " help='Print timings at each arap dof update iteration'");
  // Collapse group
  rebar->TwSetParam( "AutoDOF", "opened", TW_PARAM_INT32, 1, &INT_ZERO);
}

void Skinning::add_animation_group()
{
  rebar->TwAddVarCB("animate",TW_TYPE_BOOLCPP, set_animate,get_animate,this,
    " group='Animation' key='a' label='animate'"
    " help='Start or stop animating the current animation stack'",false);
  rebar->TwAddVarCB("animation_size",TW_TYPE_INT32,
    no_op,get_animation_size,this,
    " group='Animation' label='Animation stack size' readonly=true"
    " help='Number of keyframes in current animation stack'");
  rebar->TwAddVarRW("animation_interp_secs",TW_TYPE_DOUBLE,
    &animation_interp_secs,
    " group='Animation' label='Animation interp secs'"
    " help='Number of seconds to interpolation this keyframe with next'");
  TwEnumVal TransitionTypeEV[NUM_TRANSITION_TYPES] = 
  {
    {LINEAR_TRANSITION,"Linear"},
    {EASE_TRANSITION,"Ease"},
    {ABRUPT_TRANSITION,"Abrupt"},
    {ABRUPT_IN_TRANSITION,"Abrupt In"},
    {ABRUPT_OUT_TRANSITION,"Abrupt Out"}
  };
  TwType TransitionTypeTW = ReTwDefineEnum("TransitionType", TransitionTypeEV, NUM_TRANSITION_TYPES);
  rebar->TwAddVarRW("Transition",TransitionTypeTW,&transition_type,
    " group=Animation help='Transition Type for next keyframe'");
  rebar->TwAddButton("Push keyframe", push_keyframe, this,
    " group=Animation key='A' help='Push animation key frame'");
  rebar->TwAddButton( "Pop keyframe", pop_keyframe, this,
    " group=Animation key='Q' help='Pop animation key frame'");

  // Collapse group
  rebar->TwSetParam( "Animation", "opened", TW_PARAM_INT32, 1, &INT_ZERO);
}

void Skinning::display()
{
  /////////////////////////////////////////////////////////////////////////////
  // PRE-DISPLAY
  /////////////////////////////////////////////////////////////////////////////
  if(virgin_display)
  {
    initialize_shaders();
    virgin_display = false;
    display_count = 0;
    frames_per_lap = 10;
    // Start timer
    start_seconds = get_seconds();
    GLint n;
    glGetIntegerv(GL_MAX_VERTEX_ATTRIBS, &n);
    verbose("GL_MAX_VERTEX_ATTRIBS: %d\n",n);
    glGetIntegerv(GL_MAX_VERTEX_UNIFORM_COMPONENTS, &n);
    verbose("GL_MAX_VERTEX_UNIFORM_COMPONENTS: %d\n",n);
    verbose("GL_VERSION: %s\n", glGetString(GL_VERSION));
    verbose("GL_SHADING_LANGUAGE_VERSION: %s\n", 
      glGetString(GL_SHADING_LANGUAGE_VERSION));
    verbose("GL_RENDERER: %s\n",glGetString(GL_RENDERER));

    GLenum err = report_gl_error("virgin_display: ");
    assert(err == GL_NO_ERROR);
    clear();
  }
  update_rotation_animation();
  if(animating)
  {
  double t = get_seconds()-animation_start_seconds;
    bool still_animating = update_animation(t);
    if(!still_animating)
    {
      stop_animating();
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  // DISPLAY
  /////////////////////////////////////////////////////////////////////////////

  // Clear the screen with current background color
  if(always_clear)
  {
    clear();
  }

  // Turn on the lights
  lights();

  // Push the current scene:
  push_scene();

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_NORMALIZE);

  // Set up material
  material();

  // Make sure everything is in order to use current shader
  if(!validate_for_shader())
  {
    shader_mode = NO_SHADER;
  }

  glDisable(GL_LINE_SMOOTH);
  glDisable(GL_POINT_SMOOTH);



  ShaderMode old_shader_mode = shader_mode;
  if(use_cpu && old_shader_mode == LBS)
  {
    // temporarily use DIRECTIONAL_PER_PIXEL_COLOR
    shader_mode = DIRECTIONAL_PER_PIXEL_COLOR;
  }
  GLuint shader_program = shader_programs[shader_mode];
  // Set up current shader
  glUseProgram(shader_program);

  // If we're using LBS then we should update the handle_transformations
  if(shader_mode == LBS)
  {
    bool trans_success = transformations();
    if(!trans_success)
    {
      shader_mode = NO_SHADER;
    }else
    {
      bool send_trans_success = send_transformations();
      if(!send_trans_success)
      {
        shader_mode = NO_SHADER;
      }
    }
  }else if(shader_mode == WEIGHTCOLORS)
  {
    // Find NUM_HANDLES_NAME in shader uniforms map
    GLint num_handles_location = 
      glGetUniformLocation(
        shader_programs[WEIGHTCOLORS],
        NUM_HANDLES_NAME);
    glUniform1i(num_handles_location,OW.cols()+EW.cols());
  }else if(shader_mode == SCALAR)
  {
    // Find NUM_HANDLES_NAME in shader uniforms map
    GLint num_handles_location = 
      glGetUniformLocation(shader_programs[SCALAR], NUM_HANDLES_NAME);
    glUniform1i(num_handles_location,OW.cols()+EW.cols());
    // Find SELECTED_WEIGHT_NAME in shader uniforms map
    GLint selected_weight_location = 
      glGetUniformLocation(shader_programs[SCALAR], SELECTED_WEIGHT_NAME);
    glUniform1i(selected_weight_location,selected_weight);
  }

  if(!use_display_list || display_list_damage || use_cpu)
  {
    if(use_cpu)
    {
      int color[3] = { 255, 0, 255};
      rebar->TwSetParam(NULL, "color", TW_PARAM_INT32, 3, color);
    }else
    {
      rebar->TwSetParam(NULL, "color", TW_PARAM_INT32, 3, old_bar_color);
    }

    MatrixXd * pV = &V;
    if(V_depth_offsets.rows() == V.rows())
    {
      if(use_cpu)
      {
        // just recompute depth offsets each frame, so if reverse_depth_offsets
        // has been change the depth offset will too
        initialize_depth_offsets();
      }
      pV = &V_depth_offsets;
    }
    MatrixXd * pN;
    if(use_corner_normals)
    {
      pN = &CN;
    }else
    {
      pN = &N;
    }
    if(use_cpu)
    {
      bool trans_success = transformations();
      if(trans_success)
      {
        int dim = V.cols();
        MatrixXf Tcol;
        columnize<float,Eigen::Dynamic>(T.block(0,0,dim,T.cols()).eval(),T.cols()/4,2,Tcol);
        MatrixXd U3 = M*Tcol.cast<double>().eval();
        cpuV.resize(V.rows(),V.cols());
        cpuV.col(0) = U3.block(0,0,V.rows(),1);
        cpuV.col(1) = U3.block(V.rows(),0,V.rows(),1);
        cpuV.col(2) = U3.block(2*V.rows(),0,V.rows(),1);
        if(V_depth_offsets.rows() == V.rows())
        {
          cpuV.col(2) = V_depth_offsets.col(2);
        }
        if(use_corner_normals)
        {
          per_corner_normals(cpuV,F,corner_threshold,cpuN);
        }else
        {
          per_vertex_normals(cpuV,F,cpuN);
        }
        pV = &cpuV;
        pN = &cpuN;
      }
    }

    if(use_display_list)
    {
      glDeleteLists(display_list_id, 1);
      // generate new display list
      display_list_id = glGenLists(1);
      glNewList(display_list_id, GL_COMPILE);
    }

    GLuint W_index = 0;
    W_index = WEIGHTS0_ATTRIBUTE_LOCATION;
    GLuint WI_index = 0;
    WI_index = WEIGHT_INDICES0_ATTRIBUTE_LOCATION;

    // Use an empty matrix, so draw_mesh will ignore C
    MatrixXd E(0,0);
    MatrixXd * pC = &E;
    switch(color_option)
    {
      case COLOR_OPTION_ORIGINAL_WEIGHTS:
        pC = &C;
        break;
      case COLOR_OPTION_ONE_SETS:
        pC = &RC;
        break;
      case COLOR_OPTION_MATCH_DIFFUSE:
      default:
        // Set diffuse as color once
        glColor3fv(mat_diffuse);
        pC = &E;
        break;
    }
    MatrixXd * pTC = &E;
    if(use_texture_mapping)
    {
      // Set up texture and bind texture
      glEnable(GL_TEXTURE_2D);
      glBindTexture(GL_TEXTURE_2D, texture_id);
      assert(TC.rows() > 0);
      //report_gl_error("use_texture_mapping: ");
      glEnable(GL_COLOR_MATERIAL);
      glDisable(GL_LIGHTING);
      // Disable any colors
      pC = &E;
      pTC = &TC;
      // Use white instead
      glColor4f(1,1,1,1);
    }
    draw_mesh(*pV,F,*pN,*pC,*pTC,W,W_index, WI,WI_index);
    if(use_texture_mapping)
    {
      glDisable(GL_TEXTURE_2D);
    }
    
    if(use_display_list)
    {
      glEndList();
      // display lists are up to date
      display_list_damage = false;
    }
    if(use_cpu)
    {
      display_list_damage = true;
    }
  }
  // Draw current mesh display list
  if(use_display_list)
  {

    if(show_faces)
    {
      glEnable(GL_POLYGON_OFFSET_FILL); // Avoid Stitching!
      glPolygonOffset(1.0, 0);
      glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
#ifndef SKINNING_PROFILING
      glCallList(display_list_id);
#else
    glFinish();
    double sec_start = get_seconds_hires();
    const int numReps = 10;
    for (int i=0; i<numReps; i++) glCallList(display_list_id);    
    glFinish();
    double sec_end = get_seconds_hires();
    printf("Skinning time = %fms\n", (sec_end - sec_start)*1000.0 / numReps);
#endif
      glDisable(GL_POLYGON_OFFSET_FILL);
    }

    if(show_lines)
    {
      if(show_faces)
      {
        // Easy way to set color to black
        glDisable(GL_LIGHTING);
        glEnable(GL_COLOR_MATERIAL);
        glColor4f(0,0,0,1);
      }
      glLineWidth(1.0);
      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
      glCallList(display_list_id);
    }
  }

  // Stop using any shader
  glUseProgram(0);
  // restore shader mode
  if(use_cpu)
  {
    shader_mode = old_shader_mode;
  }

  if(draw_bones)
  {
    // get bones
    vector<Bone *> B = gather_bones(BR);
    // First draw bones without depth ordering (so all appear on top)
    glDisable(GL_DEPTH_TEST);
    for_each(B.begin(),B.end(),mem_fun(&Bone::draw));
    // Then draw again with depth ordering (so ordering comes out according to
    // depth)
    glEnable(GL_DEPTH_TEST);
    for_each(B.begin(),B.end(),mem_fun(&Bone::draw));
  }

  // Pop the current scene
  pop_scene();

  // Stop timer

  // Draw AntTweakBar
  if(draw_anttweakbar)
  {
    if(bypass_auto_dof)
    {
      int cur_color[3];
      rebar->TwGetParam(NULL, "color", TW_PARAM_INT32, 3, cur_color);
      int color[3] = { 255, 255, 0};
      color[0] = int(double(color[0])*0.5 +  double(cur_color[0])*0.5);
      color[1] = int(double(color[1])*0.5 +  double(cur_color[1])*0.5);
      color[2] = int(double(color[2])*0.5 +  double(cur_color[2])*0.5);
      rebar->TwSetParam(NULL, "color", TW_PARAM_INT32, 3, color);
    }
    //!!! TwDraw();
    if(bypass_auto_dof)
    {
      rebar->TwSetParam(NULL, "color", TW_PARAM_INT32, 3, old_bar_color);
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  // POST-DISPLAY
  /////////////////////////////////////////////////////////////////////////////
  // Display is now up-to-date
  damage = false;
  // But we force to up date again if we're animating a spin
  damage = spinning_about_up_axis;
  display_count++;
  if(display_count == frames_per_lap)
  {
    double end_seconds = get_seconds();
    fps = (double)display_count/(end_seconds-start_seconds);
    mspf = 1000.0*(end_seconds-start_seconds)/(double)display_count;
    if(!draw_anttweakbar)
    {
      verbose("%g fps\n",fps);
      verbose("%g mspf\n",mspf);
    }

    // Adjust frames per lap so that time per lap is close to 10
    if((end_seconds-start_seconds)>3.0)
    {
      // too many frames
      frames_per_lap /= 2.0;
    }
    // Adjust frames per lap so that time per lap is close to 10
    else if((end_seconds-start_seconds)<1.0)
    {
      // too few frames
      frames_per_lap *= 1.5;
    }

    // Start timer
    start_seconds = get_seconds();
    display_count = 0;
  }
  glFlush();
}

void Skinning::displayCrowd(int xNum, int yNum, float xOffset, float yOffset)
{
  /////////////////////////////////////////////////////////////////////////////
  // PRE-DISPLAY
  /////////////////////////////////////////////////////////////////////////////
  if(virgin_display)
  {
    initialize_shaders();
    virgin_display = false;
    display_count = 0;
    frames_per_lap = 10;
    // Start timer
    start_seconds = get_seconds();
    GLint n;
    glGetIntegerv(GL_MAX_VERTEX_ATTRIBS, &n);
    verbose("GL_MAX_VERTEX_ATTRIBS: %d\n",n);
    glGetIntegerv(GL_MAX_VERTEX_UNIFORM_COMPONENTS, &n);
    verbose("GL_MAX_VERTEX_UNIFORM_COMPONENTS: %d\n",n);
    verbose("GL_VERSION: %s\n", glGetString(GL_VERSION));
    verbose("GL_SHADING_LANGUAGE_VERSION: %s\n", 
      glGetString(GL_SHADING_LANGUAGE_VERSION));
    verbose("GL_RENDERER: %s\n",glGetString(GL_RENDERER));

    GLenum err = report_gl_error("virgin_display: ");
    assert(err == GL_NO_ERROR);
    clear();
  }
  update_rotation_animation();

  /////////////////////////////////////////////////////////////////////////////
  // DISPLAY
  /////////////////////////////////////////////////////////////////////////////

  // Clear the screen with current background color
  if(always_clear)
  {
    clear();
  }

  // Turn on the lights
  lights();

  // Push the current scene:
  push_scene();

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_NORMALIZE);

  // Set up material
  material();

  // Make sure everything is in order to use current shader
  if(!validate_for_shader())
  {
    shader_mode = NO_SHADER;
  }

  glDisable(GL_LINE_SMOOTH);
  glDisable(GL_POINT_SMOOTH);



  ShaderMode old_shader_mode = shader_mode;
  if(use_cpu && old_shader_mode == LBS)
  {
    // temporarily use DIRECTIONAL_PER_PIXEL_COLOR
    shader_mode = DIRECTIONAL_PER_PIXEL_COLOR;
  }
  GLuint shader_program = shader_programs[shader_mode];
  // Set up current shader
  glUseProgram(shader_program);

  // If we're using LBS then we should update the handle_transformations
  if(shader_mode == LBS)
  {
  }else if(shader_mode == WEIGHTCOLORS)
  {
    // Find NUM_HANDLES_NAME in shader uniforms map
    GLint num_handles_location = 
      glGetUniformLocation(
      shader_programs[WEIGHTCOLORS],
      NUM_HANDLES_NAME);
    glUniform1i(num_handles_location,OW.cols()+EW.cols());
  }else if(shader_mode == SCALAR)
  {
    // Find NUM_HANDLES_NAME in shader uniforms map
    GLint num_handles_location = 
      glGetUniformLocation(
      shader_programs[SCALAR],
      NUM_HANDLES_NAME);
    glUniform1i(num_handles_location,OW.cols()+EW.cols());
    // Find SELECTED_WEIGHT_NAME in shader uniforms map
    GLint selected_weight_location = 
      glGetUniformLocation(
      shader_programs[SCALAR],
      SELECTED_WEIGHT_NAME);
    glUniform1i(selected_weight_location,selected_weight);
  }

  if(!use_display_list || display_list_damage || use_cpu)
  {
    if(use_cpu)
    {
      int color[3] = { 255, 0, 255};
      rebar->TwSetParam(NULL, "color", TW_PARAM_INT32, 3, color);
    }else
    {
      rebar->TwSetParam(NULL, "color", TW_PARAM_INT32, 3, old_bar_color);
    }

    MatrixXd * pV = &V;
    MatrixXd * pN;
    if(use_corner_normals)
    {
      pN = &CN;
    }else
    {
      pN = &N;
    }
    if(use_cpu)
    {
      bool trans_success = transformations();
      if(trans_success)
      {
        int dim = V.cols();
        MatrixXf Tcol;
        columnize<float,Eigen::Dynamic>(T.block(0,0,dim,T.cols()).eval(),T.cols()/4,2,Tcol);
        MatrixXd U3 = M*Tcol.cast<double>().eval();
        cpuV.resize(V.rows(),V.cols());
        cpuV.col(0) = U3.block(0,0,V.rows(),1);
        cpuV.col(1) = U3.block(V.rows(),0,V.rows(),1);
        cpuV.col(2) = U3.block(2*V.rows(),0,V.rows(),1);
        if(use_corner_normals)
        {
          per_corner_normals(cpuV,F,corner_threshold,cpuN);
        }else
        {
          per_vertex_normals(cpuV,F,cpuN);
        }
        pV = &cpuV;
        pN = &cpuN;
      }
    }

    if(use_display_list)
    {
      glDeleteLists(display_list_id, 1);
      // generate new display list
      display_list_id = glGenLists(1);
      glNewList(display_list_id, GL_COMPILE);
    }

    GLuint W_index = 0;
    //if(shader_attribs[shader_mode].find(WEIGHTS0_ATTRIBUTE_NAME) !=
    //  shader_attribs[shader_mode].end())
    //{
    //  W_index = shader_attribs[shader_mode][WEIGHTS0_ATTRIBUTE_NAME];
    //}
    W_index = WEIGHTS0_ATTRIBUTE_LOCATION;
    GLuint WI_index = 0;
    //if(shader_attribs[shader_mode].find(WEIGHT_INDICES0_ATTRIBUTE_NAME) !=
    //  shader_attribs[shader_mode].end())
    //{
    //  WI_index = shader_attribs[shader_mode][WEIGHT_INDICES0_ATTRIBUTE_NAME];
    //}
    WI_index = WEIGHT_INDICES0_ATTRIBUTE_LOCATION;

    // Use an empty matrix, so draw_mesh will ignore C
    MatrixXd E(0,0);
    MatrixXd * pC = &E;
    switch(color_option)
    {
    case COLOR_OPTION_ORIGINAL_WEIGHTS:
      pC = &C;
      break;
    case COLOR_OPTION_ONE_SETS:
      pC = &RC;
      break;
    case COLOR_OPTION_MATCH_DIFFUSE:
    default:
      // Set diffuse as color once
      glColor3fv(mat_diffuse);
      pC = &E;
      break;
    }
    MatrixXd * pTC = &E;
    if(use_texture_mapping)
    {
      // Set up texture and bind texture
      glEnable(GL_TEXTURE_2D);
      glBindTexture(GL_TEXTURE_2D, texture_id);
      // Disable any colors
      pC = &E;
      // Use white instead
      glColor4f(0,0,0,0);
    }
    draw_mesh(*pV,F,*pN,*pC,*pTC,W,W_index, WI,WI_index);
    if(use_texture_mapping)
    {
      glDisable(GL_TEXTURE_2D);
    }
  }
  if(use_display_list)
  {
    glEndList();
    // display lists are up to date
    display_list_damage = false;
  }
  if(use_cpu)
  {
    display_list_damage = true;
  }
  // Draw current mesh display list
  if(use_display_list)
  {

    if(show_faces)
    {
      glEnable(GL_POLYGON_OFFSET_FILL); // Avoid Stitching!
      glPolygonOffset(1.0, 0);
      glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        double timer_start = get_seconds_hires();

      // hack: flip light direction so we get correct lighting
      float v[2][4];
      v[0][0] = light_direction[0]; v[0][1] = light_direction[1]; v[0][2] = light_direction[2]; v[0][3] = 0.0f;
      v[1][0] = light2_direction[0]; v[1][1] = light2_direction[1]; v[1][2] = light2_direction[2]; v[1][3] = 0.0f;
      glLightfv(GL_LIGHT0, GL_POSITION, v[0]);
      glLightfv(GL_LIGHT1, GL_POSITION, v[1]);

//!!!      glRotatef(180.0, 0.0f, 1.0f, 0.0f);

      if (play_animation) anim_timer += 1.0 / 30; // advance by 33ms
      const double totalAnimationDuration = animation.getTotalDuration();

      srand(0);
      for (int y=0; y<yNum; y++)
      {
        for (int x=0; x<xNum; x++)
        {
          const int entID = x + y*xNum;
          
          float lead_color[4] = {255.0f / 255.0f, 235.0f / 255.0f, ((rand())%255) / 255.0f, 1.0f};
          glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,  lead_color);
          glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  lead_color);

          
          double t = fmod(anim_timer + 172.35*entID, totalAnimationDuration);
          update_animation(t);

          const bool trans_success = transformations();
          const bool send_trans_success = send_transformations();
          assert(trans_success && send_trans_success);

          //const float xt = x*xOffset, yt = 0.0f, zt = - y * yOffset;
          //const float xt = -0.5f*xNum*xOffset + x*xOffset, yt = 0.0f, zt = -0.5f*yNum*yOffset + y * yOffset;
          const float xt = -0.5f*xNum*xOffset + x*xOffset + +0.5*xOffset + 0.5*(y % 2)*xOffset, yt = 0.0f, zt = -y * yOffset - lead_armadillo_distance*yOffset;
          glTranslatef(xt, yt, zt);
          glCallList(display_list_id);
          glTranslatef(-xt, -yt, -zt);
        }
      }

      //// go back:
      //glRotatef(-180.0, 0.0f, 1.0f, 0.0f);
      //v[0][0] = -v[0][0]; v[0][1] = -v[0][1]; v[0][2] = -v[0][2];
      //v[1][0] = -v[1][0]; v[1][1] = -v[1][1]; v[1][2] = -v[1][2];
      //glLightfv(GL_LIGHT0, GL_POSITION, v[0]);
      //glLightfv(GL_LIGHT1, GL_POSITION, v[1]);

      // and render the lead armadillo:
      float lead_color[4] = {1.0f, 125.0f / 255.0f, 19.0f / 255.0f, 0.0f};
      glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,  lead_color);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  lead_color);

      double t = fmod(anim_timer + 0.0, totalAnimationDuration);
      update_animation(t);

      const bool trans_success = transformations();
      const bool send_trans_success = send_transformations();
      assert(trans_success && send_trans_success);

      glCallList(display_list_id);      

        double timer_end = get_seconds_hires();
        printf("Animation update time = %fms\n", (timer_end - timer_start)*1000.0);

      //// set animation to draw correct skeleton pose for the first character
      //double t = fmod(anim_timer + 0.0, totalAnimationDuration);
      //update_animation(t);
      //transformations();

      //glFinish();
      //double sec_start = get_seconds_hires();
      //const int numReps = 10;
      //for (int i=0; i<numReps; i++) glCallList(display_list_id);    
      //glFinish();
      //double sec_end = get_seconds_hires();
      //printf("Skinning time = %fms\n", (sec_end - sec_start)*1000.0 / numReps);

      glDisable(GL_POLYGON_OFFSET_FILL);
    }

    if(show_lines)
    {
      if(show_faces)
      {
        // Easy way to set color to black
        glDisable(GL_LIGHTING);
        glEnable(GL_COLOR_MATERIAL);
        glColor4f(0,0,0,1);
      }
      glLineWidth(1.0);
      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
      glCallList(display_list_id);
    }
  }

  // Stop using any shader
  glUseProgram(0);
  // restore shader mode
  if(use_cpu)
  {
    shader_mode = old_shader_mode;
  }

  draw_floor(floor_depth);

  if(draw_bones)
  {
    // get bones
    vector<Bone *> B = gather_bones(BR);
    // First draw bones without depth ordering (so all appear on top)
    glDisable(GL_DEPTH_TEST);
    for_each(B.begin(),B.end(),mem_fun(&Bone::draw));
    // Then draw again with depth ordering (so ordering comes out according to
    // depth)
    glEnable(GL_DEPTH_TEST);
    for_each(B.begin(),B.end(),mem_fun(&Bone::draw));
  }

  // Pop the current scene
  pop_scene();

  // Draw AntTweakBar
  if(draw_anttweakbar)
  {
    if(bypass_auto_dof)
    {
      int cur_color[3];
      rebar->TwGetParam(NULL, "color", TW_PARAM_INT32, 3, cur_color);
      int color[3] = { 255, 255, 0};
      color[0] = int(double(color[0])*0.5 +  double(cur_color[0])*0.5);
      color[1] = int(double(color[1])*0.5 +  double(cur_color[1])*0.5);
      color[2] = int(double(color[2])*0.5 +  double(cur_color[2])*0.5);
      rebar->TwSetParam(NULL, "color", TW_PARAM_INT32, 3, color);
    }
    //!!! TwDraw();
    if(bypass_auto_dof)
    {
      rebar->TwSetParam(NULL, "color", TW_PARAM_INT32, 3, old_bar_color);
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  // POST-DISPLAY
  /////////////////////////////////////////////////////////////////////////////
  // Display is now up-to-date
  damage = false;
  // But we force to up date again if we're animating a spin
  damage = spinning_about_up_axis;
  display_count++;
  if(display_count == frames_per_lap)
  {
    // Stop timer
    double end_seconds = get_seconds();
    fps = (double)display_count/(end_seconds-start_seconds);
    mspf = 1000.0*(end_seconds-start_seconds)/(double)display_count;
    if(!draw_anttweakbar)
    {
      verbose("%g fps\n",fps);
      verbose("%g mspf\n",mspf);
    }

    // Adjust frames per lap so that time per lap is close to 10
    if((end_seconds-start_seconds)>3.0)
    {
      // too many frames
      frames_per_lap /= 2.0;
    }
    // Adjust frames per lap so that time per lap is close to 10
    else if((end_seconds-start_seconds)<1.0)
    {
      // too few frames
      frames_per_lap *= 1.5;
    }

    // Start timer
    start_seconds = get_seconds();
    display_count = 0;
  }
}

void Skinning::resize(const double width, const double height)
{
  this->width = width;
  this->height = height;
  glViewport(0,0,width,height);

  // Also tell AntTweakBar about size change
  resize_anttweakbar(width,height);

  damage = true;
}

void Skinning::resize_anttweakbar(const double width, const double height)
{
  // Tell AntTweakBar about new size
  TwWindowSize(width, height);

  // Keep AntTweakBar on right side of screen and height == opengl height
  // get the current position of a bar
  int size[2];
  rebar->TwGetParam(NULL, "size", TW_PARAM_INT32, 2, size);
  int pos[2];
  // Place bar on right side of opengl rect (padded by 10 pixels)
  pos[0] = max(10,(int)width - size[0] - 10);
  // place bar at top (padded by 10 pixels)
  pos[1] = 10;
  // Set height to new height of window (padded by 10 pixels on bottom)
  size[1] = height-pos[1]-10;
  rebar->TwSetParam( NULL, "position", TW_PARAM_INT32, 2, pos);
  rebar->TwSetParam( NULL, "size", TW_PARAM_INT32, 2,size);
}

void Skinning::push_scene()
{

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  // set up perspective
  //// from: http://nehe.gamedev.net/data/articles/article.asp?article=11
  //double frustum_shift_x = 0;// -0.125;
  double zNear = 1e-2;
  double zFar = 100;
  //double fH = tan( angle / 360.0 * PI ) * near;
  //double fW = fH * (double)width/(double)height;
  //// shift everything a little to the right
  //glFrustum( -fW-frustum_shift_x, fW-frustum_shift_x, -fH, fH,near, far);
  double aspect = ((double)width)/((double)height);

  // Amount of scaling needed to "fix" perspective z-shift
  double z_fix=1.0;
  // 5 is far enough to see unit "things" well
  const double camera_z = 2;
  // Test if should be using true orthographic projection
  if(angle == 0)
  {
    glOrtho(
      -0.5*camera_z*aspect,
      0.5*camera_z*aspect,
      -0.5*camera_z,
      0.5*camera_z,
      zNear,
      zFar);
  }else
  {
    angle = max(angle,min_angle);
    gluPerspective(angle,aspect,zNear,zFar);
    z_fix = 2*tan(angle/2/360*2*PI);
  }

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  // look at origin from (0,0,camera_z) with (0,1,0) pointing up
  gluLookAt(0.0,0,camera_z, 0,0,0, 0,1,0);

  // Rotate according to current object rotation
  glPushMatrix();

  // Snap current rotation (according to snap threshold) to nearest canonical
  // view quaternion)
  float snapped_rotation[4];
  snap_to_canonical_view_quat<float>(
    rotation,
    snap_threshold,
    snapped_rotation);

  // Convert quaternion representation of rotation to matrix
  float mat[4*4];
  quat_to_mat(snapped_rotation,mat);
  glMultMatrixf(mat);
  // Adjust scale to correct perspective
  glScaled(z_fix,z_fix,z_fix);
  // Zoom
  glScaled(zoom, zoom, zoom);
  glTranslatef(pan[0],pan[1],pan[2]);
  // Scale and shift
  glScaled(scale, scale, scale);
  glTranslatef(shift[0],shift[1],shift[2]);

}

void Skinning::pop_scene()
{
  glPopMatrix();
}

void Skinning::material()
{  
  // Turn-off per vertex/manual colors befor specifying material properties
  glDisable(GL_COLOR_MATERIAL);
  // Set material properties
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,  mat_ambient);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,  mat_diffuse);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
  // Set color to diffuse color to be sure that a proper color is in the color
  // field in case anybody looks there hoping to find something interesting
  glColor3fv(mat_diffuse);
}

void Skinning::lights()
{  
  // Set light
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  float v[4], ones[4] = {1.0f, 1.0f, 1.0f, 1.0f};
  v[0] = v[1] = v[2] = light_multiplier*0.4f; v[3] = 1.0f;
  glLightfv(GL_LIGHT0, GL_AMBIENT, v);
  v[0] = v[1] = v[2] = light_multiplier*0.8f; v[3] = 1.0f;
  glLightfv(GL_LIGHT0, GL_DIFFUSE, v);
  // Flip light vector
  v[0] = -light_direction[0]; v[1] = -light_direction[1]; v[2] = -light_direction[2]; v[3] = 0.0f;
  glLightfv(GL_LIGHT0, GL_POSITION, v);
  glLightfv(GL_LIGHT0, GL_SPECULAR, ones);

  glEnable(GL_LIGHT1);  
  v[0] = v[1] = v[2] = light_multiplier*0.4f; v[3] = 1.0f;
  glLightfv(GL_LIGHT1, GL_AMBIENT, v);
  v[0] = v[1] = v[2] = light_multiplier*0.8f; v[3] = 1.0f;
  glLightfv(GL_LIGHT1, GL_DIFFUSE, v);
  // Flip light vector  
  v[0] = -light2_direction[0]; v[1] = -light2_direction[1]; v[2] = -light2_direction[2]; v[3] = 0.0f;
  glLightfv(GL_LIGHT1, GL_POSITION, v);
  glLightfv(GL_LIGHT1, GL_SPECULAR, ones);
}

void Skinning::clear()
{
  if(time_clear_color)
  {
    glClearColor(
      fabs(sin(get_seconds())),
      fabs(sin(get_seconds()/3)),
      fabs(sin(get_seconds()/7)),
      0.0);
  }else
  {
    glClearColor(background[0],background[1],background[2],0.0);
  }
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Skinning::set_dial_in_each_T(const bool v)
{
  dial_in_each_T = v;
  rebar->TwSetParam("sb_last_T", "readonly", TW_PARAM_INT32, 1, 
    (dial_in_each_T ? &INT_ZERO : &INT_ONE));
}

void Skinning::initialize_mouse_and_keyboard()
{
  mouse_is_down = false;
  trackball_speed_factor = 2;
  mouse_scroll_y = 0;
  //center_root_rotations = true;
  view_vector = VIEW_VECTOR;

}

bool Skinning::key_down(int key, 
  int mouse_x,
  int mouse_y,
  bool shift_down,
  bool control_down,
  bool meta_down)
{
  int tw_mod = 
    build_anttweakbar_modifier(shift_down,control_down,meta_down);
  // Let AntTweakBar know where the mouse is
  TwMouseMotion(mouse_x,height-mouse_y);
  // First let AntTweakBar try to grab the mouse event
  if(TwKeyPressed(cocoa_key_to_anttweakbar_key(key),tw_mod))
  {
    return damage=true;
  }
  return false;
}

bool Skinning::key_up(int key, 
  int mouse_x,
  int mouse_y,
  bool shift_down,
  bool control_down,
  bool meta_down)
{
  return false;
}

bool Skinning::mouse_down( 
  int mouse_x,
  int mouse_y,
  bool shift_down,
  bool control_down,
  bool meta_down)
{
  mouse_is_down = true;
  // assume click will not be in anttweakbar
  down_in_anttweakbar = false;
  // Default is trackball off
  trackball_on = false;
  // keep track of mouse position on down
  down_mouse_x = mouse_x;
  down_mouse_y = mouse_y;
  // keep track of object rotation on down
  copy(rotation,rotation+4,down_rotation);
  // Let AntTweakBar know where the mouse is
  TwMouseMotion(mouse_x,height-mouse_y);
  // First let AntTweakBar try to grab the mouse event
  if(TwMouseButton(TW_MOUSE_PRESSED,TW_MOUSE_LEFT))
  {
    down_in_anttweakbar = true;
    return damage=true;
  }

  bool bone_down = false;
  bool reset_selection = true;
  push_scene();
  // Get bones
  vector<Bone*> B = gather_bones(BR);
  // check bones
  for(vector<Bone*>::iterator bit = B.begin();bit != B.end();bit++)
  {
    Bone * b = (*bit);
    if(b->down(mouse_x,mouse_y,false,shift_down,control_down,meta_down))
    {
      bone_down = true;
      reset_selection &=
        ((b->was_tip_selected && b->is_tip_selected) || 
        (b->was_line_segment_selected && b->is_line_segment_selected));
    }
  }

  pop_scene();
  if(bone_down)
  {
    if(reset_selection)
    {
      // if control is down and we have a click then reset selection
      for(vector<Bone*>::iterator bit = B.begin();bit != B.end();bit++)
      {
        Bone * b = (*bit);
        if(!b->is_down)
        {
          b->set_is_tip_selected(b->was_tip_selected);
          b->set_is_line_segment_selected(b->was_line_segment_selected);
        }
      }
    }
    return damage = true;
  }

  trackball_on = true;
  return damage = true;
}

bool Skinning::mouse_up( 
  int mouse_x,
  int mouse_y,
  bool shift_down,
  bool control_down,
  bool meta_down)
{
  mouse_is_down = false;

  bool bone_up = false;
  push_scene();
  // Get bones
  vector<Bone*> B = gather_bones(BR);
  // check bones
  for(vector<Bone*>::iterator bit = B.begin();bit != B.end();bit++)
  {
    Bone * b = (*bit);
    if(b->up(mouse_x,mouse_y,false,shift_down,control_down,meta_down))
    {
      bone_up= true;
    }
  }
  pop_scene();
  damage |= bone_up;

  // Let AntTweakBar know where the mouse is
  TwMouseMotion(mouse_x,height-mouse_y);
  // First let AntTweakBar try to grab the mouse event
  damage |= TwMouseButton(TW_MOUSE_RELEASED,TW_MOUSE_LEFT);

  return damage;
}

bool Skinning::mouse_move( 
  int mouse_x,
  int mouse_y,
  bool shift_down,
  bool control_down,
  bool meta_down)
{
  // Let AntTweakBar know where the mouse is
  damage |= TwMouseMotion(mouse_x,height-mouse_y);

  bool bone_move = false;
  push_scene();
  // Get bones
  vector<Bone*> B = gather_bones(BR);
  // check bones
  for(vector<Bone*>::iterator bit = B.begin();bit != B.end();bit++)
  {
    Bone * b = (*bit);
    if(b->move(mouse_x,mouse_y,shift_down,control_down,meta_down))
    {
      bone_move = true;
    }
  }
  pop_scene();
  if(bone_move)
  {
    return damage = true;
  }

  return true;
}

bool Skinning::mouse_drag( 
  int mouse_x,
  int mouse_y,
  bool shift_down,
  bool control_down,
  bool meta_down)
{
  // Let AntTweakBar know where the mouse is
  damage |= TwMouseMotion(mouse_x,height-mouse_y);
  if(down_in_anttweakbar)
  {
    return damage;
  }

  if(trackball_on)
  {
    trackball<float>(
      width,
      height,
      trackball_speed_factor,
      down_rotation,
      down_mouse_x,
      height-down_mouse_y,
      mouse_x,
      height-mouse_y,
      rotation);
    return damage = true;
  }

  bool bone_drag = false;
  push_scene();
  // Get bones
  vector<Bone*> B = gather_bones(BR);
  // check bones
  for(vector<Bone*>::iterator bit = B.begin();bit != B.end();bit++)
  {
    Bone * b = (*bit);
    if(b->drag(mouse_x,mouse_y,false,shift_down,control_down,meta_down))
    {
      bone_drag = true;
    }
  }
  pop_scene();
  if(bone_drag)
  {
    return damage = true;
  }

  return damage = true;
}

bool Skinning::right_mouse_down( 
  int mouse_x,
  int mouse_y,
  bool shift_down,
  bool control_down,
  bool meta_down)
{
  // assume click will not be in anttweakbar
  down_in_anttweakbar = false;
  // Let AntTweakBar know where the mouse is
  TwMouseMotion(mouse_x,height-mouse_y);
  // First let AntTweakBar try to grab the mouse event
  if(TwMouseButton(TW_MOUSE_PRESSED,TW_MOUSE_RIGHT))
  {
    down_in_anttweakbar = true;
    return damage=true;
  }

  bool bone_down = false;
  bool reset_selection = true;
  push_scene();
  // Get bones
  vector<Bone*> B = gather_bones(BR);
  // check bones
  for(vector<Bone*>::iterator bit = B.begin();bit != B.end();bit++)
  {
    Bone * b = (*bit);
    if(b->down(mouse_x,mouse_y,true,shift_down,control_down,meta_down))
    {
      bone_down = true;
      reset_selection &=
        ((b->was_tip_selected && b->is_tip_selected) || 
        (b->was_line_segment_selected && b->is_line_segment_selected));
    }
  }
  pop_scene();
  if(bone_down)
  {
    if(reset_selection)
    {
      // if control is down and we have a click then reset selection
      for(vector<Bone*>::iterator bit = B.begin();bit != B.end();bit++)
      {
        Bone * b = (*bit);
        if(!b->is_down)
        {
          b->set_is_tip_selected(b->was_tip_selected);
          b->set_is_line_segment_selected(b->was_line_segment_selected);
        }
      }
    }
    return damage = true;
  }
  return false;
}

bool Skinning::right_mouse_up( 
  int mouse_x,
  int mouse_y,
  bool shift_down,
  bool control_down,
  bool meta_down)
{
  // Let AntTweakBar know where the mouse is
  damage |= TwMouseMotion(mouse_x,height-mouse_y);

  bool bone_up = false;
  push_scene();
  // Get bones
  vector<Bone*> B = gather_bones(BR);
  // check bones
  for(vector<Bone*>::iterator bit = B.begin();bit != B.end();bit++)
  {
    Bone * b = (*bit);
    if(b->up(mouse_x,mouse_y,true,shift_down,control_down,meta_down))
    {
      bone_up= true;
    }
  }
  pop_scene();
  damage |= bone_up;
  // First let AntTweakBar try to grab the mouse event
  damage |= TwMouseButton(TW_MOUSE_RELEASED,TW_MOUSE_RIGHT);

  return damage;
}

bool Skinning::right_mouse_drag( 
  int mouse_x,
  int mouse_y,
  bool shift_down,
  bool control_down,
  bool meta_down)
{
  // Let AntTweakBar know where the mouse is
  damage |= TwMouseMotion(mouse_x,height-mouse_y);
  if(down_in_anttweakbar)
  {
    return damage;
  }

  bool bone_drag = false;
  push_scene();
  // Get bones
  vector<Bone*> B = gather_bones(BR);
  // check bones
  for(vector<Bone*>::iterator bit = B.begin();bit != B.end();bit++)
  {
    Bone * b = (*bit);
    if(b->drag(mouse_x,mouse_y,true,shift_down,control_down,meta_down))
    {
      bone_drag = true;
    }
  }
  pop_scene();
  if(bone_drag)
  {
    return damage = true;
  }

  return false;
}

bool Skinning::mouse_scroll(
  int mouse_x,
  int mouse_y,
  float delta_x,
  float delta_y)
{
  mouse_scroll_y += delta_y;
  if(TwMouseMotion(mouse_x,height-mouse_y))
  {
    TwMouseWheel(mouse_scroll_y);
    return damage=true;
  }

  // zoom on change in scroll y
  const double factor = 0.1f;
  if(!zoom_locked)
  {
    zoom = min(max(zoom - delta_y*factor,min_zoom),max_zoom);
  }
  if(!angle_locked)
  {
    angle = min(max(angle - delta_x,min_angle),135.0);
  }
  return damage=true;
}

bool Skinning::load_mesh_from_file(const std::string mesh_file_name)
{
  // Load mesh file into V,F
  verbose("Loading mesh from file %s...",mesh_file_name.c_str());
  string dirname, basename, extension, filename;
  pathinfo(mesh_file_name,dirname,basename,extension,filename);
  transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
  bool success = false;
  if(extension == "obj")
  {
    Tets.resize(0,0);
    success = readOBJ(mesh_file_name,V,F);
  }else if(extension == "off")
  {
    Tets.resize(0,0);
    success = readOFF(mesh_file_name,V,F);
  }else if(extension == "mesh")
  {
    success = readMESH(mesh_file_name,V,Tets,F);
  }else
  {
    fprintf(stderr,
      "Error: load_mesh_from_file: unsupported mesh exenstion %s\n",
      extension.c_str());
    success = false;
  }
  verbose("%s\n",(success?"succeeded":"failed"));
  if(!success)
  {
    return false;
  }
  if(V.size() == 0)
  {
     return false;
  }

#ifdef OPTIMIZE_INDEX_BUFFER
  optimize_index_buffer(F,false,F);
#endif

  // Get scale and shift to fit mesh to unit sphere at origin
  if(scale_and_shift_on_load)
  {
    shift[0] = -V.col(0).mean();
    shift[1] = -V.col(1).mean();
    shift[2] = -V.col(2).mean();
    scale = 2.0/
      max(V.col(0).maxCoeff() - V.col(0).minCoeff(),
        max(V.col(1).maxCoeff() - V.col(1).minCoeff(),
          V.col(2).maxCoeff() - V.col(2).minCoeff()));
#ifdef EXTREME_VERBOSE
    verbose("shift: %g %g %g\nscale: %g\n",shift[0],shift[1],shift[2],scale);
#endif
    double step = 0.01/scale;
    rebar->TwSetParam("sb_translation_x", "step", TW_PARAM_DOUBLE, 1, &step);
    rebar->TwSetParam("sb_translation_y", "step", TW_PARAM_DOUBLE, 1, &step);
    rebar->TwSetParam("sb_translation_z", "step", TW_PARAM_DOUBLE, 1, &step);
    rebar->TwSetParam("sb_translation_x", "step", TW_PARAM_DOUBLE, 1, &step);
  }else
  {
    // we're purposely keeping the last mesh's scale and shift
  }

  // Vary sb_translation anttweakbar widget 'step' based on scale

  // Compute vertex normals
  per_corner_normals(V,F,corner_threshold,CN);
  per_vertex_normals(V,F,N);

  if(C.rows() != V.rows())
  {
    // Phony colors
    C = (V.array()-V.minCoeff()).matrix()/V.maxCoeff()*0.75;
    // Reset region colors
    RC.resize(0,0);
  }

  if(V.rows() != W.rows())
  {
    // Clear weights
    verbose("Clearing weights since V.rows() (%d) != W.rows() (%d)\n",
      V.rows(),
      W.rows());
    W.resize (V.rows(),0);
    OW.resize(V.rows(),0);
    EW.resize(V.rows(),0);
    WI.resize(V.rows(),0);
  }



  initialize_depth_offsets();

  initialize_texture_mapping();


  display_list_damage = true;

  return true;
}

bool Skinning::save_deformed_mesh_to_file(const std::string mesh_file_name)
{
  // Load mesh file into V,F
  verbose("Saving deformed mesh to file %s...",mesh_file_name.c_str());
  if(!use_cpu && shader_mode != LBS)
  {
    verbose(
      "save_deformed_mesh_to_file: warning: you're not viewing the mesh that's "
      "being saved. Switch shader to LBS or use cpu\n");
  }
  string dirname, basename, extension, filename;
  pathinfo(mesh_file_name,dirname,basename,extension,filename);
  transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
  bool trans_success = transformations();
  if(!trans_success)
  {
    return false;
  }
  int dim = V.cols();
  MatrixXf Tcol;
  columnize<float,Eigen::Dynamic>(T.block(0,0,dim,T.cols()).eval(),T.cols()/4,2,Tcol);
  MatrixXd U3 = M*Tcol.cast<double>().eval();
  cpuV.resize(V.rows(),V.cols());
  cpuV.col(0) = U3.block(0,0,V.rows(),1);
  cpuV.col(1) = U3.block(V.rows(),0,V.rows(),1);
  cpuV.col(2) = U3.block(2*V.rows(),0,V.rows(),1);
  if(use_corner_normals)
  {
    per_corner_normals(cpuV,F,corner_threshold,cpuN);
  }else
  {
    per_vertex_normals(cpuV,F,cpuN);
  }
  
  bool success = false;
  if(extension == "obj")
  {
    success = writeOBJ(mesh_file_name,cpuV,F);
  }else if(extension == "off")
  {
    success = writeOFF(mesh_file_name,cpuV,F);
  }else
  {
    fprintf(stderr,
      "Error: save_deformed_mesh_to_file: unsupported mesh exenstion %s\n",
      extension.c_str());
    success = false;
  }
  return success;
}

bool Skinning::load_texture(const std::string tga_file_name)
{
  // read in texture and set up texture in openGL
  bool load_success = texture_from_tga(tga_file_name,texture_id);
  if(!load_success)
  {
    return false;
  }
  use_texture_mapping = true;
  use_cpu = true;
  display_list_damage = true;
  shader_mode = NO_SHADER;
  return true;
}

bool Skinning::initialize_texture_mapping()
{
  if(V.size() == 0)
  {
    fprintf(stderr,"initialize_texture_mapping: Error: V.size() == 0\n");
    return false;
  }
  assert(V.cols() >= 2);
  TC = V.block(0,0,V.rows(),2);
  double min_x,min_y,max_x,max_y;
  min_x = TC.col(0).minCoeff();
  min_y = TC.col(1).minCoeff();
  max_x = TC.col(0).maxCoeff();
  max_y = TC.col(1).maxCoeff();
  TC.col(0) = (TC.col(0).array() - min_x)/(max_x-min_x);
  TC.col(1) = (TC.col(1).array() - min_y)/(max_y-min_y)*-1.0+1.0;
  display_list_damage = true;
  return true;
}

bool Skinning::initialize_depth_offsets()
{
  V_depth_offsets.resize(0,0);
  if(V.rows() != OW.rows() || OW.cols() < 1)
  {
    return false;
  }
  if(!is_planar(V))
  {
    return false;
  }

  V_depth_offsets = V;
  for(int i = 0;i<OW.rows();i++)
  {
    for(int j = 0;j<OW.cols();j++)
    {
      if(reverse_depth_offsets)
      {
        V_depth_offsets(i,2) +=  OW(i,j)*-double(j)*100;
      }else
      {
        V_depth_offsets(i,2) +=  OW(i,j)*double(j)*100;
      }
    }
  }
  //V_depth_offsets.col(2) = OW.rowwise().maxCoeff();
  view_xy_plane(this);
  angle = 0;

  return true;
}

void Skinning::initialize_rotation_animation()
{
  spinning_about_up_axis = false;
  // 10 seconds per spin
  spin_period = 10.0;
}

void Skinning::update_rotation_animation()
{
  if(spinning_about_up_axis)
  {
    spin_about_up_axis();
  }
}

void Skinning::spin_about_up_axis()
{
  // Get conjugate (inverse) of current rotation
  float rotation_conj[4];
  quat_conjugate(spin_start_rotation,rotation_conj);
  // Get current up axis
  float up[3] = {0,1,0};
  rotate_by_quat(up,rotation_conj,up);
  // Spin around up axis 
  // Get angle as function of time elapsed since start
  float angle = ((get_seconds()-spin_start_seconds)/spin_period)*2.0*PI;
  // Get rotation around axis by angle as quaterion
  float spin[4];
  axis_angle_to_quat<float>(up,angle,spin);
  // rotate original rotation by this spin, place into current rotation
  quat_mult<float>(spin_start_rotation,spin,rotation);
}

void Skinning::initialize_shaders()
{
  // Initialize vertex attribute map
  shader_attribs[WEIGHTCOLORS][WEIGHTS0_ATTRIBUTE_NAME] = WEIGHTS0_ATTRIBUTE_LOCATION;
  shader_attribs[WEIGHTCOLORS][WEIGHTS4_ATTRIBUTE_NAME] = WEIGHTS0_ATTRIBUTE_LOCATION+1;
  //shader_attribs[WEIGHTCOLORS][WEIGHTS8_ATTRIBUTE_NAME] = WEIGHTS0_ATTRIBUTE_LOCATION+2;
  //shader_attribs[WEIGHTCOLORS][WEIGHTS12_ATTRIBUTE_NAME] = WEIGHTS0_ATTRIBUTE_LOCATION+3;
  shader_attribs[WEIGHTCOLORS][WEIGHT_INDICES0_ATTRIBUTE_NAME] = 
    WEIGHT_INDICES0_ATTRIBUTE_LOCATION;
  shader_attribs[WEIGHTCOLORS][WEIGHT_INDICES4_ATTRIBUTE_NAME] = 
    WEIGHT_INDICES0_ATTRIBUTE_LOCATION+1;
  //shader_attribs[WEIGHTCOLORS][WEIGHT_INDICES8_ATTRIBUTE_NAME] = 
  //  WEIGHT_INDICES0_ATTRIBUTE_LOCATION+2;
  //shader_attribs[WEIGHTCOLORS][WEIGHT_INDICES12_ATTRIBUTE_NAME] = 
  //  WEIGHT_INDICES0_ATTRIBUTE_LOCATION+3;

  shader_attribs[SCALAR][WEIGHTS0_ATTRIBUTE_NAME] = WEIGHTS0_ATTRIBUTE_LOCATION;
  shader_attribs[SCALAR][WEIGHTS4_ATTRIBUTE_NAME] = WEIGHTS0_ATTRIBUTE_LOCATION+1;
  shader_attribs[SCALAR][WEIGHTS8_ATTRIBUTE_NAME] = WEIGHTS0_ATTRIBUTE_LOCATION+2;
  shader_attribs[SCALAR][WEIGHTS12_ATTRIBUTE_NAME] = WEIGHTS0_ATTRIBUTE_LOCATION+3;
  shader_attribs[SCALAR][WEIGHT_INDICES0_ATTRIBUTE_NAME] = 
    WEIGHT_INDICES0_ATTRIBUTE_LOCATION;
  shader_attribs[SCALAR][WEIGHT_INDICES4_ATTRIBUTE_NAME] = 
    WEIGHT_INDICES0_ATTRIBUTE_LOCATION+1;
  shader_attribs[SCALAR][WEIGHT_INDICES8_ATTRIBUTE_NAME] = 
    WEIGHT_INDICES0_ATTRIBUTE_LOCATION+2;
  shader_attribs[SCALAR][WEIGHT_INDICES12_ATTRIBUTE_NAME] = 
    WEIGHT_INDICES0_ATTRIBUTE_LOCATION+3;

  shader_attribs[LBS][WEIGHTS0_ATTRIBUTE_NAME] = WEIGHTS0_ATTRIBUTE_LOCATION;
  shader_attribs[LBS][WEIGHTS4_ATTRIBUTE_NAME] = WEIGHTS0_ATTRIBUTE_LOCATION+1;
  shader_attribs[LBS][WEIGHTS8_ATTRIBUTE_NAME] = WEIGHTS0_ATTRIBUTE_LOCATION+2;
  shader_attribs[LBS][WEIGHTS12_ATTRIBUTE_NAME] = WEIGHTS0_ATTRIBUTE_LOCATION+3;
  shader_attribs[LBS][WEIGHT_INDICES0_ATTRIBUTE_NAME] = 
    WEIGHT_INDICES0_ATTRIBUTE_LOCATION;
  shader_attribs[LBS][WEIGHT_INDICES4_ATTRIBUTE_NAME] = 
    WEIGHT_INDICES0_ATTRIBUTE_LOCATION+1;
  shader_attribs[LBS][WEIGHT_INDICES8_ATTRIBUTE_NAME] = 
    WEIGHT_INDICES0_ATTRIBUTE_LOCATION+2;
  shader_attribs[LBS][WEIGHT_INDICES12_ATTRIBUTE_NAME] = 
    WEIGHT_INDICES0_ATTRIBUTE_LOCATION+3;

  // NULL shader
  shader_programs[NO_SHADER] = 0;
  bool success = true;
  success &= load_shader_pair_from_files(DIRECTIONAL_PER_PIXEL, SHADER_DIR "/directionalperpixel.frag");
  success &= load_shader_pair_from_files(DIRECTIONAL_PER_PIXEL_COLOR, SHADER_DIR "/directionalperpixelcolor.frag");
  success &= load_shader_pair_from_files(WEIGHTCOLORS, SHADER_DIR "/weightcolors.frag");
  success &= load_shader_pair_from_files(SCALAR, SHADER_DIR "/scalar.frag");
#ifdef __APPLE__
  success &= load_shader_pair_from_files(LBS, SHADER_DIR "/lbs.frag");
#else
  success &= load_shader_pair_from_files(LBS, SHADER_DIR "/lbsOptimized.frag");
#endif
  if(!success)
  {
    verbose("Error: initialize_shaders() couldn't load all shaders.\n");
  }
}

void Skinning::deinitialize_shaders()
{
  // NULL shader
  shader_programs[NO_SHADER] = 0;
  bool success = true;
  // destroy shaders
  success &=
    destroy_shader_program(shader_programs[DIRECTIONAL_PER_PIXEL]);
  success &=
    destroy_shader_program(shader_programs[DIRECTIONAL_PER_PIXEL_COLOR]);
  success &= destroy_shader_program(shader_programs[WEIGHTCOLORS]);
  success &= destroy_shader_program(shader_programs[SCALAR]);
  success &= destroy_shader_program(shader_programs[LBS]);
  if(!success)
  {
    verbose("Error: initialize_shaders() couldn't load all shaders.\n");
  }
}

bool Skinning::validate_for_shader()
{
  vector<string> errors;

  switch(shader_mode)
  {
    case WEIGHTCOLORS:
    case SCALAR:
    case LBS:
    {
      if(W.rows() != V.rows())
      {
        errors.push_back("Number of weights rows != number of vertex rows");
      }
      // Fall through to DIRECTIONAL_PER_PIXEL_COLOR
    }
    case DIRECTIONAL_PER_PIXEL_COLOR:
    {
      if(C.rows() != V.rows())
      {
        errors.push_back("Number of color rows != number of vertex rows");
      }
      if(C.cols() != 3)
      {
        errors.push_back("Number of color cols != 3");
      }
      
      break;
    }
    default:
      break;
  }

  if(!errors.empty())
  {
    cerr<<"ERROR: validate_for_shader():"<<endl;
    for(
      vector<string>::iterator eit = errors.begin();
      eit != errors.end();
      eit++)
    {
      cerr<<(*eit)<<endl;
    }
  }

  return errors.empty();
}

bool Skinning::load_shader_pair_from_files(
  const ShaderMode idx,
  const std::string shader_file_name)
{
  // split give file name into pieces 
  string dir,base,ext,prefix;
  pathinfo(shader_file_name,dir,base,ext,prefix);
  // Throw a warning if the extension is weird
  if(ext != "vert" && ext != "frag")
  {
    verbose(
      "WARNING: load_from_file_pair():"
      " %s ends in niether 'vert' nor 'frag'\n",
      shader_file_name.c_str());
  }
  // reassemble into vert and frag filenames
  string vert_file_name = dir + "/" + prefix + ".vert";
  string frag_file_name = dir + "/" + prefix + ".frag";
  // Suck in vert shader source from file
  string vert_source = "";
  if(is_readable(vert_file_name.c_str()))
  {
    bool vert_success = file_contents_as_string(vert_file_name,vert_source);
    if(!vert_success)
    {
      return false;
    }
  }

  // Suck in frag shader source from file
  string frag_source = "";
  if(is_readable(frag_file_name.c_str()))
  {
    bool frag_success = file_contents_as_string(frag_file_name,frag_source);
    if(!frag_success)
    {
      return false;
    }
  }

  bool creation_success = 
    create_shader_program(
      vert_source,
      frag_source,
      shader_attribs[idx],
      shader_programs[idx]);
  if(!creation_success)
  {
    return false;
  }
  // If we've succesfullly loaded the LBS shader then switch to that mode
  if(idx == LBS)
  {
    shader_mode = LBS;
    display_list_damage = true;
  }
  return true;
}

bool Skinning::transformations()
{

  // Compute transformations into T
  bool gather_success = gather_transformations(BR,dial_in_each_T,T);
  if(!gather_success)
  {
    return false;
  }

  if(auto_dof && !bypass_auto_dof)
  {
    // number of handles
    int m = arap_dof.m;
    if(m != T.cols()/4)
    {
      fprintf(stderr,
        "Error: transformations()" 
        "number of handles in T (%d) doesn't match that in arap_dof data (%d)\n" 
        "you must reinitialize.\n",(int)(T.cols()/4),m);
      return (auto_dof = false);
    }
    // number of dimensions
    int dim = arap_dof.dim;
    Matrix<double,Dynamic,1> B_eq;
    gather_positional_constraints_rhs(BR,m,dim,B_eq);
    // Gather transformations into a column
    MatrixXf Tcol;
    columnize<float,Eigen::Dynamic>(T.block(0,0,dim,T.cols()).eval(),m,2,Tcol);
    // use last solution as initial guess
    MatrixXd L0 = L;
    if(arap_dof.fixed_dim.size() > 0)
    {
      // But also place transformations of fixed handles into initial guess
      Eigen::Matrix<int,Eigen::Dynamic,1> zero(1); 
      zero << 0;
      MatrixXf Tcol_fixed_dim; 
      slice(Tcol,arap_dof.fixed_dim,zero,Tcol_fixed_dim);
      slice_into(Tcol_fixed_dim.cast<double>().eval(),arap_dof.fixed_dim,zero,L0);
#ifdef EXTREME_VERBOSE
      MatrixXd L0stack;
      uncolumnize<double,Dynamic>(L0,dim,dim+1,2,L0stack);
      cout<<"L0stack=["<<endl<<L0stack<<endl<<"];"<<endl;
#endif
      assert(L0.size() == L.size());
      assert(L0.rows() == L.rows());
      assert(L0.cols() == L.cols());
    }
    bool update_success = arap_dof_update(arap_dof,B_eq,L0,max_iters,tol,L);
    if(!update_success)
    {
      return false;
    }
#ifdef EXTREME_VERBOSE
    cout<<"L=["<<endl<<L<<endl<<"];"<<endl;
#endif
    // Set z-scale to 1
    if(arap_dof.effective_dim == 2 && dim == 3)
    {
      L.block(L.rows()-(dim+1)*arap_dof.m,0,arap_dof.m,1).array() *= 0;
      L.block(L.rows()-(dim+1)*arap_dof.m,0,arap_dof.m,1).array() += 1;
    }
    MatrixXd Lstack;
    uncolumnize<double,Dynamic>(L,dim,dim+1,2,Lstack);
#ifdef EXTREME_VERBOSE
    cout<<"Lstack=["<<endl<<Lstack<<endl<<"];"<<endl;
#endif
    assert(T.rows() >= Lstack.rows());
    assert(T.cols() == Lstack.cols());
    T.block(0,0,Lstack.rows(),Lstack.cols()) = Lstack.cast<float>();
#ifdef EXTREME_VERBOSE
    cout<<"T=["<<endl;
    for(int i = 0;i<T.cols()/4;i++)
    {
      cout<<endl<<T.block(0,i*4,T.rows(),4)<<endl;
    }
    cout<<"];"<<endl;
#endif
  }

  if(!dial_in_each_T)
  {
    distribute_transformations(T,BR);
  }
  return true;
}

bool Skinning::send_transformations()
{
  const int shader_id = shader_programs[LBS];
  // Number of handles according to weights index matrix
  //const GLint num_handles = T.rows()/HANDLE_TRANSFORMATION_ROWS;
  const GLint num_handles = T.cols()/4;
  // Find NUM_HANDLES_NAME in shader uniforms map
  GLint num_handles_location = 
    glGetUniformLocation(
      shader_id,
      NUM_HANDLES_NAME);
  if(num_handles_location == -1)
  {
    fprintf(
      stderr,
      "ERROR: send_transformations() could not set up uniform: %s\n",
      NUM_HANDLES_NAME);
    return false;
  }else
  {
    glUniform1i(num_handles_location,num_handles);
    GLenum err = report_gl_error("transformations(num_handles): ");
    if(err != GL_NO_ERROR)
    {
      return false;
    }
  }

  // Find "handle_transformation" in shader uniforms map
  GLint handle_transformation_location = 
    glGetUniformLocation(
      shader_id,
      HANDLE_TRANSFORMATION_NAME);
  if(handle_transformation_location == -1)
  {
    fprintf(
      stderr,
      "ERROR: send_transformations() could not set up uniform: %s\n",
      HANDLE_TRANSFORMATION_NAME);
    return false;
  }else
  {
    //const GLsizei count = T.rows()/HANDLE_TRANSFORMATION_ROWS;
    const GLsizei count = T.cols()/4;
#ifdef MAT3x4
    // Flip each matrix
    Eigen::MatrixXf TF;
    transpose_blocks(T,count,2,TF);
    eigen_glUniformMatrixXfv(handle_transformation_location,count,TF);
#else
    eigen_glUniformMatrixXfv(handle_transformation_location,count,T);
#endif
  }
  return true;
}

bool Skinning::load_weights(const string weights_file_name)
{
  // read matrix from DMAT file type
  bool read_success = readDMAT(weights_file_name,OW);
  if(!read_success)
  {
    OW.resize(0,0);
    W.resize(0,0);
    WI.resize(0,0);
    return false;
  }
  if(OW.rows()!=EW.rows())
  {
    EW.resize(OW.rows(),0);
  }
  initialize_depth_offsets();
  return initialize_weights();
}

bool Skinning::load_extra_weights(const string weights_file_name)
{
  // read matrix from DMAT file type
  bool read_success = readDMAT(weights_file_name,EW);
  if(!read_success)
  {
    return false;
  }
  if(EW.rows()!=OW.rows())
  {
    OW.resize(EW.rows(),0);
  }
  return initialize_weights();
}

bool Skinning::save_extra_weights(const string weights_file_name)
{
  // write EW to DMAT
  if(EW.size() == 0)
  {
    fprintf(stderr,
      "Error: save_extra_weights(): can't print EW because it is empty...\n");
    return false;
  }
  bool write_success = writeDMAT(weights_file_name,EW);
  return write_success;
}

bool Skinning::initialize_weights()
{
  assert( (OW.cols()+EW.cols()) <= MAX_NUM_HANDLES);
  if(OW.rows() != V.rows())
  {
    verbose(
      "Error: OW.rows() (%d) != V.rows() (%d)\n"
      "  Setting OW to []\n",
      OW.rows(),
      V.rows());
    OW.resize(0,0);
    return false;
  }
  region_colors_from_weights(OW,F,region_color,mat_diffuse,RC);
  pseudocolors_from_weights(OW,C);

  if((OW.cols()+EW.cols()) > MAX_NUM_WEIGHTS_PER_VERTEX)
  {
    verbose(
      "WARNING: too many weights (%d) per mesh vertex."
      " Keeping only %d most significant.\n",
      OW.cols()+EW.cols(),
      MAX_NUM_WEIGHTS_PER_VERTEX);
  }
  cout<<"size(OW) = "<<OW.rows()<<" "<<OW.cols()<<endl;
  cout<<"size(EW) = "<<EW.rows()<<" "<<EW.cols()<<endl;
  sort_weights(
    OW,EW,
    MAX_NUM_WEIGHTS_PER_VERTEX,
    pow(10,sort_weights_epsilon_power),
    W,WI);
#ifdef EXTREME_VERBOSE
  cout<<"W=["<<endl<<W<<endl<<"];"<<endl;
  cout<<"WI=["<<endl<<WI<<endl<<"];"<<endl;
#endif

  if((OW.cols()+EW.cols()) > MAX_NUM_HANDLES)
  {
    verbose(
      "------------------------------------------------------------\n-\n-\n"
      "WARNING: too many handles (%d>%d).\n"
      "-\n-\n------------------------------------------------------------\n",
      OW.cols()+EW.cols(),
      MAX_NUM_HANDLES);
  }
  cout<<"size(OW) = "<<OW.rows()<<" "<<OW.cols()<<endl;
  cout<<"size(EW) = "<<EW.rows()<<" "<<EW.cols()<<endl;


  // Need to reinitialize transformations if there was a change in the number
  // of handles
  if((OW.cols()+EW.cols()) != T.cols()/HANDLE_TRANSFORMATION_ROWS)
  {
    initialize_transformations();
  }


  if(W.size() != 0 && WI.size() != 0)
  {
    verbose("lbs_matrix()\n");
    //lbs_matrix(V,W,WI,M);
    MatrixXd OEW(OW.rows(),OW.cols()+EW.cols());
    if(EW.size() == 0)
    {
      OEW<<OW;
    }else
    {
      OEW<<OW,EW;
    }
    lbs_matrix(V,OEW,M);
  }

  // be sure display (and also LBS) is updated
  display_list_damage = true;

  return true;
}

bool Skinning::initialize_transformations()
{
  if(WI.size() == 0)
  {
    fprintf(stderr,"Error: initialize_transformations() WI empty\n");
    return false;
  }
  const int num_handles = WI.maxCoeff()+1;
  verbose("Initializing %d transformations to identities\n",num_handles);

  // Stack handle transformations horizontallly
  // Initialize all entries to zero
  T.setZero(HANDLE_TRANSFORMATION_ROWS,num_handles*4);

  return true;
}

bool Skinning::load_bone_roots(
  const std::string bone_roots_file_name, 
  const bool must_match,
  const bool no_recompute)
{
  // read matrix from BF file type
  vector<Bone*> new_BR;
  bool read_success = read_BF(bone_roots_file_name.c_str(),new_BR);
  // make sure drawing according to last_T is set to match for all bones
  // Check that trees match
  if(bone_roots_match(new_BR,BR))
  {
    verbose("Bones match... Just loading data into existing bones...\n");
    // If bone forests match then just load data (transformations) into
    // existing bones
    copy_bone_roots_data(new_BR,BR);
    if(auto_dof && !no_recompute)
    {
      reinitialize_auto_dof();
    }
  }else if(must_match)
  {
    verbose("Bones don't match...\n");
    return false;
  }else
  {
    destroy_bone_roots(BR);
    BR = new_BR;
    vector<Bone *> B = gather_bones(BR);
    for_each(B.begin(),B.end(),
      bind2nd(
        mem_fun(&Bone::set_draw_according_to_last_T),
        draw_bones_according_to_T));
    for_each(B.begin(),B.end(),
      bind2nd(
        mem_fun(&Bone::set_draw_connected_to_parent),
        draw_connected_skeleton));
    for_each(B.begin(),B.end(),
      bind2nd(
        mem_fun(&Bone::set_average_children_tails_to_draw_non_weighted_roots),
        average_children_tails_to_draw_non_weighted_roots));
    for_each(B.begin(),B.end(),
      bind2nd(mem_fun(&Bone::set_view_vector),view_vector));

    //clear animation stack
    animation.clear();
  }
  // reinitialize transformations
  initialize_transformations();
  return read_success;
}

bool Skinning::load_tgf_and_dmat_pair(
  const std::string tgf_file_name, 
  const std::string dmat_file_name)
{
  cout<<"Loading from lbs.app data: "<<tgf_file_name<<" & "<<dmat_file_name<<endl;
  vector<Bone* > new_BR;
  MatrixXd new_OW;
  bool load_success = load_tgf_and_dmat_pair_as_bone_roots_and_weights(
    tgf_file_name,dmat_file_name,new_BR,new_OW);
  if(!load_success)
  {
    return false;
  }
  // Immediately write each to a file
  string new_bf_file_name = 
    append_to_filename_before_extension(tgf_file_name,"-skinning","bf");
  cout<<"Writing bone roots to: "<<new_bf_file_name<<endl;
  bool bone_roots_saved = write_BF(new_bf_file_name.c_str(),new_BR);
  if(!bone_roots_saved)
  {
    return false;
  }
  string new_dmat_file_name = 
    append_to_filename_before_extension(dmat_file_name,"-skinning","dmat");
  cout<<"Writing weights to: "<<new_dmat_file_name<<endl;
  bool weights_saved = writeDMAT(new_dmat_file_name.c_str(),new_OW);;
  if(!weights_saved)
  {
    return false;
  }
  // Finally load bone roots and weights from those files
  bool bone_roots_loaded = load_bone_roots(new_bf_file_name);
  if(!bone_roots_loaded)
  {
    return false;
  }
  bool weights_loaded = load_weights(new_dmat_file_name);
  if(!weights_loaded)
  {
    return false;
  }
  return true;
}

bool Skinning::load_bone_roots_animation(const std::string bone_roots_file_name)
{
  // clear the current animation stack
  animation.clear();
  // first load the given bone roots "central" pose
  load_bone_roots(bone_roots_file_name);

  // push current BR as animation frame
  animation.push_back(
    KeyFrame<BoneBoneCopyMap>(
      BoneBoneCopyMap(BR,false),
      animation_interp_secs,
      transition_type));

  // determine basename and extension of file
  string dirname, basename, extension, filename;
  pathinfo(bone_roots_file_name,dirname,basename,extension,filename);
  int i = 1;
  while(true)
  {
    stringstream next_name;
    next_name << dirname << "/" << filename << "-" << i << "." << extension;
    verbose("next_name: %s\n",next_name.str().c_str());
    // check that this file exists
    if(is_readable(next_name.str().c_str()))
    {
      // try to load this file as bone roots
      bool load_success = load_bone_roots(next_name.str().c_str(),true,true);
      if(!load_success)
      {
        return false;
      }
      // push current BR as animation frame
      animation.push_back(
        KeyFrame<BoneBoneCopyMap>(
          BoneBoneCopyMap(BR,false),
          animation_interp_secs,
          transition_type));
    }else
    {
      // file didn't exist so we've read all available for animation
      break;
    }
    i++;
  }
  return true;
}

bool Skinning::save_bone_roots(const string bone_roots_file_name)
{
  // Try to save bones
  verbose("Saving bone forest...");
  bool bone_roots_saved = write_BF(bone_roots_file_name.c_str(),BR);
  verbose("%s\n",(bone_roots_saved?"succeeded":"failed"));
  return bone_roots_saved;
}

bool Skinning::save_bone_roots_animation(const string bone_roots_file_name)
{
  // determine basename and extension of file
  string dirname, basename, extension, filename;
  pathinfo(bone_roots_file_name,dirname,basename,extension,filename);
  verbose("Saving bone forest animation...");
  bool bone_roots_saved = true;
  // Play through animation stack and save each pose
  for(int i = 0;i<(int)animation.size();i++)
  {
    // Use lerp to load this pose into current BR
    lerp(animation[i].state,animation[i].state,0);
    stringstream name;
    if(i == 0)
    {
      // First frame gets input name
      name << bone_roots_file_name;
    }else
    {
      name << dirname << "/" << filename << "-" << i << "." << extension;
    }
    verbose("name[%d]: %s\n",i,name.str().c_str());
    bone_roots_saved &= write_BF(name.str().c_str(),BR);
    if(!bone_roots_saved)
    {
      break;
    }
  }
  verbose("%s\n",(bone_roots_saved?"succeeded":"failed"));
  return bone_roots_saved;
}

const Eigen::MatrixXi & Skinning::elements()
{
  // Decide if solving over volume (tets) or surface (triangles)
  // Indices of elements over which we're solving, either F or T
  if(Tets.size() == 0)
  {
      verbose("Solving over surface...\n");
      return F;
  }else{
      verbose("Solving over volume...\n");
      return Tets;
  }
}

bool Skinning::compute_extra_weights()
{
  MatrixXd * effective_OW;
  MatrixXd filtered_OW;
  if(num_abrupt_weight_space > 0)
  {
    filtered_OW = OW;
    for(int i = 0; i < num_abrupt_weight_space; i++)
    {
      filtered_OW = (filtered_OW.array().pow(3)*2 - filtered_OW.array().pow(2)*3 + filtered_OW.array()*2).eval();
    }
    effective_OW = &filtered_OW;
  }else
  {
    effective_OW = &OW;
  }

  switch(ew_type)
  {
    case EXTRAWEIGHT_PROJECT_COMPACT:
      project_extra_compact(*effective_OW,num_extra_weights,push,EW);
      break;
    case EXTRAWEIGHT_ISO:
      iso_extra(*effective_OW,elements(),num_extra_weights,push,ease,EW);
      break;
    default:
      fprintf(stderr,
        "initialize_auto_dof: Error: unsupported extra weight type %d\n",
        ew_type);
      return false;
  }
  if(num_extra_weights != EW.cols())
  {
    verbose("Warning: EW.cols() (%d) != num_extra_weights (%d)\n",
      EW.cols(),num_extra_weights);
  }
#ifdef EXTREME_VERBOSE
  cout<<"EW=["<<std::endl<<EW<<std::endl<<"];"<<endl;
#endif
  return true;
}

bool Skinning::initialize_auto_dof()
{
  // Default is to not accept
  auto_dof = false;

  // TODO: should verify that everything's already initialized and if so just
  // return true and continue


  // Compute extra weights into TW
  // resort W,WI
  if(OW.cols() > 0 && num_extra_weights > 0)
  {
    bool success = compute_extra_weights();
    if(!success)
    {
      return false;
    }
    initialize_weights();
  }

  // Gather original weights and extra weights
  Eigen::MatrixXd TW;
  assert(OW.rows() == EW.rows());
  // TW = [OW EW]
  TW.resize(OW.rows(),OW.cols() + EW.cols());
  TW.block(0,0,OW.rows(),OW.cols()) = OW;
  TW.block(0,OW.cols(),EW.rows(),EW.cols()) = EW;

  if(WI.size() == 0)
  {
    fprintf(stderr,"Error: initialize_auto_dof() WI empty\n");
    return false;
  }
  if(W.size() == 0)
  {
    fprintf(stderr,"Error: initialize_auto_dof() W empty\n");
    return false;
  }

  // Reinitialize transformations
  bool T_ready = initialize_transformations();
  if(!T_ready)
  {
    return false;
  }

  // number of handles
  const int num_handles = T.cols()/4;
  // number f dimensions
  const int dim = V.cols();
  // Gather rest positions
  MatrixXd C(num_handles,dim);
  bool gather_success = gather_rest_positions(BR,C);
#ifdef EXTREME_VERBOSE
  cout<<"C=["<<C<<"];"<<endl;
#endif
  if(!gather_success)
  {
    return false;
  }

  // Construct lbs matrix
#ifdef EXTREME_VERBOSE
  cout<<"MIJV=["<<endl;print_ijv(M,1);cout<<endl<<"];"<<
    endl<<"M=sparse(MIJV(:,1),MIJV(:,2),MIJV(:,3),"<<
    M.rows()<<","<<M.cols()<<");"<<endl;
#endif

  // Partition into groups
  int k = max(num_handles+1,min((int)V.rows(),num_groups));
  if(k!=num_groups)
  {
    fprintf(stderr,
      "Warning: initialize_auto_dof() num_groups (%d)"
      " invalid, setting to (%d)\n",
      num_groups,
      k);
    num_groups = k;
  }
  Matrix<int,Dynamic,1> G;
  Matrix<int,Dynamic,1> S;
  Matrix<double,Dynamic,1> GD;
  // Use original weights for partitioning
  verbose("partition()\n");
  partition(TW,k,G,S,GD);
#ifdef EXTREME_VERBOSE
  cout<<"S=["<<endl<<S<<endl<<"]+1;"<<endl;
  cout<<"GD=["<<endl<<GD<<endl<<"];"<<endl;
  cout<<"G=["<<endl<<G<<endl<<"]+1;"<<endl;
#endif

  // List of fixed *weight functions*, handles or bones are not fixed. their
  // weight functions are fixed
  Eigen::Matrix<int,Eigen::Dynamic,1> fixed(0,1);
  // List of free *weight functions*
  Eigen::Matrix<int,Eigen::Dynamic,1> free(0,1);
  gather_free(BR,TW.cols(),free);
  gather_fixed(BR,TW.cols(),fixed);
  // test for intersection
  if(!intersection(free,fixed).size() == 0)
  {
    fprintf(stderr,
      "Error: initialize_auto_dof() free ∩ fixed should be empty\n");
    return false;
  }
#ifdef EXTREME_VERBOSE
  cout<<"free=["<<endl<<free<<endl<<"]+1;"<<endl;
  cout<<"fixed=["<<endl<<fixed<<endl<<"]+1;"<<endl;
#endif
  // Linear positional constraints system
  verbose("gather_positional_constraints_system()\n");
  SparseMatrix<double> A_eq, A_fix_eq;
  gather_positional_constraints_system(BR,num_handles,dim,A_eq);
  gather_fixed_constraints_system(fixed, dim, num_handles, A_fix_eq);
  SparseMatrix<double> A_eq_merged;
  join_constraints_systems(A_eq, A_fix_eq, A_eq_merged);

  verbose("arap_dof_precomputation()\n");
  // Arap auto dof precomputation
  double before = get_seconds();
  arap_dof_precomputation(V,elements(),M,G,fixed,A_eq_merged,arap_dof);
  verbose("Precomputation time: %g\n",get_seconds()-before);
  
  // Initialize initial guess to identities, except free handles get zeros
  MatrixXd I = MatrixXd::Identity(dim,dim+1);
#ifdef EXTREME_VERBOSE
  cout<<"I=["<<endl<<I<<endl<<"];"<<endl;
#endif
  MatrixXd IGstack;
  repmat(I,1,num_handles,IGstack);
  // set free handles' initial guesses to zeros
  for(int i = 0;i<free.size();i++)
  {
    IGstack.block(0,free(i)*(dim+1),dim,dim+1) *= 0;
  }
#ifdef EXTREME_VERBOSE
  cout<<"IGstack=["<<endl<<IGstack<<endl<<"];"<<endl;
#endif
  columnize<double,Dynamic>(IGstack,num_handles,2,L);
#ifdef EXTREME_VERBOSE
  cout<<"L=["<<endl<<L<<endl<<"];"<<endl;
#endif

  // return true and accept
  return auto_dof = true;
}

bool Skinning::reinitialize_auto_dof()
{
  // Default is to not accept
  auto_dof = false;

  // number of handles
  const int num_handles = T.cols()/4;
  // number f dimensions
  const int dim = V.cols();
  // Gather rest positions
  MatrixXd C(num_handles,dim);
  bool gather_success = gather_rest_positions(BR,C);
#ifdef EXTREME_VERBOSE
  cout<<"C=["<<C<<"];"<<endl;
#endif
  if(!gather_success)
  {
    return false;
  }

  // List of fixed *weight functions*, handles or bones are not fixed. their
  // weight functions are fixed
  Eigen::Matrix<int,Eigen::Dynamic,1> fixed(0,1);
  // List of free *weight functions*
  Eigen::Matrix<int,Eigen::Dynamic,1> free(0,1);
  gather_free(BR, (OW.cols() + EW.cols()),free);
  gather_fixed(BR,(OW.cols() + EW.cols()),fixed);
  // test for intersection
  if(!intersection(free,fixed).size() == 0)
  {
    fprintf(stderr,
      "Error: reinitialize_auto_dof() free ∩ fixed should be empty\n");
    return false;
  }

#ifdef EXTREME_VERBOSE
  cout<<"free=["<<endl<<free<<endl<<"]+1;"<<endl;
  cout<<"fixed=["<<endl<<fixed<<endl<<"]+1;"<<endl;
#endif
  // Linear positional constraints system
  verbose("gather_positional_constraints_system()\n");
  SparseMatrix<double> A_eq, A_fix_eq;
  gather_positional_constraints_system(BR,num_handles,dim,A_eq);
  gather_fixed_constraints_system(fixed, dim, num_handles, A_fix_eq);
  SparseMatrix<double> A_eq_merged;
  join_constraints_systems(A_eq, A_fix_eq, A_eq_merged);

#ifdef EXTREME_VERBOSE
  cout << "A_eq_merged=[" << endl << A_eq_merged << "]+1;" << endl;
#endif

  verbose("arap_dof_recomputation()\n");
  // Arap auto dof precomputation
  arap_dof_recomputation(fixed,A_eq_merged,arap_dof);

  // return true and accept
  return auto_dof = true;
}

bool Skinning::update_animation(double t)
{  
  // indices of current keyframes
  size_t a,b;
  // factor from keyframe a to b where at
  double f;
  bool still_animated = animation.get_frame(t,a,b,f);
  // filter based on transitions
  f = Animation<BoneBoneCopyMap>::filter( 
    animation[a].transition,animation[b].transition,f);
  // interpolate bones
  lerp(animation[a].state,animation[b].state,f);
  // this stores the result into Skinning::BR (pointers to BR are setup in
  // _each_ keyframe in "animation")

  return still_animated;
}

bool Skinning::start_animating()
{
  bool was_animating = animating;
  if(was_animating)
  {
    verbose("Cannot start animation as we are already animating...\n");
    return false;
  }

  // If animation stack is empty then make a phony animation
  phony_animation = animation.size() == 0;

  if(phony_animation)
  {
    animation.push_back(
      KeyFrame<BoneBoneCopyMap>(
        BoneBoneCopyMap(BR,false),
        animation_interp_secs,
        transition_type));
    animation.push_back(
      KeyFrame<BoneBoneCopyMap>(
        BoneBoneCopyMap(BR,true),
        animation_interp_secs,
        transition_type));
  }
  // Always add current frame to stack (it will be popped when animation is
  // stopped)
  animation.push_back(
    KeyFrame<BoneBoneCopyMap>(
      BoneBoneCopyMap(BR,false),
      0,
      transition_type));

  animation_start_seconds = get_seconds();
  animating = true;

  return animating;
}

bool Skinning::stop_animating()
{
  bool was_animating = animating;
  if(!was_animating)
  {
    verbose("Cannot stop animation as we are not animating...\n");
    return false;
  }

  // better be animating
  assert(animating);
  // stop animating
  animating = false;

  assert(animation.size() >= 1);
  // Pop last frame
  animation.pop_back();
  if(phony_animation)
  {
    assert(animation.size() == 2);
    // pop remaining 2
    animation.pop_back();
    animation.pop_back();
  }

  // animation better be stopped
  assert(animating == false);
  return animating;
}
