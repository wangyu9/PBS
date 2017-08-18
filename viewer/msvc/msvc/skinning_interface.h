#define IGL_HEADER_ONLY

#include <igl/pathinfo.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/readMESH.h>
#include <igl/tetgen/mesh_with_skeleton.h>
#include <igl/faces_first.h>
#include <igl/readTGF.h>
#include <igl/launch_medit.h>
#include <igl/boundary_conditions.h>
//#include <igl/mosek/bbw.h>
#include <igl/writeDMAT.h>
#include <igl/writeMESH.h>

#include <Eigen/Dense>

#include <iostream>
#include <string>

// Whether medit program is install
//#define WITH_MEDIT



// Read a surface mesh from a {.obj|.off|.mesh} files
// Inputs:
//   mesh_filename  path to {.obj|.off|.mesh} file
// Outputs:
//   V  #V by 3 list of mesh vertex positions
//   F  #F by 3 list of triangle indices
// Returns true only if successfuly able to read file
bool load_mesh_from_file(
	const std::string mesh_filename,
	Eigen::MatrixXd & V,
	Eigen::MatrixXi & F);

// Load a skeleton (bones, points and cage edges) from a {.bf|.tgf} file
//
// Inputs:
//   skel_filename  path to skeleton {.bf|.tgf} file
// Outputs:
//  C  # vertices by 3 list of vertex positions
//  P  # point-handles list of point handle indices
//  BE # bone-edges by 2 list of bone-edge indices
//  CE # cage-edges by 2 list of cage-edge indices
bool load_skeleton_from_file(
	const std::string skel_filename,
	Eigen::MatrixXd & C,
	Eigen::VectorXi & P,
	Eigen::MatrixXi & BE,
	Eigen::MatrixXi & CE);

// Writes output files to /path/to/input/mesh-skeleton.dmat,
// mesh-volume.dmat, mesh-volume.mesh if input mesh was
// located at /path/to/input/mesh.obj and input skeleton was at
// /other/path/to/input/skel.tgf
// 
// Writes:
////   mesh.dmat  dense weights matrix corresponding to original input
////     vertices V
//   mesh-volume.dmat  dense weights matrix corresponding to all
//     vertices in tet mesh used for computation VV
//   mesh-volume.mesh  Tet mesh used for computation
//
// Inputs:
//   mesh_filename  path to {.obj|.off|.mesh} file
//   skel_filename  path to skeleton {.bf|.tgf} file
//   V  #V by 3 list of original mesh vertex positions
//   F  #F by 3 list of original triangle indices
//   VV  #VV by 3 list of tet-mesh vertex positions
//   TT  #TT by 4 list of tetrahedra indices
//   FF  #FF by 3 list of surface triangle indices
//   W   #VV by #W weights matrix
// Returns true on success
bool save_output(
	const std::string mesh_filename,
	const std::string /*skel_filename*/,
	const Eigen::MatrixXd & V,
	const Eigen::MatrixXi & /*F*/,
	const Eigen::MatrixXd & VV,
	const Eigen::MatrixXi & TT,
	const Eigen::MatrixXi & FF,
	const Eigen::MatrixXd & W);