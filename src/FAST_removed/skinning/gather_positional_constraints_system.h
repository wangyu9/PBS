#ifndef GATHER_POSITIONAL_CONSTRAINTS_SYSTEM_H
#define GATHER_POSITIONAL_CONSTRAINTS_SYSTEM_H

#include "Bone.h"
#include <vector>
#include <iostream>

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Eigen/Dense>
#include <Eigen/Sparse>

// Gather positional constraints system entries implied by bone forest
//
// Inputs:
//   BR  list of bone tree roots
//   m  number of handles/weight functions (might be more than the number of
//     weight influencing bones)
//   dim  number of dimensions for constraints, 2 or 3
// Output:
//   Aeq  dim*#constraint_points by m*dim*(dim+1)  matrix of linear equality
//     constraint coefficients. Each row corresponds to a linear constraint, so
//     that Aeq * L = Beq says that the linear transformation entries in the
//     column L should produce the user supplied positional constraints for
//     each handle in Beq. The row Aeq(i*dim+d) corresponds to the constrain on
//     coordinate d of position i
// Returns true on success, false on error
inline bool gather_positional_constraints_system(
  const std::vector<Bone*> & BR, 
  const int m,
  const int dim,
  Eigen::SparseMatrix<double> & A_eq);

// Append constraints for a given position and influencing weight function (one
// constraint for each dimension)
//
// Inputs:
//   p  >dim position vector
//   m  number of handles/weight functions (might be more than the number of
//     weight influencing bones)
//   dim  number of dimensions for constraints, 2 or 3, if less than dimension
//     of p then first dim coordinates are used
//   dyn_A_eq  current A_eq (see above) with only num_pc non-zeros rows at the
//     top, and at least dim rows free after those
//   num_pc  current number of non-zero rows at the top of dyn_A_eq
// Outputs:
//   dyn_A_eq  updated A_eq 
//   num_pc  updated number of non-zero rows at the top of dyn_A_eq
//     (num_pc += dim)
inline void append_to_constraints_system(
  const Vec3 & p,
  const int wi,
  const int m,
  const int dim,
  Eigen::DynamicSparseMatrix<double, Eigen::RowMajor> & dyn_A_eq,
  int & num_pc);

// Implementation 

#include <igl/speye.h>
#include "DegreeOfFreedomType.h"

inline bool gather_positional_constraints_system(
  const std::vector<Bone*> & BR, 
  const int m,
  const int dim,
  Eigen::SparseMatrix<double> & A_eq)
{
  // Insert roots into search queue
  std::list<const Bone*> Q;
  for(
    std::vector<Bone*>::const_iterator bit = BR.begin();
    bit != BR.end();
    bit++)
  {
    Q.push_back(*bit);
  }

  // The maximum number of constraints per weight function is in general the
  // number of dimensions+1 (completely fixed).
  // Resize with an over estimate of the number of constraints
  Eigen::DynamicSparseMatrix<double, Eigen::RowMajor> 
    dyn_A_eq(m*dim*(dim+1),m*dim*(dim+1));
  // Should probably use some hueristic to guess how many non-zeros to reserve here

  // Current count of constraints (non zero rows in dyn_A_eq)
  int num_pc = 0;

  while(!Q.empty())
  {
    const Bone * b = Q.back();
    Q.pop_back();
    if(b->wi_is_set())
    {
      int wi = b->get_wi();
      if(wi >= m)
      {
        fprintf(
          stderr,
          "Error: gather_positional_constraints_system() max weight index of bones (%d)"
          " >= number of handles (%d)\n",
          wi,
          m);
        return false;
      }
      if(wi >= 0)
      { 
        // Add positional constraint for position constraint tips
        if(b->tip_dof_type == NORMAL_DOF)
        {
          // always add tip constraint (even for roots)
          append_to_constraints_system(b->rest_tip(),wi,m,dim,dyn_A_eq,num_pc);
        }
        // Add tail if not fixed
        if(b->tip_dof_type != FIXED_DOF)
        {
          // If tail exists and is not FREE then also add positional constraint
          // for tail
          if(!b->is_root() && b->get_parent()->tip_dof_type != FREE_DOF)
          {
            // if non root then also add constraint for tail
            append_to_constraints_system(b->get_parent()->rest_tip(),wi,m,dim,dyn_A_eq,num_pc);
          }
        }
      }
    }
    // Add children to queue
    std::vector<Bone*> children = b->get_children();
    Q.insert(Q.end(),children.begin(),children.end());
  }

  std::cout<<"num_pc = "<<num_pc<<";"<<std::endl;

  // Only keep top num_pc rows
  A_eq.resize(num_pc,m*dim*(dim+1));
  assert(A_eq.rows()<=dyn_A_eq.rows());
  assert(A_eq.cols()==dyn_A_eq.cols());
  A_eq.reserve(dyn_A_eq.nonZeros());
  Eigen::SparseMatrix<double> A_eq_temp(dyn_A_eq);
  // Loop outer level
  for(int k=0; k<A_eq_temp.outerSize(); ++k)
  {
    // loop inner level
    for(Eigen::SparseMatrix<double>::InnerIterator it(A_eq_temp,k); it; ++it)
    {
      if(it.row() < A_eq.rows() && it.col() < A_eq.cols())
      {
        A_eq.insert(it.row(),it.col()) = it.value();
      }
    }
  }
  A_eq.finalize();

  
 
  return true;
}

inline void append_to_constraints_system(
  const Vec3 & p,
  const int wi,
  const int m,
  const int dim,
  Eigen::DynamicSparseMatrix<double, Eigen::RowMajor> & dyn_A_eq,
  int & num_pc)
{
  // Loop over rows corresponding to this positional constraint
  // Also rows in transformation entries
  for(int r = 0;r<dim;r++)
  {
    // Loop over columns in transformation entries
    for(int c = 0;c<(dim+1);c++)
    {
      double v;
      // Get cth homogenous coordinate
      if(c<dim)
      {
        v = p(c);
      }else
      {
        v = 1;
      }
      // Add constraint coefficient
      dyn_A_eq.coeffRef(num_pc,wi+c*m*dim + r*m) += v;
    }
    // increment number of position constraints
    num_pc++;
  }
}

// creates 'A_fix_eq matrix' corresponding to one-variable constraints
// specified by fixed transformations 'fixed':
inline void gather_fixed_constraints_system(
  const Eigen::Matrix<int,Eigen::Dynamic,1> &fixed, 
  int dim, 
  int numBones, 
  Eigen::SparseMatrix<double> & A_fix_eq)
{
  Eigen::Matrix<int,Eigen::Dynamic,1> fixed_dim;  

  // Gather list of indices to known values (each dimension of fixed)
  fixed_dim.resize(fixed.size()*dim*(dim+1));
  for(int d = 0;d<dim*(dim+1);d++)
  {
    for(int i = 0;i<(int)fixed.size();i++)
    {
      fixed_dim(fixed.size()*d + i) = d*(numBones)+fixed(i);
    }
  }

  Eigen::DynamicSparseMatrix<double> 
    M(fixed_dim.size(), numBones*dim*(dim + 1));
  for (int i=0; i<fixed_dim.size(); i++)
  {
    M.coeffRef(i, fixed_dim(i, 0)) = 1.0;
  }

  A_fix_eq = M;
}

// creates 'A_fix_eq matrix' corresponding to one-variable constraints
// specified by fixed transformations 'fixed':
inline void join_constraints_systems(
  const Eigen::SparseMatrix<double> & A1, 
  const Eigen::SparseMatrix<double> & A2, 
  Eigen::SparseMatrix<double> & Ajoin)
{
  assert(A1.cols() == A2.cols());
  Eigen::DynamicSparseMatrix<double> dyn_Ajoin(A1.rows() + A2.rows(), A1.cols());
  for (int k=0; k<A1.outerSize(); ++k)
  {
    for (Eigen::SparseMatrix<double>::InnerIterator it(A1,k); it; ++it)  
      dyn_Ajoin.coeffRef(it.row(), it.col()) = it.value();
  }
  for (int k=0; k<A2.outerSize(); ++k)
  {
    for (Eigen::SparseMatrix<double>::InnerIterator it(A2,k); it; ++it)  
      dyn_Ajoin.coeffRef(A1.rows() + it.row(), it.col()) = it.value();
  }
  Ajoin = dyn_Ajoin;
}

#endif
