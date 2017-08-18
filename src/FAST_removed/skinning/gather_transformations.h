#include "Bone.h"
#include <vector>

// Gather transformations from bone forest roots into stack of affine
// transformations
// Inputs:
//   BR  list of bone tree roots
//   use_last_T  use last_T transformation instead of .affine()
//   T  #rows by 4*num_handles horizontal stack of transformations
// Output:
//   T  #rows by 4*num_handles horizontal stack of transformations
// Returns true on success, false on error
inline bool gather_transformations(
  const std::vector<Bone*> & BR, 
  const bool use_last_T,
  Eigen::MatrixXf & T);

// Implementation
#include "Tform.h"
#include <list>

inline bool gather_transformations(
  const std::vector<Bone*> & BR, 
  const bool use_last_T,
  Eigen::MatrixXf & T)
{
  int num_handles = T.cols()/4;

  // Insert roots into search queue
  std::list<const Bone*> Q;
  for(
    std::vector<Bone*>::const_iterator bit = BR.begin();
    bit != BR.end();
    bit++)
  {
    Q.push_back(*bit);
  }
  
  while(!Q.empty())
  {
    const Bone * b = Q.back();
    Q.pop_back();
    if(b->wi_is_set())
    {
      int wi = b->get_wi();
      if(wi >= num_handles)
      {
        fprintf(
          stderr,
          "Error: gather_transformations() max weight index of bones (%d)"
          " >= number of handles (%d)\n",
          wi,
          num_handles);
        return false;
      }
      if(wi >= 0)
      {
        // compute current transformation at bone
        Tform3 bT;
        if(use_last_T)
        {
          bT = b->last_T;
        }else
        {
          bT = b->affine();
        }
        // place transform into T stack
        T.block(0,b->get_wi()*4,3,4) = bT.affine().matrix().cast<float>();
      }
    }

    // Add children to queue
    std::vector<Bone*> children = b->get_children();
    Q.insert(Q.end(),children.begin(),children.end());
  }

  return true;
}
