#include "Bone.h"
#include <vector>


// Distribute transformations from stack of affine transformations into bone
// forest roots transformations (in the last_T field)
//
// Inputs:
//   T  #rows by 4*num_handles horizontal stack of transformations
//   BR  list of bone tree roots
// Output:
//   BR  list of bone tree roots updated with last_T from T
// Returns true on success, false on error
inline bool distribute_transformations(
  const Eigen::MatrixXf & T,
  std::vector<Bone*> & BR);

// Implementation
#include "Tform.h"
#include <list>

inline bool distribute_transformations(
  const Eigen::MatrixXf & T,
  std::vector<Bone*> & BR)
{
  int num_handles = T.cols()/4;

  // Insert roots into search queue
  std::list<Bone*> Q;
  for(
    std::vector<Bone*>::iterator bit = BR.begin();
    bit != BR.end();
    bit++)
  {
    Q.push_back(*bit);
  }
  
  while(!Q.empty())
  {
    Bone * b = Q.back();
    Q.pop_back();
    if(b->wi_is_set())
    {
      int wi = b->get_wi();
      if(wi >= num_handles)
      {
        fprintf(
          stderr,
          "Error: verify_bone_roots() max weight index of bones (%d)"
          " >= number of handles (%d)\n",
          wi,
          num_handles);
        return false;
      }
      if(wi >= 0)
      {
        // compute current transformation at bone
        // place transform into bone's last_T
        b->last_T.affine() = T.block(0,b->get_wi()*4,3,4).cast<double>();
      }
    }

    // Add children to queue
    std::vector<Bone*> children = b->get_children();
    Q.insert(Q.end(),children.begin(),children.end());
  }
  
  return true;
}
