#include "Bone.h"
#include <vector>

// Gather rest positions from bone forest roots
// Inputs:
//   BR  list of bone tree roots
//   C  #handles by #dim list of rest positions 
// Output:
//   C  #handles by #dim list of rest positions 
// Returns true on success, false on error
inline bool gather_rest_positions(
  const std::vector<Bone*> & BR, 
  Eigen::MatrixXd & C);

// Implementation
#include "Tform.h"
#include <list>

inline bool gather_rest_positions(
  const std::vector<Bone*> & BR, 
  Eigen::MatrixXd & C)
{
  int num_handles = C.rows();

  // Insert roots into search queue
  std::list<const Bone*> Q;
  for(
    std::vector<Bone*>::const_iterator bit = BR.begin();
    bit != BR.end();
    bit++)
  {
    Q.push_back(*bit);
    // Don't know how to deal with ancestors yet
    assert((*bit)->is_root());
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
          "Error: gather_rest_positions() max weight index of bones (%d)"
          " >= number of handles (%d)\n",
          wi,
          num_handles);
        return false;
      }
      if(wi >= 0)
      { 
        C.row(b->get_wi()) = b->offset;
      }
    }

    // Add children to queue
    std::vector<Bone*> children = b->get_children();
    Q.insert(Q.end(),children.begin(),children.end());
  }

  return true;
}
