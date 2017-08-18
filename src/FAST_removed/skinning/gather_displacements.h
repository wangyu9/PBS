#include "Bone.h"
#include <vector>

// Gather displacements from bone forest roots
// Inputs:
//   BR  list of bone tree roots
//   D  #handles by #dim list of displacements
// Output:
//   D  #handles by #dim list of displacements
// Returns true on success, false on error
inline bool gather_displacements(
  const std::vector<Bone*> & BR, 
  Eigen::MatrixXd & D);

// Implementation
#include "Tform.h"
#include <list>

inline bool gather_displacements(
  const std::vector<Bone*> & BR, 
  Eigen::MatrixXd & D)
{
  int num_handles = D.rows();

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
          "Error: gather_displacements() max weight index of bones (%d)"
          " >= number of handles (%d)\n",
          wi,
          num_handles);
        return false;
      }
      if(wi >= 0)
      { 
        D.row(b->get_wi()) = b->tip(false,false) - b->rest_tip();
      }
    }

    // Add children to queue
    std::vector<Bone*> children = b->get_children();
    Q.insert(Q.end(),children.begin(),children.end());
  }

  return true;
}
