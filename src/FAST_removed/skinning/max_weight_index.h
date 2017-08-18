class Bone;
// Traverse all trees in BR and keep track of the max wi weight index value
// Inputs:
//  BR  list of bone tree roots
// Returns <-1 if BR is empty, -1 if all bones have undefined weight index or
// value >0
inline int max_weight_index(const std::vector<Bone> & BR);

// Implementation
#include "Bone.h"
inline int max_weight_index(const std::vector<Bone> & BR)
{
  int max_wi = -2;
  // Insert roots into search queue
  std::list<Bone*> Q;
  for(
    std::vector<Bone>::const_iterator bit = BR.begin();
    bit != BR.end();
    bit++)
  {
    Q.push_back(&(*bit));
  }
  
  while(!Q.empty())
  {
    Bone * b = Q.pop_back();
    max_wi = max(b->wi,max_wi);
    for(
      std::vector<Bone*>::iterator cit = b->children.begin();
      cit != b->children.end();
      cit++)
    {
      Q.push_back(*cit);
    }
  }
  return max_wi;
}
