// Determine the intersection between two sets of coefficients using ==
// Templates:
//   M  matrix type that implements indexing by global index M(i)
// Inputs:
//   A  matrix of coefficients
//   B  matrix of coefficients
// Output:
//   C  matrix of elements appearing in both A and B, C is always resized to
//   have a single column
template <class M>
void intersection(const M & A, const M & B, M & C);
// Last argument as return
template <class M>
M intersection(const M & A, const M & B);

// Implementation

template <class M>
void intersection(const M & A, const M & B, M & C)
{
  // Stupid O(size(A) * size(B)) to do it
  M dyn_C(A.size() > B.size() ? A.size() : B.size(),1);
  // count of intersections
  int c = 0;
  // Loop over A
  for(int i = 0;i<A.size();i++)
  {
    // Loop over B
    for(int j = 0;j<B.size();j++)
    {
      if(A(i) == B(j))
      {
        dyn_C(c) = A(i);
        c++;
      }
    }
  }

  // resize output
  C.resize(c,1);
  // Loop over intersections
  for(int i = 0;i<c;i++)
  {
    C(i) = dyn_C(i);
  }
}

template <class M>
M intersection(const M & A, const M & B)
{
  M C;
  intersection(A,B,C);
  return C;
}
