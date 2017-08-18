// PROJECT_TO_LINES  project points onto vectors, that is find the paramter
// t for a point p such that proj_p = (y-x).*t, additionally compute the
// squared distance from p to the line of the vector, such that 
// |p - proj_p|² = sqr_d
//
// [T,sqrD] = project_to_lines(P,S,D)
//
// Templates:
//   MatP matrix template for P, implementing .cols(), .rows()
//   MatL matrix template for S and D, implementing .size(), .array(), .sum()
//   Matt matrix template for t 
//   MatsqrD matrix template for sqrD
// Inputs:
//   P  #P by dim list of points to be projected
//   S  size dim start position of line vector
//   D  size dim destination position of line vector
// Outputs:
//   T  #P by 1 list of parameters
//   sqrD  #P by 1 list of squared distances
//
// Copyright 2011, Alec Jacobson (jacobson@inf.ethz.ch)
//
template <
  typename MatP, 
  typename MatL, 
  typename Matt, 
  typename MatsqrD>
inline void project_to_line(
  const MatP & P,
  const MatL & S,
  const MatL & D,
  Matt & t,
  MatsqrD & sqrD);

// Same as above but for a single query point
template <typename Scalar>
inline void project_to_line(
  const Scalar px,
  const Scalar py,
  const Scalar pz,
  const Scalar sx,
  const Scalar sy,
  const Scalar sz,
  const Scalar dx,
  const Scalar dy,
  const Scalar dz,
  Scalar & t,
  Scalar & sqrd);

// Implementation

template <
  typename MatP, 
  typename MatL, 
  typename Matt, 
  typename MatsqrD>
inline void project_to_line(
  const MatP & P,
  const MatL & S,
  const MatL & D,
  Matt & t,
  MatsqrD & sqrD)
{
  // http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html

  // number of dimensions
  int dim = P.cols();
  assert(dim == S.size());
  assert(dim == D.size());
  // number of points
  int np  = P.rows();
  // vector from source to destination
  MatL DmS = D-S;
  double v_sqrlen = (double)(DmS.array().pow(2).sum());
  assert(v_sqrlen != 0);
  // resize output
  t.resize(np,1);
  sqrD.resize(np,1);
  // loop over points 
  for(int i = 0;i<np;i++)
  {
    MatL Pi = P.row(i);
    // vector from point i to source
    MatL SmPi = S-Pi;
    t(i) = -(DmS.array()*SmPi.array()).sum() / v_sqrlen;
    // P projected onto line
    MatL projP = (1-t(i))*S + t(i)*D;
    sqrD(i) = (Pi-projP).array().pow(2).sum();
  }
}

template <typename Scalar>
inline void project_to_line(
  const Scalar px,
  const Scalar py,
  const Scalar pz,
  const Scalar sx,
  const Scalar sy,
  const Scalar sz,
  const Scalar dx,
  const Scalar dy,
  const Scalar dz,
  Scalar & t,
  Scalar & sqrd)
{
  // vector from source to destination
  Scalar dms[3];
  dms[0] = dx-sx;
  dms[1] = dy-sy;
  dms[2] = dz-sz;
  Scalar v_sqrlen = dms[0]*dms[0] + dms[1]*dms[1] + dms[2]*dms[2];
  // line should have some length
  assert(v_sqrlen != 0);
  // vector from point to source
  Scalar smp[3];
  smp[0] = sx-px;
  smp[1] = sy-py;
  smp[2] = sz-pz;
  t = -(dms[0]*smp[0]+dms[1]*smp[1]+dms[2]*smp[2])/v_sqrlen;
  // P projectred onto line
  Scalar projp[3];
  projp[0] = (1.0-t)*sx + t*dx;
  projp[1] = (1.0-t)*sy + t*dy;
  projp[2] = (1.0-t)*sz + t*dz;
  // vector from projected point to p
  Scalar pmprojp[3];
  pmprojp[0] = px-projp[0];
  pmprojp[1] = py-projp[1];
  pmprojp[2] = pz-projp[2];
  sqrd = pmprojp[0]*pmprojp[0] + pmprojp[1]*pmprojp[1] + pmprojp[2]*pmprojp[2];
}
