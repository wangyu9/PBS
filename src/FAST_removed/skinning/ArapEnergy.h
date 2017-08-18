#ifndef ARAPENERGY_H
#define ARAPENERGY_H
//     ARAP_SPOKES  "As-rigid-as-possible Surface Modeling" by [Sorkine and
//       Alexa 2007], rotations defined at vertices affecting incident edges,
//       default
//     ARAP_SPOKES-AND-RIMS  Adapted version of "As-rigid-as-possible Surface
//       Modeling" by [Sorkine and Alexa 2007] presented in section 4.2 of or
//       "A simple geometric model for elastic deformation" by [Chao et al.
//       2010], rotations defined at vertices affecting incident edges and
//       opposite edges
//     ARAP_ELEMENTS  "A local-global approach to mesh parameterization" by
//       [Liu et al.  2010] or "A simple geometric model for elastic
//       deformation" by [Chao et al.  2010], rotations defined at elements
//       (triangles or tets) 
enum ArapEnergy
{
  ARAP_SPOKES,
  ARAP_SPOKES_AND_RIMS,
  ARAP_ELEMENTS
};
#define NUM_ARAPENERGY 3
#endif
