#ifndef UP_AXIS_H
#define UP_AXIS_H 
// Determines the up axis or depth axis of the current gl matrix
// Outputs:
//   x  pointer to x-coordinate in scene coordinates of the un-normalized
//     up axis 
//   y  pointer to y-coordinate in scene coordinates of the un-normalized
//     up axis 
//   z  pointer to z-coordinate in scene coordinates of the un-normalized
//     up axis
//
// Note: Up axis is returned *UN-normalized*
inline void up_axis(double * x, double * y, double * z);

inline void up_axis(double * x, double * y, double * z)
{
  double mv[16];
  glGetDoublev(GL_MODELVIEW_MATRIX, mv);
  *x = -mv[0*4+1];
  *y = -mv[1*4+1];
  *z = -mv[2*4+1];
}
#endif


