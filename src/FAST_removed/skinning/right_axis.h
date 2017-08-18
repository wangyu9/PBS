#ifndef RIGHT_AXIS_H
#define RIGHT_AXIS_H 
// Determines the right axis or depth axis of the current gl matrix
// Outputs:
//   x  pointer to x-coordinate in scene coordinates of the un-normalized
//     right axis 
//   y  pointer to y-coordinate in scene coordinates of the un-normalized
//     right axis 
//   z  pointer to z-coordinate in scene coordinates of the un-normalized
//     right axis
//
// Note: Right axis is returned *UN-normalized*
inline void right_axis(double * x, double * y, double * z);

inline void right_axis(double * x, double * y, double * z)
{
  double mv[16];
  glGetDoublev(GL_MODELVIEW_MATRIX, mv);
  *x = -mv[0*4+0];
  *y = -mv[1*4+0];
  *z = -mv[2*4+0];
}
#endif



