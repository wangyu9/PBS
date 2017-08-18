#ifndef VIEW_AXIS_H
#define VIEW_AXIS_H 
// Determines the view axis or depth axis of the current gl matrix
// Outputs:
//   x  pointer to x-coordinate in scene coordinates of the un-normalized
//     viewing axis 
//   y  pointer to y-coordinate in scene coordinates of the un-normalized
//     viewing axis 
//   z  pointer to z-coordinate in scene coordinates of the un-normalized
//     viewing axis
//
// Note: View axis is returned *UN-normalized*
inline void view_axis(double * x, double * y, double * z);

inline void view_axis(double * x, double * y, double * z)
{
  double mv[16];
  glGetDoublev(GL_MODELVIEW_MATRIX, mv);
  *x = -mv[0*4+2];
  *y = -mv[1*4+2];
  *z = -mv[2*4+2];
}
#endif

