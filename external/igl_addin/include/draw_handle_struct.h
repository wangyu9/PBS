#ifndef IGL_ADDIN_DRAW_HANDLE_STRUCT_H
#define IGL_ADDIN_DRAW_HANDLE_STRUCT_H

#include <igl/igl_inline.h>
#include "handle_structure.h"



void draw_point_handles(
	const float handle_radius,
	const float handle_color[4],
	const float diameter,
	const float ratio_of_passive,
	const bool draw_region_handles,
	const bool draw_only_point_handle_center,
	const HandleStructure& hs,
	std::vector<bool> bToDraw=std::vector<bool>()// this is also only input
	);



#ifdef IGL_HEADER_ONLY
#	include "draw_handle_struct.cpp"
#endif

#endif /*IGL_ADDIN_DRAW_HANDLE_STRUCT_H*/