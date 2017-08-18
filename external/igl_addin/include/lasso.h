#ifndef IGL_ADDIN_LASSO_H
#define IGL_ADDIN_LASSO_H

#include "handle.h"
#include <vector>

class Lasso{
public:
	bool mouseMove(
		int mouse_x,
		int mouse_y,
		// output:
		bool& calledLassoMove,
		// input:
		bool move_in_plane = false);

	bool mouseDown(
		int mouse_x,
		int mouse_y,
		int modifiers,
		//const std::vector<handleData>& all_handle_list,
		const Eigen::MatrixXd& P,
		// output:
		bool& calledLassoDown,
		int& selected_index
		);

	bool mouseUp(
		int mouse_x,
		int mouse_y,
		const Eigen::MatrixXd& P,
		// output:
		bool& calledLassoUp,
		bool& calledLassoMove,
		std::vector<int>& indics_inside_lasso
		);

	bool mouseMove(
		int mouse_x, 
		int mouse_y, 
		//void (*pLassoMove)(),//void(_cdecl HandlePlugin::*)(void) pLassoMove,//void (*pLassoMove)(), 	
		double *modelview_matrix,
		double *projection_matrix,
		int *viewport,
		// output:
		bool& calledLassoMove,
		bool move_in_plane = false);

	bool mouseDown(
		int mouse_x, 
		int mouse_y,
		double *modelview_matrix,
		double *projection_matrix,
		int *viewport,
		int modifiers,
		//const std::vector<handleData>& all_handle_list,
		const Eigen::MatrixXd& P,
		// output:
		bool& calledLassoDown,
		int& selected_index
		);

	bool mouseUp(
		int mouse_x, 
		int mouse_y,
		double *modelview_matrix,
		double *projection_matrix,
		int *viewport,
		//const std::vector<handleData>& all_handle_list,
		const Eigen::MatrixXd& P,
		// output:
		bool& calledLassoUp,
		bool& calledLassoMove,
		std::vector<int>& indics_inside_lasso
		);

	Lasso();
	void draw();
	// lasso tool
	float translation[3];
	bool is_activate();
private:
	// for mouse call back
	//stuff for mouse callbacks
	int from_x, from_y;
	float depth_at_mouse_pos;
	int to_x, to_y;
	// stores the mesh before a translation/rotation
	//PointMatrixType *previous_vertices;
	//flag, shows whether action (translation/rotation) should be taken in the next move (eg. if the mouse did not hit the mesh, no action will be taken)
	bool activate;
	//flag, shows whether the mouse is currently selecting
	bool selecting;

	//vector of mouse position used by the lasso selection tool
	//anything inside the polyline specified by these points will be picked
	std::vector<std::pair<int, int> > mousePoints;
	//option to update the mesh while moving
	//if set, mesh will be updated while the used drags the mouse
	//otherwise, it will only be updated upon release of the mouse button
	bool m_UPDATE_WHILE_MOVING;
};




#endif /*IGL_ADDIN_LASSO_H*/