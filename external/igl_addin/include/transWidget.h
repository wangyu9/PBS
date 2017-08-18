#ifndef IGL_ADDIN_TRANS_WIDGET
#define IGL_ADDIN_TRANS_WIDGET

//#include <igl/RotateWidget.h>
#include "rotatewidget.h"
#include "lasso.h"

class PreTransWidgetGLSettings:public igl::PreRotateWidgetGLSettings{

	// do not need to use by now
};

class TransWidget :public igl::RotateWidget
{
public:
	bool down(const int x, const int y, const int height);
	bool drag(const int x, const int y, const int height);
	bool up(const int x, const int y, const int height);
	void draw() const;

	void draw_pre() const;
	void draw_mid() const;
	void draw_post() const;

	void draw_mid_pre(PreTransWidgetGLSettings &ps) const;
	void draw_mid_mid() const;
	void draw_mid_post(const PreTransWidgetGLSettings&ps) const;

	bool selected;
	bool is_selected() const;

	TransWidget();

	bool update_during_move;

	bool draw_only_when_selected;
	bool draw_center;

	bool enable_rotation;
	bool enable_translation;
	Eigen::Vector3d translation;
	double scaling;


	void push_translation() const;
	void pop_translation() const;

	void set_displacement(const Eigen::Vector3d& t);

	Eigen::Vector2d mouse_xy;
	Eigen::Vector2d mouse_down_xy;
	bool is_mouse_down;

	// still under construction:

	bool lasso_selected;

	Lasso lasso;
	void lassoDownAlt(int mouse_x, int mouse_y){}
	void lassoDownShift(){}
	void lassoDownNone(int mouse_x, int mouse_y, int selected_index);
	void lassoUp(const std::vector<int>& indics_inside_lasso);
	void lassoMove(float t[3]);

	Eigen::MatrixXd RotateWidgetPos();

};

#ifdef IGL_HEADER_ONLY
#include "transWidget.cpp"
#endif

#endif