#include "transWidget.h"

#include "Viewer.h"
#include "transform.h"

#define TEMP_DEBUG_MARKER true
#define TRANSLATION_HAS_NOT_FINISHED_YET false

TransWidget::TransWidget() :
translation(0., 0., 0.), 
scaling(1.0),
RotateWidget(), enable_translation(true), enable_rotation(true), 
lasso_selected(false), is_mouse_down(false), update_during_move(true), 
draw_center(true), draw_only_when_selected(false),
selected(false)
{

}

void TransWidget::push_translation() const
{
	Eigen::MatrixXf Mat = Eigen::MatrixXf::Identity(4, 4);
	Mat.block(0, 3, 3, 1) = translation.cast<float>();
	glPushMatrix();
	glMultMatrixf(Mat.data());
}

void TransWidget::pop_translation() const
{
	glPopMatrix();
}

bool TransWidget::down(const int x, const int y, const int height)
{
	push_translation();

	// look at translation first
	if (enable_translation&&TEMP_DEBUG_MARKER)
	{

		using namespace Eigen;
		using namespace igl;
		using namespace std;

		mouse_down_xy = mouse_xy = Vector2d(x, height - y);
		Vector3d ppos = project(RotateWidget::pos);
		const double r = (ppos.head(2) - mouse_xy).norm();
		const double thresh = 10;

		if (r<thresh)
		{
			is_mouse_down = true;
		}
		else
		{
			is_mouse_down = false;
		}

		if (is_mouse_down)
		{
			pop_translation();
			return true;
		}
	}

	if (enable_translation&&TRANSLATION_HAS_NOT_FINISHED_YET)
	{

		bool any_response = false;
		{// lasso 
			bool callLassoDown = false;
			int inserted_index = -1;
			int modifiers = Preview3D::NO_KEY;
			any_response |= lasso.mouseDown(
				x, y,
				modifiers,
				RotateWidgetPos(),
				callLassoDown,
				inserted_index
				);
			if (callLassoDown)
			{
				switch (modifiers)
				{
				case Preview3D::ALT:
					lassoDownAlt(x, y);
					break;
				case Preview3D::SHIFT:
					lassoDownShift();
					break;
				case Preview3D::CTRL:
					break;
				default:
					lassoDownNone(x, y, inserted_index);
				}
			}
		}
		if (any_response) return true;
	}
	
	
	bool r = false;
	// if no translation then look at rotation
	if (enable_rotation)
	{
		r = RotateWidget::down(x, height - y);//alec's usage: (mouse_x,height-mouse_y);
	}
	
	pop_translation();

	return r;
}

void TransWidget::set_displacement(const Eigen::Vector3d& d)
{
	translation += d;
}

bool TransWidget::up(const int x, const int y, const int height)
{
	push_translation();

	// look at translation first
	if (enable_translation&&TEMP_DEBUG_MARKER)
	{

		using namespace Eigen;
		using namespace igl;
		using namespace std;

		mouse_xy = Vector2d(x, height-y);
		
		if (is_mouse_down)
		{
			//Vector3d t = unproject(Vector3d((mouse_xy - mouse_down_xy).x(), (mouse_xy - mouse_down_xy).y(), 0.));
			float t[3];
			get_translation( (int)mouse_xy.x(), (int)mouse_down_xy.x(), height-(int)mouse_xy.y(), height-(int)mouse_down_xy.y(), 0., t);// this is because of the mouse_xy = height - y;
			//Vector3d t = Vector3d((mouse_xy - mouse_down_xy).x(), (mouse_xy - mouse_down_xy).y(), 0.);
			set_displacement(Vector3d(t[0], t[1], t[2]));
		}

		if (is_mouse_down)
		{
			is_mouse_down = false;
			pop_translation();
			return true;
		}	
	}
	if (enable_translation&&TRANSLATION_HAS_NOT_FINISHED_YET)
	{
		bool any_response = false;
		bool callLassoUp = false;
		bool callLassoMove = false;
		std::vector<int> indics_inside_lasso;
		any_response |= lasso.mouseUp(
			x, y,
			RotateWidgetPos(),
			callLassoUp,
			callLassoMove,
			indics_inside_lasso
			);
		if (callLassoUp) lassoUp(indics_inside_lasso);
		if (callLassoMove) lassoMove(lasso.translation);

		if (any_response) return true;
	}

	bool r = false;
	// if no translation then look at rotation
	if (enable_rotation)
	{
		r = RotateWidget::up(x, y);// no height here accoridng to alec's usage.
	}

	pop_translation();

	return r;
}

bool TransWidget::drag(const int x, const int y, int height)
{
	push_translation();

	// look at translation first
	if (enable_translation&&TEMP_DEBUG_MARKER)
	{
		using namespace Eigen;
		using namespace igl;
		using namespace std;

		mouse_xy = Vector2d(x, height - y);

		if (is_mouse_down&&update_during_move)
		{
			//Vector3d t = unproject(Vector3d((mouse_xy - mouse_down_xy).x(), (mouse_xy - mouse_down_xy).y(), 0.));
			float t[3];
			get_translation((int)mouse_xy.x(), (int)mouse_down_xy.x(), height - (int)mouse_xy.y(), height - (int)mouse_down_xy.y(), 0., t);// this is because of the mouse_xy = height - y;
			//Vector3d t = Vector3d((mouse_xy - mouse_down_xy).x(), (mouse_xy - mouse_down_xy).y(), 0.);
			//if (t.norm()>40)
			//{
			//	set_displacement(Vector3d(0., 0., 0.));
			//}
			//else
			{
				set_displacement(Vector3d(t[0], t[1], t[2]));// this is because of the mouse_xy = height - y;
			}
			
			mouse_down_xy = mouse_xy;
		}

		if (is_mouse_down)
		{
			pop_translation();
			return true;
		}
	}
	if (enable_translation&&TRANSLATION_HAS_NOT_FINISHED_YET)
	{
		bool any_response = false;
		bool callLassoMove = false;
		any_response |= lasso.mouseMove(x, y, callLassoMove);
		if (callLassoMove) lassoMove(lasso.translation);

		if (any_response) return true;
	}

	bool r = false;
	// if no translation then look at rotation
	if (enable_rotation)
	{
		r = RotateWidget::drag(x, height - y);//alec's usage: (mouse_x,height-mouse_y);
	}

	pop_translation();

	return r;
}

#include "draw_point.h"
void TransWidget::draw() const
{
	draw_pre();
	draw_mid();
	draw_post();
}

bool TransWidget::is_selected() const
{
	return selected;//(RotateWidget::selected_type != RotateWidget::DOWN_TYPE_NONE);// RotateWidget::is_down();//
}

void TransWidget::draw_pre() const 
{
	push_translation();
	if (enable_translation)
	{
		// Disable draw here: 
		if (draw_center)
		{
			if(true)//if (!draw_only_when_selected||is_selected())
			{
				// push depth setting
				GLboolean old_depth_test;
				glGetBooleanv(GL_DEPTH_TEST, &old_depth_test);
				glDisable(GL_DEPTH_TEST);
				
				draw_point(pos(0), pos(1), pos(2), 7., is_selected(), 0., 1., 0.);

				// pop depth setting
				if (old_depth_test)	glEnable(GL_DEPTH_TEST);
			}
		}
			
	}
}

void TransWidget::draw_mid() const
{
	PreTransWidgetGLSettings ps;
	draw_mid_pre(ps);
	draw_mid_mid();
	draw_mid_post(ps);
}

void TransWidget::draw_mid_pre(PreTransWidgetGLSettings& ps) const
{
	if (enable_rotation)
	{
		if (!draw_only_when_selected || is_selected())
		{
			RotateWidget::draw_pre(ps);
		}
	}
}

void TransWidget::draw_mid_mid() const
{
	if (enable_rotation)
	{
		if (!draw_only_when_selected || is_selected())
		{
			RotateWidget::draw_mid();
		}		
	}
}

void TransWidget::draw_mid_post(const PreTransWidgetGLSettings& ps) const
{
	if (enable_rotation)
	{
		if (!draw_only_when_selected || is_selected())
		{
			RotateWidget::draw_post(ps);
		}
	}
}

void TransWidget::draw_post() const
{
	pop_translation();
}

Eigen::MatrixXd TransWidget::RotateWidgetPos()
{
	Eigen::MatrixXd P(1, 3);
	P(0, 0) = pos(0);
	P(0, 1) = pos(1);
	P(0, 2) = pos(2);
	return P;
}

void TransWidget::lassoDownNone(int mouse_x, int mouse_y, int selected_index)
{
	if (selected_index>=0)
	{
		lasso_selected = true;
	}
}

void TransWidget::lassoUp(const std::vector<int>& indics_inside_lasso)
{
	lasso_selected = false;
}

void TransWidget::lassoMove(float t[3])
{
	if (lasso_selected)
	{
		//translation(0) += t[0];
		//translation(1) += t[1];
		//translation(2) += t[2];
	}
}