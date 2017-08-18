#include "Camera.h"

#include "FileDialog.h"
#include "math_helper.h"

#define SQRT_2_OVER_2 0.707106781f
#define NUM_CANONICAL_VIEW_QUATERNIONS 24
#define pi 3.1415926535897932384626433832795

float Camera::ZERO[] = { 0.0f, 0.0f, 0.0f, 0.0f };
float Camera::DEFAULT_QUATERNION[] = { 0.0f, 0.0f, 0.0f, 1.0f };
float Camera::CANONICAL_VIEW_QUATERNIONS[][4] =
{
	{ 0, 0, 0, 1 },
	{ 0, 0, SQRT_2_OVER_2, SQRT_2_OVER_2 },
	{ 0, 0, 1, 0 },
	{ 0, 0, SQRT_2_OVER_2, -SQRT_2_OVER_2 },

	{ 0, -1, 0, 0 },
	{ -SQRT_2_OVER_2, SQRT_2_OVER_2, 0, 0 },
	{ -1, 0, 0, 0 },
	{ -SQRT_2_OVER_2, -SQRT_2_OVER_2, 0, 0 },

	{ -0.5, -0.5, -0.5, 0.5 },
	{ 0, -SQRT_2_OVER_2, 0, SQRT_2_OVER_2 },
	{ 0.5, -0.5, 0.5, 0.5 },
	{ SQRT_2_OVER_2, 0, SQRT_2_OVER_2, 0 },

	{ SQRT_2_OVER_2, 0, -SQRT_2_OVER_2, 0 },
	{ 0.5, 0.5, -0.5, 0.5 },
	{ 0, SQRT_2_OVER_2, 0, SQRT_2_OVER_2 },
	{ -0.5, 0.5, 0.5, 0.5 },

	{ 0, SQRT_2_OVER_2, SQRT_2_OVER_2, 0 },
	{ -0.5, 0.5, 0.5, -0.5 },
	{ -SQRT_2_OVER_2, 0, 0, -SQRT_2_OVER_2 },
	{ -0.5, -0.5, -0.5, -0.5 },

	{ -SQRT_2_OVER_2, 0, 0, SQRT_2_OVER_2 },
	{ -0.5, -0.5, 0.5, 0.5 },
	{ 0, -SQRT_2_OVER_2, SQRT_2_OVER_2, 0 },
	{ 0.5, -0.5, 0.5, -0.5 }
};


bool SnapToCanonicalQuaternion(
	float q[4],
	const double threshold)
{
	// 0.290019
	// 0.300000
	// 0.400000
	double q_mag = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	if (q_mag == 0)
	{
		printf("ERROR: snap_to_canonical_quaternion passed (0,0,0,0)\n");
		return false;
	}

	const double MAX_DISTANCE = 0.5;
	double min_distance = 2 * MAX_DISTANCE;
	int min_index = -1;
	int min_sign = 0;
	// loop over canonical view quaternions
	for (int sign = -1; sign <= 1; sign += 2)
	{
		for (int i = 0; i < NUM_CANONICAL_VIEW_QUATERNIONS; i++)
		{
			float distance = 0.0;
			// loop over coordinates
			for (int j = 0; j < 4; j++)
			{
				distance +=
					(q[j] - sign*Camera::CANONICAL_VIEW_QUATERNIONS[i][j])*
					(q[j] - sign*Camera::CANONICAL_VIEW_QUATERNIONS[i][j]);
			}
			if (min_distance > distance)
			{
				min_distance = distance;
				min_index = i;
				min_sign = sign;
			}
		}
	}

	if (MAX_DISTANCE < min_distance)
	{
		printf("FOUND NEW MAX MIN_DISTANCE: %g\n", min_distance);
	}

	if (min_index < 0)
	{
		printf("q: %g %g %g %g\n", q[0], q[1], q[2], q[3]);
	}
	assert(min_index >= 0);

	//printf("min/max: %g <=? %g\n",(min_distance/MAX_DISTANCE),threshold);
	if (min_distance / MAX_DISTANCE <= threshold)
	{
		// loop over coordinates
		for (int j = 0; j < 4; j++)
		{
			q[j] = min_sign*Camera::CANONICAL_VIEW_QUATERNIONS[min_index][j];
		}
		return true;
	}
	return false;
}


Camera::Camera()
{
	// Camera
	frustum_shift_x = 0.125;
	//    frustum_shift_x = 0.;

	view_angle = 45.0;
	dnear = 1.0;
	dfar = 100.0;
	g_Zoom = 1.0f;
	zoom = 1.0f;
	g_Translation << 0, 0, 0;
	CopyArray4(DEFAULT_QUATERNION, g_Rotation);

	// View

	g_AutoRotate = 0;
	g_KeyBoardRotate = false;//wangyu
	g_RotateSpeed = 1.0;//wangyu
	g_StopRotateAngle = 1e30;//wangyu set to a huge value, the max possible value for float is +3.4e^38

	useOthographic = false;

	key_board_x_rotation = 0.0;
	key_board_y_rotation = 0.0;

	trackball_snap_to_canonical = 0.0;
	enable_rotation = true;

	auto_rotate_restart = false;
	g_RotateTime = 0;

	CopyArray4(DEFAULT_QUATERNION, g_RotateStart);

	// Init rotation
	float axis[] = { 0.7f, 0.7f, 0.0f }; // initial model rotation
	float angle = 0.8f;
	SetQuaternionFromAxisAngle(axis, angle, g_Rotation);
	SetQuaternionFromAxisAngle(axis, angle, g_RotateStart);
}

Camera::Camera(const Camera& other)
{
	// Camera
	frustum_shift_x = other.frustum_shift_x;

	view_angle = other.view_angle;
	dnear = other.dnear;
	dfar = other.dfar;
	g_Zoom = other.g_Zoom;
	zoom = other.zoom;
	g_Translation = other.g_Translation;
	CopyArray4(other.g_Rotation, g_Rotation);

	// View

	g_AutoRotate = other.g_AutoRotate;
	g_KeyBoardRotate = other.g_KeyBoardRotate;//wangyu
	g_RotateSpeed = other.g_RotateSpeed;//wangyu
	g_StopRotateAngle = other.g_StopRotateAngle;//wangyu set to a huge value, the max possible value for float is +3.4e^38

	useOthographic = other.useOthographic;

	key_board_x_rotation = other.key_board_x_rotation;
	key_board_y_rotation = other.key_board_y_rotation;

	trackball_snap_to_canonical = other.trackball_snap_to_canonical;
	enable_rotation = other.enable_rotation;

	auto_rotate_restart = other.auto_rotate_restart;
	g_RotateTime = other.g_RotateTime;

	CopyArray4(other.g_Rotation, g_Rotation);
	CopyArray4(other.g_RotateStart, g_RotateStart);

}

void Camera::SetRotateTime(const float value)
{
	g_RotateTime = value;
}

bool Camera::saveCamera()
{
	bool ret = false;
	char fname[FILE_DIALOG_MAX_BUFFER];
	fname[0] = 0;
	get_save_file_path(fname);

	if (fname[0] == 0)
		return false;
	FILE *f = fopen(fname, "w");
	if (NULL == f)
	{
		printf("IOError: %s could not be opened for reading...", fname);
		return false;
	}

	ret = write_camera_to_file(f);
	fclose(f);
	return ret;
}

bool Camera::write_camera_to_file(FILE *f)
{
	fprintf(f, "%.10g %.10g %.10g %.10g %.10g %.10g %.10g %.10g %.10g",
		g_Translation[0],
		g_Translation[1],
		g_Translation[2],
		g_Rotation[0],
		g_Rotation[1],
		g_Rotation[2],
		g_Rotation[3],
		g_Zoom,
		zoom);
	return true;
}

bool Camera::loadCamera(const char* fname)
{
	bool ret = false;

	if (fname[0] == 0)
		return false;
	FILE *f = fopen(fname, "r");
	if (NULL == f)
	{
		printf("IOError: %s could not be opened for reading...", fname);
		return false;
	}
	ret = read_camera_from_file(f);
	fclose(f);

	return ret;
}

bool Camera::read_camera_from_file(FILE *f)
{
	int count = fscanf(f, "%g %g %g %g %g %g %g %g %g",
		&(g_Translation[0]),
		&(g_Translation[1]),
		&(g_Translation[2]),
		&(g_Rotation[0]),
		&(g_Rotation[1]),
		&(g_Rotation[2]),
		&(g_Rotation[3]),
		&(g_Zoom),
		&(zoom));
	return count == 9;

	return true;
}

void Camera::SetAutoRotate(bool value)
{
	g_AutoRotate = value; // copy value to g_AutoRotate
	if (g_AutoRotate != 0)
	{
		// init rotation
		//g_RotateTime = glutGet(GLUT_ELAPSED_TIME);
		auto_rotate_restart = true;

		// make Rotation variable read-only
		//TwDefine(" IGLViewer/ObjRotation readonly ");
	}
	else
	{
		// make Rotation variable read-write
		//TwDefine(" IGLViewer/ObjRotation readwrite ");
	}
}

bool Camera::GetAutoRotate()
{
	return g_AutoRotate; // copy g_AutoRotate to value
}

void Camera::UpdateCamera(int current_time)
{
	if (g_AutoRotate)
	{
		if (auto_rotate_restart)
		{
			g_RotateStart[0] = g_Rotation[0];
			g_RotateStart[1] = g_Rotation[1];
			g_RotateStart[2] = g_Rotation[2];
			g_RotateStart[3] = g_Rotation[3];
			g_RotateTime = current_time;
			auto_rotate_restart = false;

		}

		float axis[3] = { 0, 1, 0 };
		float angle = (float)(current_time - g_RotateTime) / 1000.0f;
		if (abs(angle*g_RotateSpeed) < g_StopRotateAngle)
		{
			float quat[4];
			SetQuaternionFromAxisAngle(axis, angle*g_RotateSpeed, quat);
			MultiplyQuaternions(g_RotateStart, quat, g_Rotation);
		}
	}
	else if (g_KeyBoardRotate)
	{
		float identity_rotate[4] = { 0, 0, 0, 1 };
		//added by wangyu key board rotate
		float axis_x[3] = { 1, 0, 0 };
		float quat_x[4];
		SetQuaternionFromAxisAngle(axis_x, key_board_x_rotation, quat_x);
		MultiplyQuaternions(identity_rotate, quat_x, g_Rotation);
		float axis_y[3] = { 0, 1, 0 };
		float quat_y[4];
		SetQuaternionFromAxisAngle(axis_y, key_board_y_rotation*g_RotateSpeed, quat_y);
		MultiplyQuaternions(g_Rotation, quat_y, g_Rotation);
	}
}

Eigen::Vector3f Camera::screen_to_world(float x, float y, float z)
{
	GLfloat winX, winY;
	Eigen::Vector3d point;

	winX = (float)x;
	winY = (float)m_viewport[3] - (float)y;

	gluUnProject(winX, winY, z, m_modelview_matrix, m_projection_matrix, m_viewport, (GLdouble*)&point[0], (GLdouble*)&point[1], (GLdouble*)&point[2]);

	return Eigen::Vector3f(point[0], point[1], point[2]);
}

void Camera::view_xy_plane()
{
	g_Rotation[0] = 0.0f;
	g_Rotation[1] = 0.0f;
	g_Rotation[2] = 0.0f;
	g_Rotation[3] = 1.0f;
	auto_rotate_restart = g_AutoRotate;
}

void Camera::view_xz_plane()
{
	g_Rotation[0] = -sqrt(2.0f) / 2.0f;
	g_Rotation[1] = 0.0f;
	g_Rotation[2] = 0.0f;
	g_Rotation[3] = sqrt(2.0f) / 2.0f;
	auto_rotate_restart = g_AutoRotate;
}

void Camera::view_yz_plane()
{
	g_Rotation[0] = -0.5f;
	g_Rotation[1] = -0.5f;
	g_Rotation[2] = -0.5f;
	g_Rotation[3] = 0.5f;
	auto_rotate_restart = g_AutoRotate;
}

void Camera::resize(int width, int height)
{
	m_viewport[2] = width;
	m_viewport[3] = height;

	eye[0] = 0;
	eye[1] = 0;
	eye[2] = 5;

	center[0] = 0;
	center[1] = 0;
	center[2] = 0;

	up[0] = 0;
	up[1] = 1;
	up[2] = 0;

	if (useOthographic)
	{
		// Set OpenGL viewport and camera
		glViewport(0, 0, width, height);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();

		// Set the correct perspective.
		double length = sqrt((eye[0] - center[0]) * (eye[0] - center[0]) + (eye[1] - center[1]) * (eye[1] - center[1]) + (eye[2] - center[2]) * (eye[2] - center[2]));
		double h = tan(view_angle / 360.0 * pi) * (length);
		glOrtho(-h*width / height - frustum_shift_x*length / dnear, h*width / height - frustum_shift_x*length / dnear, -h, h, dnear, dfar);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		gluLookAt(eye[0], eye[1], eye[2], center[0], center[1], center[2], up[0], up[1], up[2]);

	}
	else
	{
		glViewport(0, 0, width, height);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();

		//gluPerspective(40, (double)width/height, 1, 100);
		// from: http://nehe.gamedev.net/data/articles/article.asp?article=11
		// shift everything a little to the right
		double fH = tan(view_angle / 360.0 * pi) * dnear;
		double fW = fH * (double)width / (double)height;

		glFrustum(-fW - frustum_shift_x, fW - frustum_shift_x, -fH, fH, dnear, dfar);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		gluLookAt(eye[0], eye[1], eye[2], center[0], center[1], center[2], up[0], up[1], up[2]);
	}
}

void Camera::set_toggle_ortho(bool value, int width, int height)
{
	if (useOthographic != value){
		useOthographic = value;
		if (useOthographic)
		{
			// Set OpenGL viewport and camera
			glViewport(0, 0, width, height);
			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();

			// Set the correct perspective
			// Set the average depth of the scene to the distance between the camera viewpoint and the center of the scene (where the camera looks at.
			double length = sqrt((eye[0] - center[0]) * (eye[0] - center[0]) + (eye[1] - center[1]) * (eye[1] - center[1]) + (eye[2] - center[2]) * (eye[2] - center[2]));
			//this is basically the same height as the one specified in the perspective case, except now it is scaled by this average depth
			//instead of dnear
			double h = tan(view_angle / 360.0 * pi) * (length);
			//we also need to scale the frustum_shift_x to take into account this change
			glOrtho(-h*width / height - frustum_shift_x*length / dnear, h*width / height - frustum_shift_x*length / dnear, -h, h, dnear, dfar);

			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();
			gluLookAt(eye[0], eye[1], eye[2], center[0], center[1], center[2], up[0], up[1], up[2]);

		}
		else
		{
			glViewport(0, 0, width, height);
			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();

			double fH = tan(view_angle / 360.0 * pi) * dnear;
			double fW = fH * (double)width / (double)height;

			glFrustum(-fW - frustum_shift_x, fW - frustum_shift_x, -fH, fH, dnear, dfar);

			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();
			gluLookAt(eye[0], eye[1], eye[2], center[0], center[1], center[2], up[0], up[1], up[2]);
		}
	}

}

bool Camera::get_toggle_ortho()
{
	return useOthographic;
}

bool Camera::snap_to_canonical_quaternion(const double threshold)
{
	return SnapToCanonicalQuaternion(g_Rotation, threshold);
}

void Camera::SetGL()
{
	float mat[4 * 4]; // rotation matrix

	ConvertQuaternionToMatrix(g_Rotation, mat);
	glMultMatrixf(mat);

	glScaled(g_Zoom, g_Zoom, g_Zoom);
	glScaled(zoom, zoom, zoom);
	glTranslatef(g_Translation[0], g_Translation[1], g_Translation[2]);

	glGetIntegerv(GL_VIEWPORT, m_viewport);
	glGetDoublev(GL_PROJECTION_MATRIX, m_projection_matrix);
	glGetDoublev(GL_MODELVIEW_MATRIX, m_modelview_matrix);

}

GLint Camera::_gluProject(GLdouble objX,
	GLdouble objY,
	GLdouble objZ,
	GLdouble* winX,
	GLdouble* winY,
	GLdouble* winZ)
{
//	GLint iVP[4] = { m_viewport[0], m_viewport[1], m_viewport[2], m_viewport[3] };

	return gluProject(
		objX, 
		objY, 
		objZ,
		m_modelview_matrix,
		m_projection_matrix,
		m_viewport,
		winX,
		winY,
		winZ);

}