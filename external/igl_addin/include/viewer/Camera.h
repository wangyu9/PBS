#ifndef CAMERA_H
#define CAMERA_H

#include <Eigen/Core>
#include <igl/OpenGL_convenience.h>

// Snap the quaternion q to the nearest canonical view quaternion
// Input:
//   q  quaternion to be snapped (also see Outputs)
//   threshold  (optional) threshold:
//     1.0 --> snap any input
//     0.5 --> snap inputs somewhat close to canonical views
//     0.0 --> snap no input
// Output:
//   q  quaternion possibly set to nearest canonical view
// Return:
//   true only if q was snapped to the nearest canonical view
bool SnapToCanonicalQuaternion(
	float q[4],
	const double threshold = 1.0);

class Camera
{
public:
	/********* Scene/Camera Options *********/
	float eye[3];
	float up[3];
	float center[3];
	// Scene frustum shift
	double frustum_shift_x;
	// Scene scale
	float g_Zoom;
	// zoom and shift are set once when mesh is loaded to center and normalize
	// shape to fit unit box
	float zoom;
	Eigen::Vector3f g_Translation;
	double view_angle;
	double dnear;
	double dfar;
	double m_projection_matrix[16];
	double m_modelview_matrix[16];
	GLint m_viewport[4];
	// Shape orientation (stored as a quaternion)
	float g_Rotation[4];

	/***********************************/
	bool g_AutoRotate;
	bool g_KeyBoardRotate;//added by wangyu
	float g_RotateSpeed;//added by wangyu
	float key_board_x_rotation;//added by wangyu
	float key_board_y_rotation;//added by wangyu
	float g_StopRotateAngle;//added by wangyu

	// disable rotation capability
	bool enable_rotation;
	// Auto rotate
	bool auto_rotate_restart;
	int g_RotateTime; // This is the starting time of rotation.
	float g_RotateStart[4];
	// snap to nearest canonical view quaternion
	double trackball_snap_to_canonical;

	bool useOthographic;

	/***********************************/
public:
	static float ZERO[];
	static float DEFAULT_QUATERNION[];
	static float CANONICAL_VIEW_QUATERNIONS[][4];

	/********* Loading-Saving Camera*********/
public:

	Camera();
	Camera(const Camera & other);

	bool saveCamera();
	bool loadCamera(const char* fname);
	bool read_camera_from_file(FILE *f);
	bool write_camera_to_file(FILE *f);

	void UpdateCamera(int current_time);

	Eigen::Vector3f screen_to_world(float x, float y, float z);

	void SetAutoRotate(bool value);
	bool GetAutoRotate();
	void SetRotateTime(const float value);
	void view_xy_plane();
	void view_xz_plane();
	void view_yz_plane();

	void resize(int w, int h);

	void set_toggle_ortho(bool value, int w, int h);
	bool get_toggle_ortho();
	bool snap_to_canonical_quaternion(const double threshold = 1.0);
	
	void SetGL();

	GLint _gluProject(GLdouble objX,
		GLdouble objY,
		GLdouble objZ,
		GLdouble* winX,
		GLdouble* winY,
		GLdouble* winZ);
};

#endif/*CAMERA_H*/