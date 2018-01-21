#ifndef MESH_H
#define MESH_H


#ifdef _WIN32
#include <GL/glew.h>
#endif

#ifdef __APPLE__
#define _MACOSX
#endif

#include <AntTweakBar.h>
#include <set>

#ifdef __APPLE__
#   include <OpenGL/gl.h>
#else
#   ifdef _WIN32
#       include <windows.h>
#   endif
#   include <GL/gl.h>
#endif

//igl addin
#include <colorbar.h>//wangyu
#include <igl/per_vertex_normals.h>//wangyu

// Mesh Drawing
enum MeshDrawingType
{
	MESH_DRAWING_DEFAULT,
	MESH_DRAWING_PBS
};
#define NUM_MESH_DRAWING_TYPE 2

// Type of normals
enum NormalsType
{
	PER_FACE,   // Use normals defined on every face
	PER_VERTEX, // Use normals defined on every vertex
	PER_CORNER  // Use normals defined on corners of triangles (multiple normals per vertex)
};
#define NUM_NORMALS_TYPE 3

inline void draw_text(double x, double y, double z, std::string str, bool small_b);

#include "MeshData.h"

class Light
{
public:
	// Light parameter
	bool use_lighting;
	float g_LightMultiplier;
	float g_LightDirection[3];
	float g_LightModel_Ambient[4];

	Light();

	void set_lighting_GL();

	static float DEFAULT_LIGHT_DIRECTION[];
};

/********* Static Variables ***************/

const float GOLD_AMBIENT[4] = { 51.0 / 255.0, 43.0 / 255.0, 33.3 / 255.0, 1.0f };
const float GOLD_DIFFUSE[4] = { 255.0 / 255.0, 228.0 / 255.0, 58.0 / 255.0, 1.0f };
const float GOLD_SPECULAR[4] = { 255.0 / 255.0, 235.0 / 255.0, 80.0 / 255.0, 1.0f };
const float SILVER_AMBIENT[4] = { 0.2f, 0.2f, 0.2f, 1.0f };
const float SILVER_DIFFUSE[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
const float SILVER_SPECULAR[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
const float DARK_BLUE[4] = { 0.3f, 0.3f, 0.5f, 1.0f };
const float BLACK[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
const float WHITE[4] = { 1.0f, 1.0f, 1.0f, 1.0f };

//static float GOLD_AMBIENT[4];
//static float GOLD_DIFFUSE[4];
//static float GOLD_SPECULAR[4];
//static float SILVER_AMBIENT[4];
//static float SILVER_DIFFUSE[4];
//static float SILVER_SPECULAR[4];
//static float DARK_BLUE[4];
//static float BLACK[4];
//static float WHITE[4];
/******************************************/

class Material
{
public:
	// Shapes material
	float g_MatAmbient[4];
	float g_MatDiffuse[4];
	float g_MatSpecular[4];
	float g_MatShininess;

	void set(const float * amb, const float *diff, const float *spe, const float shine);

	void set_material_GL();

};

#ifndef PREVIEW3D_NO_SHADERS
#include <shaders.h>
#endif

enum NormalUpdateMethod //added by wangyu
{
	NORMAL_UPDATE_NAIVE_RECOMPUTE,	// Simply recompute all normals
	NORMAL_UPDATE_PBS,				// Normals are updated using PBS rule
	NORMAL_UPDATE_ALL
};

class MeshDisplay : public MeshData
{

public:

	/********* Display Lists and VBOs*********/
	// OpenGL Display list of mesh
	int faces_display_list;
	// Display list without colors
	int lines_display_list;
	// Display list without colors
	int overlay_display_list;
	int isolines_display_list;
	/*********************************/

	/********* Display Options *********/
	

	double corner_threshold;
	bool show_overlay;
	bool show_overlay_depth;
	bool show_trackball;
	
	bool show_isolines;
	bool show_texture;
	int linewidth;

	double draw_displacement[3];
	float draw_rotation[4];

	bool show_faces;
	bool show_lines;
	bool show_vertid;
	bool show_faceid;
	bool invert_normals;
	

	NormalsType normals_type;
	igl::PerVertexNormalsWeightingType per_vertex_normal_type;//added by wangyu

	
	float line_color[4];
	int numIsoLevels;

	bool show_grid;

	bool has_wedge_texture;
	/***********************************/
	// Name of the texture to load
	std::string texFilename;

	GLuint texture_id;
	bool is_compiled;

	bool bFlipYCoord;

	typedef std::pair< IndexType, IndexType > vertexPair;
	typedef std::pair< vertexPair, double > isoPoint;

	std::vector<std::vector< std::pair< isoPoint, isoPoint > > > isoSegments;
	std::vector< double > isoLevels;

	/***********************************/
	double diameter;

	RowVector3 aabb_min;
	RowVector3 aabb_max;
	RowVector3 aabb_center;
	double avg_edge_length;

	ColorBarType colorBarType;

	bool use_glCallList;//the compile of callList of mesh vertices/normal/isoline could be very slow

	//
	bool normals_changed;//added by wangyu

	bool use_lighting_at_compile;
	NormalsType normals_type_at_compile;
	bool show_vertid_at_compile;
	bool show_faceid_at_compile;

	
public:

public:

	MeshDisplay();


	void preDraw(Light light);

	bool load_mesh_from_file(const char* mesh_file_name);

	bool load_texture_coordinates_from_file(const char* file_name);

	bool load_property_from_file(const char* file_name);

	// Compile mesh into opengl display list
	void compile_mesh();

	// Compile the overlay into opengl display list
	void compile_overlay();

	// Compile strictly quad mesh into opengl display list
	void compile_triangle_mesh();
	void compile_triangle_mesh_aux(bool color);

	// Compile strictly triangle mesh into opengl display list
	void compile_quad_mesh();
	void compile_quad_mesh_aux();
	// Compile strictly triangle mesh into opengl display list
	void compile_polygon_mesh();
	void compile_polygon_mesh_aux();

	/********* Mesh Computation Helpers *********/

	// Computes a color for each face
	static void compute_face_colors(
		const VectorX *face_property,
		Eigen::MatrixXd * face_color,
		const ColorBarType colorBarType);
	// Computes a color for each vertex
	static void compute_vertex_colors(
		const VectorX *vertex_property,
		Eigen::MatrixXd *vertex_colors,
		const ColorBarType colorBarType);
	// Compute bounding box and centroid
	static void compute_bounding_box(
		const Eigen::MatrixXd *vertices,
		RowVector3 & min_point,
		RowVector3 & max_point);
	// Compute bounding box and centroid
	static void compute_centroid(
		const Eigen::MatrixXd *vertices,
		RowVector3 & centroid);
	// Determines how much to zoom and shift such that the mesh fills the unit
	// box (centered at the origin)
	static void get_scale_and_shift_to_fit_mesh(
		const Eigen::MatrixXd *vertices,
		float & zoom,
		Eigen::Vector3f& shift);
	// Returns true if normals are inverted
	static bool test_for_inverted_normals(
		const Eigen::MatrixXd *vertices,
		const Eigen::MatrixXd *vertex_normals);

	void compute_isolevels();

	void resetTexCoords(bool reset_uv, bool reset_valid);

	// Computes surface normals
	void recompute_all_normals();
	void recompute_face_normals();
	void recompute_vertex_normals();
	void recompute_corner_normals();

	void update_vertices_in_GL();
	void compile_mesh_vertices();
	void setup_weights_for_normal_update(Eigen::MatrixXd Weights, Eigen::MatrixXd WeightsGradient);
	void draw_triangle_mesh_or_line(bool is_face, bool transparent = false, float alpha = 1.0f);

	void draw_mesh();
	void draw_grid();

	void draw_isolines();
	void draw_normals();
	void drawSelectedFace(int selected_face);

	void update_colors();

	void ResetColor(const float default_color[4]);

	/**************** Set and Get *****************/

	void set_corner_threshold(double value);
	double get_corner_threshold();
	void set_numIsoLevels(int value);
	int get_numIsoLevels();

	void SetTexFilename(const std::string fname);
	std::string GetTexFilename();

	void SetShowTexture(const bool value);
	bool GetShowTexture();

	void set_linewidth(int value);

	bool set_vertex_colors(const float default_color[4], const Eigen::VectorXi& new_color_indices, const Eigen::MatrixXd& new_vertex_colors, bool same_color = true);//new_vertex_colors could be modified
	bool set_face_colors(const float default_color[4], const Eigen::VectorXi& new_color_indices, const Eigen::MatrixXd& new_face_colors, bool same_color = true);//new_vertex_colors could be modified	
	bool set_face_colors_from_vertex(const Eigen::VectorXi& new_color_indices, const Eigen::MatrixXd& new_vertex_colors);

	void SetFlipYCoord(bool f);
	bool GetFlipYCoord() const
	{
		return bFlipYCoord;
	}

	void SetColorBarType(ColorBarType cbt)
	{
		colorBarType = cbt;
	}
	ColorBarType GetColorBarType() const
	{
		return colorBarType;
	}

	/********* Rendering Helpers *********/
	static std::string convertInt(int number);
	static std::string convertDouble(double number);

#ifndef PREVIEW3D_NO_SHADERS
public:
	// Selected the active shader
	enum ShaderMode {
		OFF,                   // Shaders are disabled
		DIRECTIONAL_PER_PIXEL  // Phong shader (it only supports 1 light and per-vertex color)
	};
#define NUM_SHADER_MODE 2
#endif

	// Shaders
#ifndef PREVIEW3D_NO_SHADERS
	bool update_shader_attribs_every_frame;

	struct GLSL_Program s_directionalPerPixelProgram;
	struct GLSL_Program s_directionalPerPixelColorProgram;
	struct GLSL_Program s_pbsShaderProgram;
	ShaderMode shader_mode;
	int shader_id;
#endif

#ifndef PREVIEW3D_NO_SHADERS
	void set_shader_mode(const ShaderMode id, const bool use_lighting);
	ShaderMode get_shader_mode();

	void reload_shader();//added by wangyu
	void load_shader();
#endif

	/****************/


	/********* Operations for Mesh *********/

	void draw_colors_on_mesh(Eigen::VectorXd& new_color);

	void SetMeshDrawingType(MeshDrawingType mt)
	{
		meshDrawingType = mt;
	}
	MeshDrawingType GetMeshDrawingType() const
	{
		return meshDrawingType;
	}
	MeshDrawingType meshDrawingType;

#ifdef USE_MATLAB
	bool send_status_to_matlab() const;
	bool send_color_to_matlab() const;
#endif

	void update_normal(NormalUpdateMethod method);

};

#endif /*MESH_H*/