#include "Mesh.h"


#ifdef __APPLE__
#   include <OpenGL/gl.h>
#   include <OpenGL/glu.h>
#   include <GLUT/glut.h>
#else
#   ifdef _WIN32
#       include <windows.h>
#       include <GL/glew.h>
#       include <GL/glut.h>
#   endif
#   include <GL/gl.h>
#   include <GL/glu.h>
#endif

#include <cmath>
#include <cstdio>
#include <string>
#include <sstream>
#include <iomanip>

#include <algorithm>
using namespace std;

//#include "igl/vf.h"
#include <vf.h>//wangyu this is removed from igl
#include "igl/adjacency_list.h"


#include "igl/per_face_normals.h"
#include "igl/per_vertex_normals.h"
#include "igl/per_corner_normals.h"

#include <draw_primitives.h>//wangyu

#include "igl/timer.h"
#include "igl/png/render_to_png.h"//wangyu
#include "igl/readDMAT.h" //wangyu

#include <utils/texture_png.h>//wangyu
#include "utils/stb_image_write.h"//added by wangyu, file got from tiantian

#include "math_helper.h"

// Undef Visual Studio macros...
#undef max
#undef min


void draw_text(double x, double y, double z, string str, bool bsmall)
{
	const char *c = str.c_str();
	glRasterPos3f(x, y, z);
	for (; *c != '\0'; c++) {
		glutBitmapCharacter(bsmall ? GLUT_BITMAP_TIMES_ROMAN_10 : GLUT_BITMAP_TIMES_ROMAN_24, *c);
	}
}

void Material::set(const float * amb, const float *diff, const float *spe, const float shine)
{
	CopyArray4(amb, g_MatAmbient);
	CopyArray4(diff, g_MatDiffuse);
	CopyArray4(spe, g_MatSpecular);
	g_MatShininess = shine;
}

void Material::set_material_GL()
{
	// Set material
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, g_MatAmbient);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, g_MatDiffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, g_MatSpecular);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, g_MatShininess);
}

Light::Light()
{
	use_lighting = true;
	g_LightMultiplier = 1.0f;
	CopyArray3(DEFAULT_LIGHT_DIRECTION, g_LightDirection);
	g_LightModel_Ambient[0] = 0.2;
	g_LightModel_Ambient[1] = 0.2;
	g_LightModel_Ambient[2] = 0.2;
	g_LightModel_Ambient[3] = 1.0;
}

float Light::DEFAULT_LIGHT_DIRECTION[] = { -0.50f, -0.40f, -0.75f };

void Light::set_lighting_GL()
{
	// Set light
	if (use_lighting)
	{
		// will be used to set light paramters
		float v[4];
		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);

		glLightModelfv(GL_LIGHT_MODEL_AMBIENT, g_LightModel_Ambient);
		//glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,1);//added by wangyu

		v[0] = v[1] = v[2] = g_LightMultiplier*0.4f; v[3] = 1.0f;
		glLightfv(GL_LIGHT0, GL_AMBIENT, v);
		v[0] = v[1] = v[2] = g_LightMultiplier*0.8f; v[3] = 1.0f;
		glLightfv(GL_LIGHT0, GL_DIFFUSE, v);
		v[0] = -g_LightDirection[0]; v[1] = -g_LightDirection[1]; v[2] = -g_LightDirection[2]; v[3] = 0.0f;
		glLightfv(GL_LIGHT0, GL_POSITION, v);
	}
	else
	{
		glDisable(GL_LIGHTING);
	}
}




MeshDisplay::MeshDisplay(): MeshData()
{
	corner_threshold = 20;
	// Causes gldeletelists to return error on first call
	faces_display_list = -1;

	is_compiled = false;
	show_faces = true;
	show_lines = true;
	invert_normals = false;
	show_overlay = true;
	show_overlay_depth = true;
	show_vertid = false;
	show_faceid = false;

	draw_displacement[0]
		= draw_displacement[1]
		= draw_displacement[2]
		= 0.0;
	
	draw_rotation[0] = 0.0f;
	draw_rotation[1] = 0.0f;
	draw_rotation[2] = 0.0f;
	draw_rotation[3] = 1.0f;

	show_trackball = false;
	numIsoLevels = 50;
	show_isolines = false;
	show_texture = false;

	texture_id = 0;
	linewidth = 1;

	// Display options
	colorBarType = COLORBAR_HIGHLIGHT_NEG;

	// Mesh Display
	use_glCallList = false;//added by wangyu
	bFlipYCoord = false;//added by wangyu

	normals_type = PER_VERTEX;//this is not used, wangyu
	per_vertex_normal_type = igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_AREA;

	normals_changed = false;

	CopyArray4(BLACK, line_color);

	show_grid = false;

#ifndef PREVIEW3D_NO_SHADERS
	meshDrawingType = MESH_DRAWING_DEFAULT;
	// Load shaders

	//load_shader();// this is disabled temply

	//s_directionalPerPixelProgram = loadShaderProgramStr(directionalperpixel_vert, directionalperpixel_frag);
	//s_directionalPerPixelColorProgram = loadShaderProgramStr(directionalperpixelcolor_vert, directionalperpixelcolor_frag);
	//s_pbsShaderProgram = loadShaderProgram(PBS_SHADER_VERT_PATH, PBS_SHADER_FRAG_PATH);
	shader_mode = OFF;
	shader_id = 0;

	//printShaderInfoLog(s_pbsShaderProgram.p);
	//printProgramInfoLog(s_pbsShaderProgram.p); 
#endif
};



//float GOLD_AMBIENT[4] = { 51.0 / 255.0, 43.0 / 255.0, 33.3 / 255.0, 1.0f };
//float GOLD_DIFFUSE[4] = { 255.0 / 255.0, 228.0 / 255.0, 58.0 / 255.0, 1.0f };
//float GOLD_SPECULAR[4] = { 255.0 / 255.0, 235.0 / 255.0, 80.0 / 255.0, 1.0f };
//float SILVER_AMBIENT[4] = { 0.2f, 0.2f, 0.2f, 1.0f };
//float SILVER_DIFFUSE[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
//float SILVER_SPECULAR[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
//float DARK_BLUE[4] = { 0.3f, 0.3f, 0.5f, 1.0f };
//float BLACK[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
//float WHITE[4] = { 1.0f, 1.0f, 1.0f, 1.0f };



void MeshDisplay::preDraw(Light light)
{
	// need to recompile if flat_shading has changed
	is_compiled = is_compiled &&
		(light.use_lighting == use_lighting_at_compile) &&
		(normals_type == normals_type_at_compile) &&
		(show_vertid == show_vertid_at_compile) &&
		(show_faceid == show_faceid_at_compile);

	if (!is_compiled)
	{
		use_lighting_at_compile = light.use_lighting;
		normals_type_at_compile = normals_type;
		show_vertid_at_compile = show_vertid;
		show_faceid_at_compile = show_faceid;

		compile_mesh();
		compile_overlay();
		is_compiled = true;
	}

	//added by wangyu
	if (normals_changed)
	{
		recompute_all_normals();
		normals_changed = false;
	}
}

bool MeshDisplay::load_texture_coordinates_from_file(const char* file_name)
{
	if (!MeshData::load_texture_coordinates_from_file(file_name))
		return false;
	
	resetTexCoords(!texCoords->rows(), !fUseTexture.size());

	return true;
}

bool MeshDisplay::load_mesh_from_file(const char* mesh_file_name)
{
	if (!MeshData::load_mesh_from_file(mesh_file_name))
		return false;

	if (vertex_property->rows() != 0)
	{
		show_isolines = true;
	}

	show_faces = true;
	normals_type = PER_VERTEX;//wangyu PER_FACE

	is_compiled = false;

	if (face_normals->rows() == 0 || face_normals->cols() != 3)
		recompute_face_normals();

	if (vertex_normals->rows() == 0 || vertex_normals->cols() != 3)
		recompute_vertex_normals();

	std::vector<std::vector<IndexType> > vfi;
	//igl::vf(*vertices, *faces, vertex_to_faces, vfi);
	vf(*vertices, *faces, vertex_to_faces, vfi);
	igl::adjacency_list(*faces, vertex_to_vertices);
	if (corner_normals->rows() == 0 || corner_normals->cols() != 3 || fNormIndices->size() != faces->size())
		recompute_corner_normals();

	if (face_colors->rows() == 0 || face_colors->cols() != 3)
		compute_face_colors(face_property, face_colors, colorBarType);
	if (vertex_colors->rows() == 0 || vertex_colors->cols() != 3)
		compute_vertex_colors(vertex_property, vertex_colors, colorBarType);

	if (vertex_property->rows())
		compute_isolevels();

	resetTexCoords(!texCoords->rows(), !fUseTexture.size());

	// compute mesh dimension properties
	compute_bounding_box(vertices, aabb_min, aabb_max);
	compute_centroid(vertices, aabb_center);
	diameter = (aabb_min - aabb_max).norm();
	avg_edge_length = 0;
	for (int f = 0; f < faces->rows(); f++)
	{
		for (int b = 0, e = 2; b < 3; b++, e = b)
		{
			Eigen::Matrix<ScalarType, 1, 3> begin = vertices->row(faces->row(f)[b]);
			Eigen::Matrix<ScalarType, 1, 3> end = vertices->row(faces->row(f)[e]);
			avg_edge_length += (begin - end).norm();
		}
	}
	avg_edge_length /= faces->rows() * 3;

#ifndef PREVIEW3D_NO_SHADERS
	if (face_colors->rows() > 0)
	{
		shader_mode = DIRECTIONAL_PER_PIXEL;
		shader_id = 0;
	}
#endif  

	return true;
}


bool MeshDisplay::load_property_from_file(const char* file_name)
{
	Eigen::MatrixXd VP;
	igl::readDMAT(file_name, VP);
		
	*vertex_property = VP;

	if (face_colors->rows() == 0 || face_colors->cols() != 3)
		compute_face_colors(face_property, face_colors, colorBarType);
	if (vertex_colors->rows() == 0 || vertex_colors->cols() != 3)
		compute_vertex_colors(vertex_property, vertex_colors, colorBarType);

	if (vertex_property->rows())
		compute_isolevels();

	if (vertex_property->rows() != 0)
	{
		show_isolines = true;
	}

	return true;
}

void MeshDisplay::recompute_all_normals()
{
	recompute_face_normals();
	recompute_vertex_normals();
	recompute_corner_normals();
}

void MeshDisplay::recompute_face_normals()
{
	igl::per_face_normals(*vertices, *faces, *face_normals);
}

void MeshDisplay::recompute_vertex_normals()
{
	igl::per_vertex_normals(*vertices, *faces, per_vertex_normal_type, *face_normals, *vertex_normals);
	invert_normals = test_for_inverted_normals(vertices, vertex_normals);
}

void MeshDisplay::recompute_corner_normals()
{
	igl::per_corner_normals(*vertices, *faces, *face_normals, vertex_to_faces, corner_threshold, *corner_normals);
	fNormIndices->resize(faces->rows(), faces->cols());
	int index = 0;
	for (int i = 0; i < faces->rows(); ++i)
		for (int j = 0; j < faces->cols(); ++j)
			(*fNormIndices)(i, j) = index++;
}


void MeshDisplay::resetTexCoords(bool reset_uv, bool reset_valid)
{
	if (fTexIndices->size() != faces->size())
	{
		fTexIndices->resize(faces->rows(), faces->cols());
		for (int fi = 0; fi < faces->rows(); ++fi)
			fTexIndices->row(fi) = faces->row(fi);
		has_wedge_texture = false;
	}
	else
		has_wedge_texture = (*faces != *fTexIndices);

	if (reset_uv)
	{
		// use vertex and faces as texture coordinates and corner texture indices
		if (!copyXY(vertices, texCoords))
		{
			exit(-1);
		}
		// flip Y coordinate since we usually think of images with reversed Y
		// direction
		if (bFlipYCoord)
			flipCoord(texCoords, 1, texCoords);
		normalize(texCoords, texCoords);
		fprintf(stderr,
			"WARNING: texture coordinates not given."
			" Using vertex (X,max(Y)-Y-min(Y)) positions as texture coordinates.\n");

		if (!normalized(texCoords))
		{
			normalize(texCoords, texCoords);
			fprintf(stderr,
				"WARNING: texture coordinates are not between 0 and 1."
				" Normalizing them so that they are.\n");
		}
	}
	if (reset_valid)
	{
		fUseTexture.clear();
		fUseTexture.resize(faces->rows(), true);
	}
	is_compiled = false;
}

void MeshDisplay::compute_isolevels()
{
	if (vertex_property->rows() == vertices->rows()
		&& vertex_property->rows() > 0)//wangyu
	{
		glDeleteLists(isolines_display_list, 1);

		double min_val = vertex_property->minCoeff();
		double max_val = vertex_property->maxCoeff();

		numIsoLevels = std::max(1, std::min(numIsoLevels, 5000));

		isoLevels.clear();
		isoLevels.resize(numIsoLevels);
		isoSegments.clear();
		isoSegments.resize(numIsoLevels, std::vector< std::pair< isoPoint, isoPoint > >(0));

		std::vector< std::vector< double > > isocolors;
		isocolors.resize(numIsoLevels);

		for (int i = 0; i < numIsoLevels - 1; ++i)
		{
			double val = (1.0 *i) / (numIsoLevels - 1);
			isoLevels[i] = min_val + (max_val - min_val) * val;
			isocolors[i].resize(3);
			isocolors[i][0] = 1.;
			isocolors[i][1] = 0.;
			isocolors[i][2] = 0.;
		}

		isoLevels[numIsoLevels - 1] = max_val;
		isocolors[numIsoLevels - 1].resize(3);
		isocolors[numIsoLevels - 1][0] = 1.;
		isocolors[numIsoLevels - 1][1] = 0.;
		isocolors[numIsoLevels - 1][2] = 0.;

		for (int i = 0; i < faces->rows(); ++i)
		{
			double t1, t2, p0, p1, p2;
			size_t v0, v1, v2;

			for (int j = 0; j < faces->cols(); j++)
			{
				v0 = (*faces)(i, j);
				p0 = (*vertex_property)(v0, 0);

				v1 = (*faces)(i, (j + 1) % (faces->cols()));
				v2 = (*faces)(i, (j - 1 + faces->cols()) % (faces->cols()));

				p1 = (*vertex_property)(v1, 0);
				p2 = (*vertex_property)(v2, 0);

				isoPoint iso1, iso2;
				for (int l = 0; l < numIsoLevels; ++l)
				{
					double level = isoLevels[l];
					if (((p0 >= level && p1 <= level) || (p1 >= level && p0 <= level)) && (p0 != p1))
					{
						t1 = (p1 - level) / (p1 - p0);
						vertexPair v = std::make_pair(v0, v1);
						iso1 = std::make_pair(v, t1);

						if (((p0 >= level && p2 <= level) || (p2 >= level && p0 <= level)) && (p0 != p2))
						{
							t2 = (p2 - level) / (p2 - p0);
							vertexPair v = std::make_pair(v0, v2);
							iso2 = std::make_pair(v, t2);
							isoSegments[l].push_back(std::make_pair(iso1, iso2));
						}
					}
				}
			}
		}

		glDeleteLists(isolines_display_list, 1);

		// generate new display list
		isolines_display_list = glGenLists(1);
		glNewList(isolines_display_list, GL_COMPILE);
		float linewidth;
		glGetFloatv(GL_LINE_WIDTH, &linewidth);
		glLineWidth(2);
		for (int l = 0; l < numIsoLevels; ++l)
		{
			glColor3d(isocolors[l][0], isocolors[l][1], isocolors[l][2]);
			for (unsigned int i = 0; i < isoSegments[l].size(); ++i)
			{
				isoPoint iso1 = isoSegments[l][i].first;
				isoPoint iso2 = isoSegments[l][i].second;

				double p1[3], p2[3], t;
				vertexPair vp;

				t = iso1.second;
				vp = iso1.first;
				p1[0] = t*(*vertices)(vp.first, 0) + (1 - t)*(*vertices)(vp.second, 0);
				p1[1] = t*(*vertices)(vp.first, 1) + (1 - t)*(*vertices)(vp.second, 1);
				p1[2] = t*(*vertices)(vp.first, 2) + (1 - t)*(*vertices)(vp.second, 2);

				t = iso2.second;
				vp = iso2.first;
				p2[0] = t*(*vertices)(vp.first, 0) + (1 - t)*(*vertices)(vp.second, 0);
				p2[1] = t*(*vertices)(vp.first, 1) + (1 - t)*(*vertices)(vp.second, 1);
				p2[2] = t*(*vertices)(vp.first, 2) + (1 - t)*(*vertices)(vp.second, 2);

				glBegin(GL_LINES);
				glVertex3dv(p1);
				glVertex3dv(p2);
				glEnd();
			}
		}
		glLineWidth(linewidth);
		glEndList();
	}
}

void MeshDisplay::compile_mesh()
{
	if (invert_normals)
	{
		*vertex_normals = -(*vertex_normals);
		*face_normals = -(*face_normals);
		*corner_normals = -(*corner_normals);
		invert_normals = false;
	}

	if (faces->cols() == 3)
	{
		MeshDisplay::compile_triangle_mesh();
	}
	else if (faces->cols() == 4)
	{
		MeshDisplay::compile_quad_mesh();
	}
	else
	{
		MeshDisplay::compile_polygon_mesh();
	}
}

void MeshDisplay::compile_triangle_mesh_aux(bool color)
{
	glLineWidth(linewidth);


	glBegin(GL_TRIANGLES);
	// loop over all faces
	for (int face_index = 0; face_index < faces->rows(); face_index++)
	{
		if (show_texture && texture_id && fUseTexture[face_index])
			continue;

		if (color && face_colors->rows() > 0)
		{
			const ScalarType* color_data = face_colors->data() + 3 * face_index;
			glColor3f(color_data[0], color_data[1], color_data[2]);
		}
		// flat shading
		if (normals_type == PER_FACE)
		{

			const RowVector3 &normal = face_normals->row(face_index);
			glNormal3f(normal[0], normal[1], normal[2]);
		}
		// loop over vertices in this face
		for (int vit = 0; vit < faces->cols(); ++vit)
		{
			if (color && vertex_colors->rows() > 0)
			{
				const ScalarType* color_data = vertex_colors->data() + 3 * ((*faces)(face_index, vit));
				glColor3f(color_data[0], color_data[1], color_data[2]);
			}
			// not flat shading goes here
			if (normals_type == PER_VERTEX)
			{
				const RowVector3 &normal = vertex_normals->row((*faces)(face_index, vit));
				glNormal3f(normal[0], normal[1], normal[2]);
			}
			else
				if (normals_type == PER_CORNER)
				{
					const RowVector3 &normal = corner_normals->row((*fNormIndices)(face_index, vit));
					glNormal3f(normal[0], normal[1], normal[2]);
				}
			// assumes every point in vertices is length == 3
			const ScalarType *vertex_data = vertices->data() + 3 * (*faces)(face_index, vit);
			glVertex3f(vertex_data[0], vertex_data[1], vertex_data[2]);

		}
	}
	glEnd();

	if (!show_texture || !texture_id)
		return;

	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glEnable(GL_TEXTURE_2D);

	glBegin(GL_TRIANGLES);
	// loop over all faces
	for (int face_index = 0; face_index < faces->rows(); face_index++)
	{
		if (!fUseTexture[face_index])
			continue;

		if (color && face_colors->rows() > 0)
		{
			const ScalarType* color_data = face_colors->data() + 3 * face_index;
			glColor3f(color_data[0], color_data[1], color_data[2]);
		}
		// flat shading
		if (normals_type == PER_FACE)
		{

			const RowVector3 &normal = face_normals->row(face_index);
			glNormal3f(normal[0], normal[1], normal[2]);
		}
		int corner_index = 0;
		// loop over vertices in this face
		for (int vit = 0; vit < faces->cols(); ++vit, ++corner_index)
		{
			if (color && vertex_colors->rows() > 0)
			{
				const ScalarType* color_data = vertex_colors->data() + 3 * ((*faces)(face_index, vit));
				glColor3f(color_data[0], color_data[1], color_data[2]);
			}
			// not flat shading goes here
			if (normals_type == PER_VERTEX)
			{
				const RowVector3 &normal = vertex_normals->row((*faces)(face_index, vit));
				glNormal3f(normal[0], normal[1], normal[2]);
			}
			else
				if (normals_type == PER_CORNER)
				{
					const RowVector3 &normal = corner_normals->row((*fNormIndices)(face_index, vit));
					glNormal3f(normal[0], normal[1], normal[2]);
				}

			int ti = (*fTexIndices)(face_index, vit);
			double u = (*texCoords)(ti, 0);
			double v = (*texCoords)(ti, 1);
			glTexCoord2d(u, v);
			// assumes every point in vertices is length == 3
			const ScalarType *vertex_data = vertices->data() + 3 * (*faces)(face_index, vit);
			glVertex3f(vertex_data[0], vertex_data[1], vertex_data[2]);

		}
	}
	glEnd();

	glDisable(GL_TEXTURE_2D);

	glLineWidth(1);

}

void MeshDisplay::compile_triangle_mesh()
{
	// Delete old display list
	glDeleteLists(faces_display_list, 1);
	glDeleteLists(lines_display_list, 1);

	// generate new display list
	faces_display_list = glGenLists(1);

	glNewList(faces_display_list, GL_COMPILE);
	compile_triangle_mesh_aux(true);
	glEndList();

	lines_display_list = glGenLists(1);
	glNewList(lines_display_list, GL_COMPILE);
	compile_triangle_mesh_aux(false);
	glEndList();

}

void MeshDisplay::compile_overlay()
{
	// Delete old display list
	glDeleteLists(overlay_display_list, 1);

	// generate new display list
	overlay_display_list = glGenLists(1);

	glNewList(overlay_display_list, GL_COMPILE);

	// Lines
	glBegin(GL_LINES);
	for (unsigned int i = 0; i < lines.size(); ++i)
	{
		vector<double> l = lines[i];
		glColor3d(l[6], l[7], l[8]);
		glVertex3d(l[0], l[1], l[2]);
		glVertex3d(l[3], l[4], l[5]);
	}
	glEnd();

	// Points
	for (unsigned int i = 0; i < points.size(); ++i)
	{
		vector<double> p = points[i];
		glPointSize(p[3]);
		glColor3d(p[4], p[5], p[6]);
		glBegin(GL_POINTS);
		glVertex3d(p[0], p[1], p[2]);
		glEnd();
	}

	// Text
	for (unsigned int i = 0; i < textp.size(); ++i)
	{
		vector<double> p = textp[i];
		glColor3f(0, 0, 0);
		draw_text(p[0], p[1], p[2], texts[i], false);
	}

	if (show_vertid)
	{
		glColor3f(0, 0, 0);
		for (int i = 0; i < vertices->rows(); ++i)
		{
			draw_text((*vertices)(i, 0), (*vertices)(i, 1), (*vertices)(i, 2), convertInt(i + 1), false);
		}

	}

	if (show_faceid)
	{
		glColor3f(0, 0, 0);
		for (int i = 0; i < faces->rows(); ++i)
		{
			RowVector3 p = RowVector3::Zero();

			for (int j = 0; j < faces->cols(); ++j)
				p += vertices->row((*faces)(i, j));

			p /= faces->cols();

			draw_text(p[0], p[1], p[2], convertInt(i + 1), false);
		}

	}

	glEndList();

}

void MeshDisplay::compile_quad_mesh()
{
	// Delete old display list
	glDeleteLists(faces_display_list, 1);
	// generate new display list
	faces_display_list = glGenLists(1);

	glNewList(faces_display_list, GL_COMPILE);
	compile_quad_mesh_aux();
	glEndList();

	lines_display_list = glGenLists(1);
	glNewList(lines_display_list, GL_COMPILE);
	compile_quad_mesh_aux();
	glEndList();
}

void MeshDisplay::compile_quad_mesh_aux()
{
	glLineWidth(linewidth);

	glBegin(GL_QUADS);
	// loop over all faces
	int face_index = 0;
	for (face_index = 0; face_index < faces->rows(); face_index++)
	{
		// flat shading
		if (normals_type == PER_FACE)
		{
			const RowVector3 &normal = face_normals->row(face_index);
			glNormal3f(normal[0], normal[1], normal[2]);
		}
	{
		// loop over vertices in this face
		for (int vit = 0; vit < faces->cols(); ++vit)
		{
			// not flat shading goes here
			if (normals_type == PER_VERTEX)
			{
				const RowVector3 &normal = (invert_normals ? -1 : 1) * vertex_normals->row((*faces)(face_index, vit));
				glNormal3f(normal[0], normal[1], normal[2]);
			}
			else
				if (normals_type == PER_CORNER)
				{
					const RowVector3 &normal = corner_normals->row((*fNormIndices)(face_index, vit));
					glNormal3f(normal[0], normal[1], normal[2]);
				}
			// assumes every point in vertices is length == 3
			const ScalarType *vertex_data = vertices->data() + 3 * (*faces)(face_index, vit);
			glVertex3f(vertex_data[0], vertex_data[1], vertex_data[2]);

		}
	}
	}
	glEnd();
	glLineWidth(1);

}

void MeshDisplay::compile_polygon_mesh()
{
	// Delete old display list
	glDeleteLists(faces_display_list, 1);
	// generate new display list
	faces_display_list = glGenLists(1);

	glNewList(faces_display_list, GL_COMPILE);
	compile_polygon_mesh_aux();
	glEndList();

	lines_display_list = glGenLists(1);
	glNewList(lines_display_list, GL_COMPILE);
	compile_polygon_mesh_aux();
	glEndList();
}

void MeshDisplay::compile_polygon_mesh_aux()
{
	glLineWidth(linewidth);

	// loop over all faces
	int face_index = 0;
	for (face_index = 0; face_index < faces->rows(); face_index++)
	{
		// flat shading
		if (normals_type == PER_FACE)
		{
			const RowVector3 &normal = face_normals->row(face_index);
			glNormal3f(normal[0], normal[1], normal[2]);
		}
	{
		glBegin(GL_POLYGON);
		// loop over vertices in this face
		for (int vit = 0; vit < faces->cols(); ++vit)
		{
			// not flat shading goes here
			if (normals_type == PER_VERTEX)
			{
				const RowVector3 &normal = vertex_normals->row((*faces)(face_index, vit));
				glNormal3f(normal[0], normal[1], normal[2]);
			}
			else
				if (normals_type == PER_CORNER)
				{
					const RowVector3 &normal = corner_normals->row((*fNormIndices)(face_index, vit));
					glNormal3f(normal[0], normal[1], normal[2]);
				}

			// assumes every point in vertices is length == 3
			const ScalarType *vertex_data = vertices->data() + 3 * (*faces)(face_index, vit);
			glVertex3f(vertex_data[0], vertex_data[1], vertex_data[2]);

		}
		glEnd();
	}
	}
	glLineWidth(1);
}

void MeshDisplay::compute_vertex_colors(
	const VectorX *vertex_property,
	Eigen::MatrixXd *vertex_colors,
	const ColorBarType colorBarType)
{
	per_attrib_colors(vertex_property, vertex_colors, colorBarType);
}

void MeshDisplay::compute_face_colors(
	const VectorX *face_property,
	Eigen::MatrixXd * face_colors,
	const ColorBarType colorBarType)
{
	per_attrib_colors(face_property, face_colors, colorBarType);
}


void MeshDisplay::compute_bounding_box(
	const Eigen::MatrixXd *vertices,
	RowVector3 & min_point,
	RowVector3 & max_point)
{
	min_point = vertices->colwise().minCoeff();
	max_point = vertices->colwise().maxCoeff();
}

void MeshDisplay::compute_centroid(
	const Eigen::MatrixXd *vertices,
	RowVector3 & centroid)
{
	if(vertices->rows()&&vertices->cols())
		//centroid = vertices->colwise().sum()/vertices->rows();
		centroid = (vertices->colwise().maxCoeff() + vertices->colwise().minCoeff()) / 2.0;
}


void MeshDisplay::get_scale_and_shift_to_fit_mesh(
	const Eigen::MatrixXd *vertices,
	float& zoom,
	Eigen::Vector3f& shift)
{
	//Compute mesh centroid
	RowVector3 centroid;
	RowVector3 min_point;
	RowVector3 max_point;
	compute_bounding_box(vertices, min_point, max_point);
	compute_centroid(vertices, centroid);

	shift[0] = -centroid[0];
	shift[1] = -centroid[1];
	shift[2] = -centroid[2];
	double x_scale = fabs(max_point[0] - min_point[0]);
	double y_scale = fabs(max_point[1] - min_point[1]);
	double z_scale = fabs(max_point[2] - min_point[2]);
	zoom = 2.0 / std::max(z_scale, std::max(x_scale, y_scale));
}

bool MeshDisplay::test_for_inverted_normals(
	const Eigen::MatrixXd *vertices,
	const Eigen::MatrixXd *vertex_normals)
{
	RowVector3 centroid;
	RowVector3 min_point;
	RowVector3 max_point;
	compute_bounding_box(vertices, min_point, max_point);
	compute_centroid(vertices, centroid);

	double average_dot_product = 0.0;
	// loop over vertices
	for (int vi = 0; vi < vertices->rows(); ++vi)
	{
		// take dot product of unit displacement vector and normal
		RowVector3 unit_displacement = vertices->row(vi) - centroid;
		unit_displacement.normalize();

		double dot_product = vertex_normals->row(vi).dot(unit_displacement);
		average_dot_product += dot_product;
	}
	return average_dot_product < 0;
}

std::string MeshDisplay::convertInt(int number)
{
	std::stringstream ss;//create a stringstream
	ss << number;//add number to the stream
	return ss.str();//return a string with the contents of the stream
}

std::string MeshDisplay::convertDouble(double number)
{
	std::stringstream ss;//create a stringstream
	ss << std::setprecision(4) << number;//add number to the stream
	return ss.str();//return a string with the contents of the stream
}

void MeshDisplay::update_vertices_in_GL()
{

	// Delete old display list
	//glDeleteLists(faces_display_list, 1);
	//glDeleteLists(lines_display_list, 1);

	// generate new display list
	//faces_display_list = glGenLists(1);

	//glNewList(faces_display_list, GL_COMPILE);
	compile_mesh_vertices();
	//glEndList();

	//lines_display_list = glGenLists(1);
	glNewList(lines_display_list, GL_COMPILE);
	compile_mesh_vertices();
	glEndList();
	//bool color = false;

	//glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	//glEnable(GL_TEXTURE_2D);

	//glBegin(GL_TRIANGLES);
	//// loop over all faces
	//for(int face_index = 0; face_index<faces->rows();face_index++)
	//{
	//	if(!fUseTexture[face_index])
	//		continue;

	//	if (color && face_colors->rows() > 0)
	//	{
	//		const ScalarType* color_data = face_colors->data() + 3 * face_index;
	//		glColor3f(color_data[0], color_data[1], color_data[2]);
	//	}
	//	// flat shading
	//	if(normals_type == PER_FACE)
	//	{

	//		const RowVector3 &normal = face_normals->row(face_index);
	//		glNormal3f(normal[0], normal[1], normal[2]);
	//	}
	//	int corner_index = 0;
	//	// loop over vertices in this face
	//	for(int vit = 0; vit < faces->cols(); ++vit, ++corner_index)
	//	{
	//		if (color && vertex_colors->rows() > 0)
	//		{
	//			const ScalarType* color_data = vertex_colors->data() + 3 * ( (*faces)(face_index,vit) );
	//			glColor3f(color_data[0], color_data[1], color_data[2]);
	//		}
	//		// not flat shading goes here
	//		if(normals_type == PER_VERTEX)
	//		{
	//			const RowVector3 &normal = vertex_normals->row((*faces)(face_index,vit));
	//			glNormal3f(normal[0], normal[1], normal[2]);
	//		}
	//		else
	//			if (normals_type == PER_CORNER)
	//			{
	//				const RowVector3 &normal = corner_normals->row((*fNormIndices)(face_index,vit));
	//				glNormal3f(normal[0], normal[1], normal[2]);
	//			}

	//			int ti = (*fTexIndices)(face_index,vit);
	//			double u = (*texCoords)(ti,0);
	//			double v = (*texCoords)(ti,1);
	//			glTexCoord2d(u,v);
	//			// assumes every point in vertices is length == 3
	//			const ScalarType *vertex_data = vertices->data() + 3* (*faces)(face_index,vit);
	//			glVertex3f(vertex_data[0], vertex_data[1], vertex_data[2]);

	//	}
	//}
	//glEnd();

	//glDisable(GL_TEXTURE_2D);  
}

void MeshDisplay::compile_mesh_vertices()
{
	bool color = false;

	glLineWidth(linewidth);


	glBegin(GL_TRIANGLES);
	// loop over all faces
	for (int face_index = 0; face_index < faces->rows(); face_index++)
	{
		if (show_texture && texture_id && fUseTexture[face_index])
			continue;

		//if (color && face_colors->rows() > 0)
		//{
		//	const ScalarType* color_data = face_colors->data() + 3 * face_index;
		//	glColor3f(color_data[0], color_data[1], color_data[2]);
		//}
		//// flat shading
		//if(normals_type == PER_FACE)
		//{

		//	const RowVector3 &normal = face_normals->row(face_index);
		//	glNormal3f(normal[0], normal[1], normal[2]);
		//}
		// loop over vertices in this face
		for (int vit = 0; vit < faces->cols(); ++vit)
		{
			//if (color && vertex_colors->rows() > 0)
			//{
			//	const ScalarType* color_data = vertex_colors->data() + 3 * ( (*faces)(face_index,vit) );
			//	glColor3f(color_data[0], color_data[1], color_data[2]);
			//}
			//// not flat shading goes here
			//if(normals_type == PER_VERTEX)
			//{
			//	const RowVector3 &normal = vertex_normals->row((*faces)(face_index,vit));
			//	glNormal3f(normal[0], normal[1], normal[2]);
			//}
			//else
			//	if (normals_type == PER_CORNER)
			//	{
			//		const RowVector3 &normal = corner_normals->row((*fNormIndices)(face_index,vit));
			//		glNormal3f(normal[0], normal[1], normal[2]);
			//	}
			//	// assumes every point in vertices is length == 3
			const ScalarType *vertex_data = vertices->data() + 3 * (*faces)(face_index, vit);
			glVertex3f(vertex_data[0], vertex_data[1], vertex_data[2]);

		}
	}
	glEnd();

	if (!show_texture || !texture_id)
		return;

	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glEnable(GL_TEXTURE_2D);

	glBegin(GL_TRIANGLES);
	// loop over all faces
	//for(int face_index = 0; face_index<faces->rows();face_index++)
	//{
	//	if(!fUseTexture[face_index])
	//		continue;

	//	if (color && face_colors->rows() > 0)
	//	{
	//		const ScalarType* color_data = face_colors->data() + 3 * face_index;
	//		glColor3f(color_data[0], color_data[1], color_data[2]);
	//	}
	//	// flat shading
	//	if(normals_type == PER_FACE)
	//	{

	//		const RowVector3 &normal = face_normals->row(face_index);
	//		glNormal3f(normal[0], normal[1], normal[2]);
	//	}
	//	int corner_index = 0;
	//	// loop over vertices in this face
	//	for(int vit = 0; vit < faces->cols(); ++vit, ++corner_index)
	//	{
	//		if (color && vertex_colors->rows() > 0)
	//		{
	//			const ScalarType* color_data = vertex_colors->data() + 3 * ( (*faces)(face_index,vit) );
	//			glColor3f(color_data[0], color_data[1], color_data[2]);
	//		}
	//		// not flat shading goes here
	//		if(normals_type == PER_VERTEX)
	//		{
	//			const RowVector3 &normal = vertex_normals->row((*faces)(face_index,vit));
	//			glNormal3f(normal[0], normal[1], normal[2]);
	//		}
	//		else
	//			if (normals_type == PER_CORNER)
	//			{
	//				const RowVector3 &normal = corner_normals->row((*fNormIndices)(face_index,vit));
	//				glNormal3f(normal[0], normal[1], normal[2]);
	//			}

	//			int ti = (*fTexIndices)(face_index,vit);
	//			double u = (*texCoords)(ti,0);
	//			double v = (*texCoords)(ti,1);
	//			glTexCoord2d(u,v);
	//			// assumes every point in vertices is length == 3
	//			const ScalarType *vertex_data = vertices->data() + 3* (*faces)(face_index,vit);
	//			glVertex3f(vertex_data[0], vertex_data[1], vertex_data[2]);

	//	}
	//}
	glEnd();

	glDisable(GL_TEXTURE_2D);

	glLineWidth(1);

}

void MeshDisplay::draw_triangle_mesh_or_line(bool is_face, bool transparent, float alpha)
{

	bool color = is_face;

	glLineWidth(linewidth);


	if (transparent)
	{
		glEnable(GL_BLEND);
		glDisable(GL_LIGHTING);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}
	// Push Current Color
	float currentColor[4];
	glGetFloatv(GL_CURRENT_COLOR, currentColor);

	glPushMatrix();
	float mat[4 * 4]; // rotation matrix
	ConvertQuaternionToMatrix(draw_rotation, mat);
	glMultMatrixf(mat);
	//glRotatef(draw_rotation[0], draw_rotation[1], draw_rotation[2], draw_rotation[3]);
	glTranslatef(draw_displacement[0], draw_displacement[1], draw_displacement[2]);

	glBegin(GL_TRIANGLES);

	// loop over all faces
	for (int face_index = 0; face_index < faces->rows(); face_index++)
	{
		if (show_texture && texture_id && fUseTexture[face_index])
			continue;

		if (color)
		{
			if (face_colors->rows() > 0)
			{
				glColor4f((*face_colors)(face_index,0), (*face_colors)(face_index, 1), (*face_colors)(face_index, 2), alpha);
			}
		}
		if (transparent)
		{
			glColor4f(currentColor[0], currentColor[1], currentColor[2], alpha);
		}
		// flat shading
		if (normals_type == PER_FACE)
		{
			const RowVector3 &normal = face_normals->row(face_index);
			glNormal3f(normal[0], normal[1], normal[2]);
		}
		// loop over vertices in this face
		for (int vit = 0; vit < faces->cols(); ++vit)
		{
			if (color)
			{
				if (vertex_colors->rows() > 0)
				{
					const RowVector3 color_data = vertex_colors->row((*faces)(face_index, vit));
					glColor4f(color_data[0], color_data[1], color_data[2], alpha);
				}
			}
			if (transparent)
			{
				glColor4f(currentColor[0], currentColor[1], currentColor[2], alpha);
			}
			// not flat shading goes here
			if (normals_type == PER_VERTEX)
			{
				const RowVector3 &normal = vertex_normals->row((*faces)(face_index, vit));
				glNormal3f(normal[0], normal[1], normal[2]);
			}
			else
				if (normals_type == PER_CORNER)
				{
					const RowVector3 &normal = corner_normals->row((*fNormIndices)(face_index, vit));
					glNormal3f(normal[0], normal[1], normal[2]);
				}
			// assumes every point in vertices is length == 3
			const RowVector3 vertex_data = vertices->row((*faces)(face_index, vit));
			glVertex3f(vertex_data[0], vertex_data[1], vertex_data[2]);

		}
	}

	glEnd();

	glPopMatrix();

	if (transparent)
	{
		glDisable(GL_BLEND);
	}

	// Popcurrent color
	glColor4f(currentColor[0], currentColor[1], currentColor[2], currentColor[3]);//wangyu

	if (!show_texture || !texture_id)
		return;

	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glEnable(GL_TEXTURE_2D);

	glPushMatrix();
	glMultMatrixf(mat);
	//glRotatef(draw_rotation[0], draw_rotation[1], draw_rotation[2], draw_rotation[3]);
	glTranslatef(draw_displacement[0], draw_displacement[1], draw_displacement[2]);

	glBegin(GL_TRIANGLES);
	// loop over all faces
	for (int face_index = 0; face_index < faces->rows(); face_index++)
	{
		if (!fUseTexture[face_index])
			continue;

		if (color && face_colors->rows() > 0)
		{
			const RowVector3 color_data = face_colors->row(face_index);
			glColor4f(color_data[0], color_data[1], color_data[2], alpha);
		}
		// flat shading
		if (normals_type == PER_FACE)
		{

			const RowVector3 &normal = face_normals->row(face_index);
			glNormal3f(normal[0], normal[1], normal[2]);
		}
		int corner_index = 0;
		// loop over vertices in this face
		for (int vit = 0; vit < faces->cols(); ++vit, ++corner_index)
		{
			if (color && vertex_colors->rows() > 0)
			{
				const RowVector3 color_data = vertex_colors->row((*faces)(face_index, vit));
				glColor4f(color_data[0], color_data[1], color_data[2], alpha);
			}
			// not flat shading goes here
			if (normals_type == PER_VERTEX)
			{
				const RowVector3 &normal = vertex_normals->row((*faces)(face_index, vit));
				glNormal3f(normal[0], normal[1], normal[2]);
			}
			else
				if (normals_type == PER_CORNER)
				{
					const RowVector3 &normal = corner_normals->row((*fNormIndices)(face_index, vit));
					glNormal3f(normal[0], normal[1], normal[2]);
				}

			int ti = (*fTexIndices)(face_index, vit);
			double u = (*texCoords)(ti, 0);
			double v = (*texCoords)(ti, 1);
			glTexCoord2d(u, v);
			// assumes every point in vertices is length == 3
			const RowVector3 vertex_data = vertices->row((*faces)(face_index, vit));
			glVertex3f(vertex_data[0], vertex_data[1], vertex_data[2]);

		}
	}
	glEnd();

	glPopMatrix();
	//glTranslatef(-draw_displacement[0], -draw_displacement[1], -draw_displacement[2]);
	// recover from the influence of draw_displacement

	glDisable(GL_TEXTURE_2D);

	glLineWidth(1);
}

void drawgrid(const RowVector3 &aabb_min,
	const RowVector3 &aabb_max,
	const RowVector3 &aabb_center)
{
	RowVector3 min = aabb_min;
	RowVector3 max = aabb_max;

	RowVector3 o;
	o[0] = aabb_center[0];
	o[1] = aabb_min[1] - (max[1] - min[1]) / 10;
	o[2] = aabb_center[2];


	double cells = 50;
	double cellSize = (max[1] - min[1]) / 10;

	glColor3f(0.5, 0.5, 0.5);

	// draw axis
	glLineWidth(3.0);
	glBegin(GL_LINES);
	glVertex3f(o[0] - cellSize*cells / 2, o[1], aabb_center[2]);
	glVertex3f(o[0] + cellSize*cells / 2, o[1], aabb_center[2]);
	glVertex3f(aabb_center[0], o[1], o[2] - cellSize*cells / 2);
	glVertex3f(aabb_center[0], o[1], o[2] + cellSize*cells / 2);
	glEnd();


	glLineWidth(1.0);
	// draw grid
	for (int i = 0; i < cells; ++i)
	{
		for (int j = 0; j < cells; ++j)
		{
			glBegin(GL_LINE_LOOP);
			glVertex3f(o[0] + cellSize*i - cellSize / 2 * cells, o[1], o[2] + cellSize*j - cellSize / 2 * cells);
			glVertex3f(o[0] + cellSize*(i + 1) - cellSize / 2 * cells, o[1], o[2] + cellSize*j - cellSize / 2 * cells);
			glVertex3f(o[0] + cellSize*(i + 1) - cellSize / 2 * cells, o[1], o[2] + cellSize*(j + 1) - cellSize / 2 * cells);
			glVertex3f(o[0] + cellSize*i - cellSize / 2 * cells, o[1], o[2] + cellSize*(j + 1) - cellSize / 2 * cells);
			glEnd();
		}
	}
}

void MeshDisplay::draw_grid()
{
	// draw grid
	if (show_grid && (aabb_min.size() == 3))
	{
		// Push GL Settings
		GLboolean old_lighting;
		glGetBooleanv(GL_LIGHTING, &old_lighting);
		glDisable(GL_LIGHTING);

		drawgrid(aabb_min, aabb_max, aabb_center);

		// Pop GL Settings
		if (old_lighting)
			glEnable(GL_LIGHTING);
	}
}

void MeshDisplay::draw_mesh()
{
	if (show_faces)
	{
		glEnable(GL_POLYGON_OFFSET_FILL); // Avoid Stitching!
		glPolygonOffset(1.0, 1.0);
		if (texture_id)
			glBindTexture(GL_TEXTURE_2D, texture_id);
		if (use_glCallList)//wangyu 
		{
			glCallList(faces_display_list);
		}
		else
		{
			draw_triangle_mesh_or_line(true);

			//draw_triangle_mesh_or_line_with_shader(true, 
			//		DeformSkinning::GetReference().W,
			//		DeformSkinning::GetReference().WI,
			//		DeformSkinning::GetReference().sorted_alphaFactors,
			//		DeformSkinning::GetReference().sorted_betaFactors,
			//		shader_id_for_face_and_line);
		}
		glDisable(GL_POLYGON_OFFSET_FILL);
	}

	if (show_lines)
	{
		if (show_faces)
		{
			glDisable(GL_LIGHTING);
			glColor3fv(line_color);
		}
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		if (use_glCallList)//wangyu 
		{
			glCallList(lines_display_list);//wangyu
		}
		else
		{
			draw_triangle_mesh_or_line(false);
			//draw_triangle_mesh_or_line_with_shader(false,
			//	DeformSkinning::GetReference().W,
			//	DeformSkinning::GetReference().WI,
			//	shader_id_for_face_and_line);
		}
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}

	if (show_isolines && vertex_property->rows())
	{
		if (show_faces)
		{
			glDisable(GL_LIGHTING);
			glColor3fv(line_color);
		}

		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		if (use_glCallList)//wangyu 
		{
			glCallList(isolines_display_list);//wangyu
		}
		else
		{
			draw_isolines();
		}
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}

	if (show_overlay)
	{
		if (!show_overlay_depth)
			glDisable(GL_DEPTH_TEST);

		// Push GL settings
		GLboolean was_use_lighting;
		glGetBooleanv(GL_LIGHTING, &was_use_lighting);

		glDisable(GL_LIGHTING);

		glCallList(overlay_display_list);

		glEnable(GL_DEPTH_TEST);

		if (was_use_lighting)
			glEnable(GL_LIGHTING);
		else
			glDisable(GL_LIGHTING);
	}

	if (false)//show_normals
	{
		draw_normals();
	}
}

void MeshDisplay::draw_normals()
{
	glLineWidth(0.5);
	glColor3f(0.0, 0.0, 1.0);
	glBegin(GL_LINES);

	for (int i = 0; i < vertices->rows(); i++)
	{
		Vector3 from((*vertices)(i, 0), (*vertices)(i, 1), (*vertices)(i, 2));
		Vector3 to((*vertex_normals)(i, 0), (*vertex_normals)(i, 1), (*vertex_normals)(i, 2));
		to = to*0.02 + from;
		//paintArrow(from,to,2);
		glVertex3f(from(0), from(1), from(2));
		glVertex3f(to(0), to(1), to(2));
	}

	glEnd();
}

void MeshDisplay::draw_isolines()
{
	if (vertex_property->rows() == vertices->rows())
	{
		//glDeleteLists(isolines_display_list, 1);

		double min_val = vertex_property->minCoeff();
		double max_val = vertex_property->maxCoeff();

		numIsoLevels = std::max(1, std::min(numIsoLevels, 5000));

		isoLevels.clear();
		isoLevels.resize(numIsoLevels);
		isoSegments.clear();
		isoSegments.resize(numIsoLevels, std::vector< std::pair< isoPoint, isoPoint > >(0));

		std::vector< std::vector< double > > isocolors;
		isocolors.resize(numIsoLevels);

		for (int i = 0; i < numIsoLevels - 1; ++i)
		{
			double val = (1.0 *i) / (numIsoLevels - 1);
			isoLevels[i] = min_val + (max_val - min_val) * val;
			isocolors[i].resize(3);
			isocolors[i][0] = 1.;
			isocolors[i][1] = 0.;
			isocolors[i][2] = 0.;
		}

		isoLevels[numIsoLevels - 1] = max_val;
		isocolors[numIsoLevels - 1].resize(3);
		isocolors[numIsoLevels - 1][0] = 1.;
		isocolors[numIsoLevels - 1][1] = 0.;
		isocolors[numIsoLevels - 1][2] = 0.;

		for (int i = 0; i < faces->rows(); ++i)
		{
			double t1, t2, p0, p1, p2;
			size_t v0, v1, v2;

			for (int j = 0; j < faces->cols(); j++)
			{
				v0 = (*faces)(i, j);
				p0 = (*vertex_property)(v0, 0);

				v1 = (*faces)(i, (j + 1) % (faces->cols()));
				v2 = (*faces)(i, (j - 1 + faces->cols()) % (faces->cols()));

				p1 = (*vertex_property)(v1, 0);
				p2 = (*vertex_property)(v2, 0);

				isoPoint iso1, iso2;
				for (int l = 0; l < numIsoLevels; ++l)
				{
					double level = isoLevels[l];
					if (((p0 >= level && p1 <= level) || (p1 >= level && p0 <= level)) && (p0 != p1))
					{
						t1 = (p1 - level) / (p1 - p0);
						vertexPair v = std::make_pair(v0, v1);
						iso1 = std::make_pair(v, t1);

						if (((p0 >= level && p2 <= level) || (p2 >= level && p0 <= level)) && (p0 != p2))
						{
							t2 = (p2 - level) / (p2 - p0);
							vertexPair v = std::make_pair(v0, v2);
							iso2 = std::make_pair(v, t2);
							isoSegments[l].push_back(std::make_pair(iso1, iso2));
						}
					}
				}
			}
		}

		//glDeleteLists(isolines_display_list, 1);

		// generate new display list
		//isolines_display_list = glGenLists(1);
		//glNewList(isolines_display_list, GL_COMPILE);
		float linewidth;
		glGetFloatv(GL_LINE_WIDTH, &linewidth);
		glLineWidth(2);
		for (int l = 0; l < numIsoLevels; ++l)
		{
			glColor3d(isocolors[l][0], isocolors[l][1], isocolors[l][2]);
			for (unsigned int i = 0; i < isoSegments[l].size(); ++i)
			{
				isoPoint iso1 = isoSegments[l][i].first;
				isoPoint iso2 = isoSegments[l][i].second;

				double p1[3], p2[3], t;
				vertexPair vp;

				t = iso1.second;
				vp = iso1.first;
				p1[0] = t*(*vertices)(vp.first, 0) + (1 - t)*(*vertices)(vp.second, 0);
				p1[1] = t*(*vertices)(vp.first, 1) + (1 - t)*(*vertices)(vp.second, 1);
				p1[2] = t*(*vertices)(vp.first, 2) + (1 - t)*(*vertices)(vp.second, 2);

				t = iso2.second;
				vp = iso2.first;
				p2[0] = t*(*vertices)(vp.first, 0) + (1 - t)*(*vertices)(vp.second, 0);
				p2[1] = t*(*vertices)(vp.first, 1) + (1 - t)*(*vertices)(vp.second, 1);
				p2[2] = t*(*vertices)(vp.first, 2) + (1 - t)*(*vertices)(vp.second, 2);

				glBegin(GL_LINES);
				glVertex3dv(p1);
				glVertex3dv(p2);
				glEnd();
			}
		}
		glLineWidth(linewidth);
		//glEndList();
	}
}

void MeshDisplay::drawSelectedFace(int selected_face)
{
	if (selected_face > 0)
	{
		//if (use_lighting)
		//	glEnable(GL_LIGHTING);
		//else
		//	glDisable(GL_LIGHTING);

		glEnable(GL_BLEND);
		glEnable(GL_POLYGON_OFFSET_FILL); // Avoid Stitching!
		glDisable(GL_DEPTH_TEST);
		glPolygonOffset(1.0, 1.0);
		//draw mesh faces
		{
			glBegin(GL_TRIANGLES);

			glColor3d(0., 0., 0.);

			// flat shading
			if (normals_type == PER_FACE)
			{
				const RowVector3 &normal = face_normals->row(selected_face);
				glNormal3f(normal[0], normal[1], normal[2]);
			}
			// loop over vertices in this face
			for (int vit = 0; vit < faces->cols(); ++vit)
			{
				glColor3d(0., 0., 0.);
				// not flat shading goes here
				if (normals_type == PER_VERTEX)
				{
					const RowVector3 &normal = vertex_normals->row((*faces)(selected_face, vit));
					glNormal3f(normal[0], normal[1], normal[2]);
				}
				else
					if (normals_type == PER_CORNER)
					{
						const RowVector3 &normal = corner_normals->row((*fNormIndices)(selected_face, vit));
						glNormal3f(normal[0], normal[1], normal[2]);
					}
				// assumes every point in vertices is length == 3
				const ScalarType *vertex_data = vertices->data() + 3 * (*faces)(selected_face, vit);
				glVertex3f(vertex_data[0], vertex_data[1], vertex_data[2]);

			}
			glEnd();
		}
		glDisable(GL_POLYGON_OFFSET_FILL);
		glEnable(GL_DEPTH_TEST);


//#ifndef PREVIEW3D_NO_SHADERS
//		glUseProgram(shader_id);
//#endif
	}

}

void MeshDisplay::update_colors()
{
	compute_face_colors(face_property, face_colors, colorBarType);
	compute_vertex_colors(vertex_property, vertex_colors, colorBarType);
}

void MeshDisplay::set_corner_threshold(double value)
{
	corner_threshold = value;
	igl::per_corner_normals(*vertices, *faces, *face_normals, vertex_to_faces, corner_threshold, *corner_normals);
	is_compiled = false;
}

double MeshDisplay::get_corner_threshold()
{
	return corner_threshold;
}

void MeshDisplay::set_numIsoLevels(int value)
{
	numIsoLevels = value;
	compute_isolevels();
	is_compiled = false;
}

int MeshDisplay::get_numIsoLevels()
{
	return numIsoLevels;
}

std::string MeshDisplay::GetTexFilename()
{
	return texFilename;
}

void MeshDisplay::SetTexFilename(const std::string fname)
{
	texFilename = fname;
	texture_id = texture_from_png(texFilename);//wangyu texture_id = texture_from_tga(texFilename);
	if (texture_id)
	{
		show_texture = true;
		is_compiled = false;
	}
}

bool MeshDisplay::GetShowTexture()
{
	return show_texture;
}

void MeshDisplay::SetShowTexture(const bool value)
{
	if (show_texture != value)
	{
		show_texture = value;
		is_compiled = false;
	}
}

void MeshDisplay::set_linewidth(int value)
{
	linewidth = value;
	is_compiled = false;
}

bool MeshDisplay::set_vertex_colors(const float default_color[4], const Eigen::VectorXi& new_color_indices, const Eigen::MatrixXd& new_vertex_colors, bool same_color /*= true*/)
{
	if (vertices->rows() == 0)
	{
		printf("Mesh is not set yet!\n");
		return false;
	}

	if (vertex_colors->rows() != vertices->rows())
	{
		vertex_colors->resize(vertices->rows(), 3);
		vertex_colors->col(0).setConstant(default_color[0]);
		vertex_colors->col(1).setConstant(default_color[1]);
		vertex_colors->col(2).setConstant(default_color[2]);
	}


	if (new_vertex_colors.cols() != 3)
	{
		printf("Error: the new_vertex_colors matrix does not have correct columns.\n");
		return false;
	}

	if (new_color_indices.rows() != new_vertex_colors.rows())
	{

		if (new_vertex_colors.rows() == 1)
		{
			if (!same_color)
				printf("Warning: the new_color_indices size does not match new_vertex_colors size, use the first color for all indices.\n");
		}
		else
		{
			printf("Error: the new_color_indices size does not match new_vertex_colors size.\n");
			return false;
		}
	}

	for (int i = 0; i < new_color_indices.rows(); i++)
	{
		if (new_color_indices(i) >= vertex_colors->rows())
		{
			printf("Error: the new_color_indices(%d) exceeds the rows of vertex_colors.\n");
			is_compiled = false;
			return false;
		}
		else
		{
			if (i >= new_vertex_colors.rows())
			{
				vertex_colors->row(new_color_indices(i)) = new_vertex_colors.row(0);
			}
			else
			{
				vertex_colors->row(new_color_indices(i)) = new_vertex_colors.row(i);
			}

		}

	}

	is_compiled = false;
	return true;
}

bool MeshDisplay::set_face_colors(const float default_color[4], const Eigen::VectorXi& new_color_indices, const Eigen::MatrixXd& new_face_colors, bool same_color /*= true*/)// there could be redundant indices, the earlier ones will be overwritten by the later ones. 
{
	if (faces->rows() == 0)
	{
		printf("Mesh is not set yet!\n");
		return false;
	}

	if (face_colors->rows() != faces->rows())
	{
		face_colors->resize(faces->rows(), 3);
		face_colors->col(0).setConstant(default_color[0]);
		face_colors->col(1).setConstant(default_color[1]);
		face_colors->col(2).setConstant(default_color[2]);
	}


	if (new_face_colors.cols() != 3)
	{
		printf("Error: the new_face_colors matrix does not have correct columns.\n");
		return false;
	}

	if (new_color_indices.rows() != new_face_colors.rows())
	{

		if (new_face_colors.rows() == 1)
		{
			if (!same_color)
				printf("Warning: the new_color_indices size does not match new_face_colors size, use the first color for all indices.\n");
		}
		else
		{
			printf("Error: the new_color_indices size does not match new_face_colors size.\n");
			return false;
		}
	}

	for (int i = 0; i < new_color_indices.rows(); i++)
	{
		if (new_color_indices(i) >= face_colors->rows())
		{
			printf("Error: the new_color_indices(%d) exceeds the rows of face_colors.\n");
			is_compiled = false;
			return false;
		}
		else
		{
			if (i >= new_face_colors.rows())
			{
				face_colors->row(new_color_indices(i)) = new_face_colors.row(0);
			}
			else
			{
				face_colors->row(new_color_indices(i)) = new_face_colors.row(i);
			}

		}

	}

	is_compiled = false;
	return true;
}

bool MeshDisplay::set_face_colors_from_vertex(const Eigen::VectorXi& new_color_indices, const Eigen::MatrixXd& new_vertex_colors)
{
	//assert(new_vertex_colors.rows() == 1); // So far only implemented single color for all indices

	int num_to_write = 0;

	for (int i = 0; i < new_color_indices.rows(); i++)
	{
		num_to_write += vertex_to_faces[new_color_indices(i)].size();
	}

	Eigen::VectorXi face_color_indices(num_to_write);

	int k = 0;
	for (int i = 0; i < new_color_indices.rows(); i++)
	{
		for (int j = 0; j < vertex_to_faces[new_color_indices(i)].size(); j++)
		{
			face_color_indices(k++) = vertex_to_faces[new_color_indices(i)][j];
		}
	}

	//printf("num of faces to be set %d.\n", num_to_write);

	float default_color[4] = { 1., 1., 1., 1. };
	return set_face_colors(default_color, face_color_indices, new_vertex_colors);

}

void MeshDisplay::SetFlipYCoord(bool f)
{
	if (bFlipYCoord != f&&texCoords->rows() > 0)
		flipCoord(texCoords, 1, texCoords);
	bFlipYCoord = f;
	resetTexCoords(!texCoords->rows(), !fUseTexture.size());
}

void MeshDisplay::ResetColor(const float default_color[4])
{
	vertex_colors->col(0).setConstant(default_color[0]);
	vertex_colors->col(1).setConstant(default_color[1]);
	vertex_colors->col(2).setConstant(default_color[2]);

	face_colors->col(0).setConstant(default_color[0]);
	face_colors->col(1).setConstant(default_color[1]);
	face_colors->col(2).setConstant(default_color[2]);
}

#ifndef PREVIEW3D_NO_SHADERS

void MeshDisplay::set_shader_mode(const ShaderMode id, const bool use_lighting)
{
	int nid = id;
	if (nid >= NUM_SHADER_MODE)
		nid = 0;
	shader_mode = id;
	if (texture_id)
		shader_id = 0;
	else
	{
		switch (shader_mode) {
		case DIRECTIONAL_PER_PIXEL:
		{
			if (use_lighting)
			{
				if (vertex_colors->rows() > 0)
				{
					shader_id = s_directionalPerPixelColorProgram.p;
				}
				else
				{
					shader_id = s_directionalPerPixelProgram.p;
				}
			}
			else
			{
				shader_id = 0;
			}
			break;
		}
		default:
		{
			shader_id = 0;
			break;
		}
		}
	}
}

MeshDisplay::ShaderMode MeshDisplay::get_shader_mode()
{
	return shader_mode;
}

#endif

void MeshDisplay::draw_colors_on_mesh(Eigen::VectorXd& new_color)
{
	*vertex_property = new_color;
	compute_isolevels();
	// do not need this: compute_vertex_colors(vertex_property,vertex_colors);
	update_colors();
}

void MeshDisplay::update_normal(NormalUpdateMethod method)
{
	//DeformSkinning::GetReference().update_skinning = true;
	switch (method)
	{
	case NORMAL_UPDATE_NAIVE_RECOMPUTE:
	{
		normals_changed = true;
	}
	break;
	case NORMAL_UPDATE_PBS:
	{

	}
	break;
	case NORMAL_UPDATE_ALL:
	{
		normals_changed = true;
	}
	break;
	}
}

#ifndef PREVIEW3D_NO_SHADERS
#include "GLSL/directionalperpixel.h"
#include "GLSL/directionalperpixelcolor.h"
#include "GLSL/isolines.h"
#endif

#define PBS_SHADER_VERT_PATH "C:\\WorkSpace\\Visual Studio 2013\\PBS\\viewer\\src\\GLSL\\PBSshader.vert"
#define PBS_SHADER_FRAG_PATH "C:\\WorkSpace\\Visual Studio 2013\\PBS\\viewer\\src\\GLSL\\PBSshader.frag"


void MeshDisplay::reload_shader()
{
	deinitShaderProgram(s_directionalPerPixelProgram);
	deinitShaderProgram(s_directionalPerPixelColorProgram);
	deinitShaderProgram(s_pbsShaderProgram);

	// Load shaders
	//s_directionalPerPixelProgram = loadShaderProgramStr(directionalperpixel_vert, directionalperpixel_frag);
	//s_directionalPerPixelColorProgram = loadShaderProgramStr(directionalperpixelcolor_vert, directionalperpixelcolor_frag);
	//s_pbsShaderProgram = loadShaderProgram(PBS_SHADER_VERT_PATH, PBS_SHADER_FRAG_PATH);
	load_shader();
}

void MeshDisplay::load_shader()
{
	printf("Loading shader.\n");

	GLint n;
	glGetIntegerv(GL_MAX_VERTEX_ATTRIBS, &n);
	printf("GL_MAX_VERTEX_ATTRIBS: %d\n", n);
	glGetIntegerv(GL_MAX_VERTEX_UNIFORM_COMPONENTS, &n);
	printf("GL_MAX_VERTEX_UNIFORM_COMPONENTS: %d\n", n);
	printf("GL_VERSION: %s\n", glGetString(GL_VERSION));
	printf("GL_SHADING_LANGUAGE_VERSION: %s\n",
		glGetString(GL_SHADING_LANGUAGE_VERSION));
	printf("GL_RENDERER: %s\n", glGetString(GL_RENDERER));


	std::vector<struct GLSL_Attrib> glsl_attribs;
	glsl_attribs.clear();
	//for (int i=0; i<NUM_WEIGHTS_SLOTS_IN_SHADER; i++)
	//{
	//	// This should corresponds to the name in the shader file.
	//	char name_weights[256] = "weights0";
	//	name_weights[7] += i;
	//	char name_weight_indices[256] = "weight_indices0";
	//	name_weight_indices[14] += i;


	//	struct GLSL_Attrib new_attrib;

	//	loc_of_weights_in_shader[i] = 2 + i;// Location is assigned here.
	//	new_attrib.id = loc_of_weights_in_shader[i];
	//	new_attrib.name = std::string(name_weights);
	//	glsl_attribs.push_back(new_attrib);

	//	loc_of_weight_indices_in_shader[i] = 2+ NUM_WEIGHTS_SLOTS_IN_SHADER + i;// Location is assigned here.
	//	new_attrib.id = loc_of_weight_indices_in_shader[i];
	//	new_attrib.name = std::string(name_weight_indices);
	//	glsl_attribs.push_back(new_attrib);
	//}

	//for (int i=0; i<NUM_WEIGHTS_SLOTS_IN_SHADER; i++)
	//{
	//	loc_of_weights_in_shader[i] = 0;
	//	loc_of_weight_indices_in_shader[i] = 0;
	//}

	// Load shaders
	s_directionalPerPixelProgram = loadShaderProgramStr(directionalperpixel_vert, directionalperpixel_frag);
	s_directionalPerPixelColorProgram = loadShaderProgramStr(directionalperpixelcolor_vert, directionalperpixelcolor_frag);
	s_pbsShaderProgram = loadShaderProgramWithAttribs(PBS_SHADER_VERT_PATH, PBS_SHADER_FRAG_PATH, glsl_attribs);
	//s_pbsShaderProgram = loadShaderProgram(PBS_SHADER_VERT_PATH, PBS_SHADER_FRAG_PATH);

	printShaderInfoLog(s_pbsShaderProgram.p);
	printProgramInfoLog(s_pbsShaderProgram.p);
}

#include <print_matlab.h>

bool MeshDisplay::send_status_to_matlab() const
{
	const Eigen::MatrixXd vertices_ = vertices->cast<double>();
	const Eigen::MatrixXi faces_ = faces->cast<int>();
	const Eigen::MatrixXi tets_ = tets->cast<int>();
	print_matlab("vertices", vertices_);
	print_matlab("faces", faces_);
	print_matlab("tets", tets_);

	return true;
}

bool MeshDisplay::send_color_to_matlab() const
{
	const Eigen::MatrixXd vertex_property_ = vertex_property->cast<double>();
	const Eigen::MatrixXd face_property_ = face_property->cast<double>();
	print_matlab("vertex_property", vertex_property_);
	print_matlab("face_property", face_property_);
	//print_matlab("tet_property",)
	return true;
}


