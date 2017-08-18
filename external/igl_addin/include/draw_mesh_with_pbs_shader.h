#ifndef IGL_ADDIN_DRAW_MESH_WITH_PBS_SHADER
#define IGL_ADDIN_DRAW_MESH_WITH_PBS_SHADER

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

#include <types.h>
#include <Eigen/Core>
#include <utils/TimerWrapper.h>

#include <Mesh.h> //for Material and Light
#include <Camera.h>

struct Weights_shader_info{
	GLuint m_attrib_weights;
	GLuint m_attrib_weight_indices;
	float * pWeights;
	float * pWeightsIndices;
};

struct Weight_gradient
{
	GLuint m_alpha;
	GLuint m_beta;
	float *pAlpha;
	float *pBeta;
	int loc_alpha;
	int loc_beta;
	// loc_of_weight_gradients_in_shader, assume it is in the same order of weights
	// i.e. loc_of_weight_indices_in_shader is used for indexing.
};

#define SHADER_HANDLE_VEC_DIM 4

inline void write_matrix_to_array(Eigen::MatrixXd M, float *a)
{
	for (int i=0; i<M.rows(); i++)
	{
		for (int j=0; j<M.cols(); j++)
		{
			a[i*SHADER_HANDLE_VEC_DIM+j] = M(i,j);
		}
		//printf("%f %f %f %f\n",a[i*4+0],a[i*4+1],a[i*4+2],a[i*4+3]);
	}
}

GLfloat glLightModel_ambient[4] = {0.2,0.2, 0.2, 1.0};

void draw_triangle_mesh_or_line_with_shader(
	const bool is_face, 
	const Eigen::MatrixXd copy_Handle,
	const Eigen::MatrixXd& W, 
	const Eigen::MatrixXi& WI, 
	const Eigen::MatrixXd& Alpha, 
	const Eigen::MatrixXd& Beta, 
	const GLuint shader_id_for_face_and_line,
	const Eigen::MatrixXd *vertices,
	const Eigen::MatrixXi *faces,
	const Eigen::MatrixXd *vertex_normals,
	const struct GLSL_Program s_pbsShaderProgram,
	 Weight_gradient *weight_gradient,
	 Weights_shader_info* weights_shader_info,
	const Camera camera,
	const Material material,
	const Light light,
	int linewidth,
	GLuint& m_vbo,
	GLuint& m_nbo,
	GLuint& m_ibo,
	int * loc_of_weights_in_shader,
	int * loc_of_weight_indices_in_shader,
	bool &buffers_setup,
	bool &weights_setup
	)
{		
	const Eigen::MatrixXi& F = *faces;
	const Eigen::MatrixXd& V = *vertices;

	if (buffers_setup==false)
	{
		for (int group=0; group<NUM_WEIGHTS_SLOTS_IN_SHADER; group++)
		{
			if(!glIsBuffer(weights_shader_info[group].m_attrib_weights))
				glGenBuffers(1, &weights_shader_info[group].m_attrib_weights);
			if (!glIsBuffer(weights_shader_info[group].m_attrib_weight_indices)) 
				glGenBuffers(1, &weights_shader_info[group].m_attrib_weight_indices);
		}
		for (int group=0; group<NUM_WEIGHT_GRADIENTS_SLOTS_IN_SHADER; group++)
		{
			if(!glIsBuffer(weight_gradient[group].m_alpha))
				glGenBuffers(1, &weight_gradient[group].m_alpha);	
			if(!glIsBuffer(weight_gradient[group].m_beta))
				glGenBuffers(1, &weight_gradient[group].m_beta);	
		}

		if(!glIsBuffer(m_vbo))
			glGenBuffers(1, &m_vbo);
		//if(!glIsBuffer(m_cbo))
		//	glGenBuffers(1, &m_cbo);
		if(!glIsBuffer(m_nbo))
			glGenBuffers(1, &m_nbo);
		//if(!glIsBuffer(m_tbo))
		//	glGenBuffers(1, &m_tbo);
		if(!glIsBuffer(m_ibo))
			glGenBuffers(1, &m_ibo);
		buffers_setup = true;
	}

	int m = (WI.cols()>0)?(WI.maxCoeff()+1):0;//HandlePlugin::GetReference().Handles().rows();

	if(weights_setup)
	{
		// Weights
		for (int i=0; i<NUM_WEIGHTS_SLOTS_IN_SHADER; i++)
		{
			char name_weights0[256] = "weights0";
			name_weights0[7] += i;
			char name_weight_indices0[256] = "weight_indices0";
			name_weight_indices0[14] += i;
			loc_of_weights_in_shader[i] = glGetAttribLocation(shader_id_for_face_and_line, name_weights0);
			loc_of_weight_indices_in_shader[i] = glGetAttribLocation(shader_id_for_face_and_line, name_weight_indices0);
			if (loc_of_weights_in_shader[i]==-1||loc_of_weight_indices_in_shader[i]==-1)
			{
				printf("Allocate attributes loc_of_weights_in_shader[%d] or loc_of_weight_indices_in_shader[%d] fails!\n",i,i);
			}
		}

		// Weight Gradients
		for (int i=0; i<NUM_WEIGHT_GRADIENTS_SLOTS_IN_SHADER; i++)
		{
			char name_alpha[256] = "alpha0";
			name_alpha[5] += i;
			char name_beta[256] = "beta0";
			name_beta[4] += i;
			weight_gradient[i].loc_alpha = glGetAttribLocation(shader_id_for_face_and_line, name_alpha);
			weight_gradient[i].loc_beta = glGetAttribLocation(shader_id_for_face_and_line, name_beta);
			if (weight_gradient[i].loc_alpha==-1||weight_gradient[i].loc_beta==-1)
			{
				printf("Allocate attributes alpha%d or beta%d fails!\n",i,i);
			}
		}


		// Handles
#define MAX_SUPPORTED_HANDLE 256
		float a[MAX_SUPPORTED_HANDLE*4] = {0};

		write_matrix_to_array(copy_Handle,a);

		char name_handles[256] = "handle_pos[0]";
		GLint handle_location_in_shader = glGetUniformLocation(s_pbsShaderProgram.p, name_handles);
		if (handle_location_in_shader==-1)
		{
			printf("Get handle pos fails!!\n");
		}
		else
		{
			glProgramUniform4fv(shader_id_for_face_and_line, handle_location_in_shader, MAX_SUPPORTED_HANDLE, a);// for the 2nd last parameter, 0 means no skiped memory
		}
	}

	bool color = is_face;

	glLineWidth(linewidth);

	TimerWrapper timerWrapper;
	timerWrapper.Tic();

	for (int group=0; group<NUM_WEIGHTS_SLOTS_IN_SHADER; group++)
	{
		if(weights_shader_info[group].pWeights!=NULL)
			delete [] weights_shader_info[group].pWeights;
		//weights_shader_info[group].pWeights = new float[4*3*faces->rows()];
		weights_shader_info[group].pWeights = new float[4*V.rows()];

		if (weights_shader_info[group].pWeightsIndices!=NULL)
			delete [] weights_shader_info[group].pWeightsIndices;
		weights_shader_info[group].pWeightsIndices = new float[4*V.rows()];

		for(int i = 0; i<V.rows(); i++)
		{
			for (int k=0; k<4; k++)
			{
				if (k+4*group<W.cols())
				{
					(weights_shader_info[group].pWeights)[4*i+k]		= W(i,k+4*group);
					(weights_shader_info[group].pWeightsIndices)[4*i+k] = WI(i,k+4*group);
				} 
				else
				{
					(weights_shader_info[group].pWeights)[4*i+k]		= 0;
					(weights_shader_info[group].pWeightsIndices)[4*i+k] = 0;
				}
			}
		}
	}
	for (int group=0; group<NUM_WEIGHT_GRADIENTS_SLOTS_IN_SHADER; group++)
	{
		if (weight_gradient[group].pAlpha!=NULL)
			delete [] weight_gradient[group].pAlpha;
		weight_gradient[group].pAlpha = new float[4*V.rows()];

		if (weight_gradient[group].pBeta!=NULL)
			delete [] weight_gradient[group].pBeta;
		weight_gradient[group].pBeta = new float[4*V.rows()];

		for(int i = 0; i<V.rows(); i++)
		{
			for (int k=0; k<4; k++)
			{
				if (k+4*group<Alpha.cols())
				{
					(weight_gradient[group].pAlpha)[4*i+k]	= Alpha(i,k+4*group);
					(weight_gradient[group].pBeta)[4*i+k]	= Beta(i,k+4*group);
				} 
				else
				{
					(weight_gradient[group].pAlpha)[4*i+k]	= 0;
					(weight_gradient[group].pBeta)[4*i+k]	= 0;
				}
			}
		}
	}

	bool wire_frame = false;
	glPolygonMode(GL_FRONT_AND_BACK, (wire_frame ? GL_LINE : GL_FILL));

	unsigned int size = V.rows();
	//unsigned int element_num = F.rows();

	//#define SHADER_WITH_VERTEX_AND_NORMAL
#ifdef SHADER_WITH_VERTEX_AND_NORMAL
	int loc_of_vertex	= glGetAttribLocation(shader_id_for_face_and_line, "my_Vertex"	);
	if (loc_of_vertex==-1)
	{
		printf("Cannot locate gl_Vertex position in the shader program.\n");
	}
	//int loc_of_colors	= glGetAttribLocation(shader_id_for_face_and_line, "v_color"	);
	int loc_of_normals	= glGetAttribLocation(shader_id_for_face_and_line, "my_Normal"	);
	if (loc_of_normals==-1)
	{
		printf("Cannot locate gl_Normal position in the shader program.\n");
	}
	//int loc_of_texture	= glGetAttribLocation(shader_id_for_face_and_line, "v_texcoord"	);
#endif

	int loc_of_modelviewmatrix = glGetUniformLocation(shader_id_for_face_and_line, "my_ModelViewMatrix"	);
	if (loc_of_modelviewmatrix==-1)
	{
		printf("Cannot locate my_ModelViewMatrix position in the shader program.\n");
	}
	int loc_of_projectionmatrix = glGetUniformLocation(shader_id_for_face_and_line, "my_ProjectionMatrix"	);
	if (loc_of_projectionmatrix==-1)
	{
		printf("Cannot locate my_ProjectionMatrix position in the shader program.\n");
	}

	int loc_of_LightSource_position = glGetUniformLocation(shader_id_for_face_and_line, "my_LightSource_position");
	int loc_of_LightSource_ambient = glGetUniformLocation(shader_id_for_face_and_line, "my_LightSource_ambient");
	int loc_of_LightSource_diffuse = glGetUniformLocation(shader_id_for_face_and_line, "my_LightSource_diffuse");
	int loc_of_LightSource_halfVector = glGetUniformLocation(shader_id_for_face_and_line, "my_LightSource_halfVector");
	if (loc_of_LightSource_position==-1
		||loc_of_LightSource_ambient==-1
		||loc_of_LightSource_diffuse==-1
		||loc_of_LightSource_halfVector==-1)
	{
		printf("Cannot locate my_LightSource position in the shader program.\n");
	}

	int loc_of_FrontMaterial_diffuse = glGetUniformLocation(shader_id_for_face_and_line, "my_FrontMaterial_diffuse");
	int loc_of_FrontMaterial_ambient = glGetUniformLocation(shader_id_for_face_and_line, "my_FrontMaterial_ambient");
	if (loc_of_FrontMaterial_diffuse==-1
		||loc_of_FrontMaterial_ambient==-1)
	{
		printf("Cannot locate my_FrontMaterial position in the shader program.\n");
	}


	int loc_of_LightModel_ambient = glGetUniformLocation(shader_id_for_face_and_line, "my_LightModel_ambient");
	if (loc_of_LightModel_ambient==-1)
	{
		printf("Cannot locate my_LightModel position in the shader program.\n");
	}


	float * m_positions = new float[4*V.rows()];
	//float * m_colors	= new float[3*V.rows()];
	float * m_normals	= new float[4*V.rows()];
	//float * m_texcoords = new float[3*V.rows()];

	for (int i=0; i<V.rows(); i++)
	{
		for (int c=0; c<3; c++)
		{
			m_positions[4*i+c]	= V(i,c);
			//m_colors[3*i+c]		= 255;//(*vertex_colors)(i,c);
			m_normals[4*i+c]	= (*vertex_normals)(i,c);
			//m_texcoords[3*i+c]	= 0;//(*vertex)
		}
		int c = 3;
		m_positions[4*i+c]	= 1.0;
		m_normals[4*i+c] = 1.0;
	}


	unsigned int * m_triangle_list = new unsigned int[3*F.rows()];
	for (int i=0; i<F.rows(); i++)
	{
		for (int c=0; c<3; c++)
		{
			m_triangle_list[3*i+c] = F(i,c);
			if (F(i,c)<0||F(i,c)>=V.rows())
			{
				printf("Error in vertex index for glsl shader: %d",F(i,c));
			}
		}
	}

	// weights
	for (int group=0; group<NUM_WEIGHTS_SLOTS_IN_SHADER; group++)
	{
		glBindBuffer(GL_ARRAY_BUFFER, weights_shader_info[group].m_attrib_weights);
		glBufferData(GL_ARRAY_BUFFER, 4*V.rows() * sizeof(float), weights_shader_info[group].pWeights, GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, weights_shader_info[group].m_attrib_weight_indices);
		glBufferData(GL_ARRAY_BUFFER, 4*V.rows() * sizeof(float), weights_shader_info[group].pWeightsIndices, GL_STATIC_DRAW);
	}
	for (int group=0; group<NUM_WEIGHT_GRADIENTS_SLOTS_IN_SHADER; group++)
	{
		glBindBuffer(GL_ARRAY_BUFFER, weight_gradient[group].m_alpha);
		glBufferData(GL_ARRAY_BUFFER, 4*V.rows() * sizeof(float), weight_gradient[group].pAlpha, GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, weight_gradient[group].m_beta);
		glBufferData(GL_ARRAY_BUFFER, 4*V.rows() * sizeof(float), weight_gradient[group].pBeta, GL_STATIC_DRAW);
	}

#ifdef SHADER_WITH_VERTEX_AND_NORMAL
	// position
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	glBufferData(GL_ARRAY_BUFFER, 4 * size * sizeof(float), &m_positions[0], GL_DYNAMIC_DRAW);

	//// color
	//glBindBuffer(GL_ARRAY_BUFFER, m_cbo);
	//glBufferData(GL_ARRAY_BUFFER, 3 * size * sizeof(float), &m_colors[0], GL_STATIC_DRAW);
	// normal
	glBindBuffer(GL_ARRAY_BUFFER, m_nbo);
	glBufferData(GL_ARRAY_BUFFER, 4 * size * sizeof(float), &m_normals[0], GL_DYNAMIC_DRAW);
	//// texture
	//glBindBuffer(GL_ARRAY_BUFFER, m_tbo);
	//glBufferData(GL_ARRAY_BUFFER, 2 * size * sizeof(float), &m_texcoords[0], GL_STATIC_DRAW);
#endif

	// indices
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, 3 * F.rows() * sizeof(unsigned int), &m_triangle_list[0], GL_STATIC_DRAW);


	for (int group=0; group<NUM_WEIGHTS_SLOTS_IN_SHADER; group++)
	{
		glEnableVertexAttribArray(loc_of_weights_in_shader[group]);
		glEnableVertexAttribArray(loc_of_weight_indices_in_shader[group]);
	}
	for (int group=0; group<NUM_WEIGHT_GRADIENTS_SLOTS_IN_SHADER; group++)
	{
		glEnableVertexAttribArray(weight_gradient[group].loc_alpha);
		glEnableVertexAttribArray(weight_gradient[group].loc_beta);
	}
#ifdef SHADER_WITH_VERTEX_AND_NORMAL
	glEnableVertexAttribArray(loc_of_vertex);
	glEnableVertexAttribArray(loc_of_normals);
#endif
	for (int group=0; group<NUM_WEIGHTS_SLOTS_IN_SHADER; group++)
	{
		glBindBuffer(GL_ARRAY_BUFFER, weights_shader_info[group].m_attrib_weights);
		glVertexAttribPointer(loc_of_weights_in_shader[group], 4, GL_FLOAT, GL_FALSE, 0, 0);

		glBindBuffer(GL_ARRAY_BUFFER, weights_shader_info[group].m_attrib_weight_indices);
		glVertexAttribPointer(loc_of_weight_indices_in_shader[group], 4, GL_FLOAT, GL_FALSE, 0, 0);
	}
	for (int group=0; group<NUM_WEIGHT_GRADIENTS_SLOTS_IN_SHADER; group++)
	{
		glBindBuffer(GL_ARRAY_BUFFER, weight_gradient[group].m_alpha);
		glVertexAttribPointer(weight_gradient[group].loc_alpha, 4, GL_FLOAT, GL_FALSE, 0, 0);

		glBindBuffer(GL_ARRAY_BUFFER, weight_gradient[group].m_beta);
		glVertexAttribPointer(weight_gradient[group].loc_beta, 4, GL_FLOAT, GL_FALSE, 0, 0);
	}

#ifdef SHADER_WITH_VERTEX_AND_NORMAL
	//glEnableClientState(GL_VERTEX_ARRAY);
	//glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	//glVertexPointer(4,GL_FLOAT,0,0);

	glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	glVertexAttribPointer(loc_of_vertex, 4, GL_FLOAT, GL_FALSE, 0, 0);

	//glBindBuffer(GL_ARRAY_BUFFER, m_cbo);
	//glVertexAttribPointer(loc_of_colors, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, m_nbo);
	glVertexAttribPointer(loc_of_normals, 4, GL_FLOAT, GL_FALSE, 0, 0);

	//glBindBuffer(GL_ARRAY_BUFFER, m_tbo);
	//glVertexAttribPointer(loc_of_texture, 2, GL_FLOAT, GL_FALSE, 0, 0);
#endif
	//glArrayElement(loc_of_weights_in_shader[0]);
	//glArrayElement(loc_of_weight_indices_in_shader[0]);

	float float_model_view_matrix[16];
	float float_projection_matrix[16];

	for (int i=0; i<16; i++)
	{
		float_model_view_matrix[i] = camera.m_modelview_matrix[i];
		float_projection_matrix[i] = camera.m_projection_matrix[i];
	}

	float v[4]; 
	v[0] = v[1] = v[2] = light.g_LightMultiplier*0.4f; v[3] = 1.0f;
	glUniform4fv(loc_of_LightSource_ambient, 1, &v[0]);//glLightfv(GL_LIGHT0, GL_AMBIENT, v);

	glUniform4fv(loc_of_LightModel_ambient, 1, &glLightModel_ambient[0]);//TEMP code

	v[0] = v[1] = v[2] = light.g_LightMultiplier*0.8f; v[3] = 1.0f;
	glUniform4fv(loc_of_LightSource_diffuse, 1, &v[0]);//glLightfv(GL_LIGHT0, GL_DIFFUSE, v);
	v[0] = -light.g_LightDirection[0]; v[1] = -light.g_LightDirection[1]; v[2] = -light.g_LightDirection[2]; v[3] = 0.0f;
	glUniform4fv(loc_of_LightSource_position, 1, &v[0]);//glLightfv(GL_LIGHT0, GL_POSITION, v);

	glUniform4fv(loc_of_FrontMaterial_ambient, 1, material.g_MatAmbient);
	glUniform4fv(loc_of_FrontMaterial_diffuse, 1, material.g_MatDiffuse);
	//glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, g_MatAmbient);
	//glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, g_MatDiffuse);
	//glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, g_MatSpecular);
	//glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS,g_MatShininess);


	v[0] = v[1] = v[2] = v[3] = 1.0f;
	glUniform4fv(loc_of_LightSource_halfVector, 1, &v[0]);


	glUniformMatrix4fv(loc_of_modelviewmatrix, 1, false, float_model_view_matrix);
	glUniformMatrix4fv(loc_of_projectionmatrix, 1, false, float_projection_matrix);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ibo);
	glDrawElements(GL_TRIANGLES, 3*F.rows(), GL_UNSIGNED_INT, 0);

	GLenum error = glGetError();

	for (int group=0; group<NUM_WEIGHTS_SLOTS_IN_SHADER; group++)
	{
		glDisableVertexAttribArray(loc_of_weights_in_shader[group]);
		glDisableVertexAttribArray(loc_of_weight_indices_in_shader[group]);
	}
	for (int group=0; group<NUM_WEIGHT_GRADIENTS_SLOTS_IN_SHADER; group++)
	{
		glDisableVertexAttribArray(weight_gradient[group].loc_alpha);
		glDisableVertexAttribArray(weight_gradient[group].loc_beta);
	}
#ifdef SHADER_WITH_VERTEX_AND_NORMAL
	glDisableVertexAttribArray(loc_of_vertex);
	//glDisableVertexAttribArray(loc_of_colors);
	glDisableVertexAttribArray(loc_of_normals);
	//glDisableVertexAttribArray(loc_of_texture);
#endif
	//glUniform1i(vbos.m_uniform_enable_texture, 0); // disable texture

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); 

	delete [] m_positions;
	delete [] m_normals;
	delete [] m_triangle_list;

	;

	timerWrapper.Toc();
	printf("Draw Mesh time: %f\n", timerWrapper.Duration());

	if (W.rows()!=0)
	{
		weights_setup = true;
	}

}

#endif /*IGL_ADDIN_DRAW_MESH_WITH_PBS_SHADER*/