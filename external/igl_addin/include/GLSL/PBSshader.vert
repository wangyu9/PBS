#version 150

uniform mat4 my_ModelViewMatrix;
uniform mat4 my_ProjectionMatrix;
//uniform mat4 u_transformMatrix;

//attribute vec3 whatever;
//attribute vec4 my_Vertex;//in vec3 v_position;
//attribute vec3 v_color;
//attribute vec4 my_Normal;
//attribute vec2 v_texcoord;

varying vec4 f_color;
varying vec4 f_normal;
varying vec4 f_light1;
varying vec4 f_light2;
varying vec4 f_light3;
//out vec2 f_texcoord;

// rendering
varying vec4 diffuse,ambient;
varying vec3 normal,lightDir,halfVector;
varying vec3 viewerDir;

uniform vec4 my_LightSource_position;
uniform vec4 my_LightSource_ambient;
uniform vec4 my_LightSource_diffuse;
uniform vec4 my_LightSource_halfVector;

uniform vec4 my_FrontMaterial_diffuse;
uniform vec4 my_FrontMaterial_ambient;
uniform vec4 my_LightModel_ambient;

//Point Based Skinning
uniform vec4 handle_pos[256];

attribute vec4 weights0;
attribute vec4 weight_indices0;

attribute vec4 weights1;
attribute vec4 weight_indices1;

attribute vec4 weights2;
attribute vec4 weight_indices2;

attribute vec4 weights3;
attribute vec4 weight_indices3;

//attribute vec4 weights4;
//attribute vec4 weight_indices4;

//attribute vec4 weights5;
//attribute vec4 weight_indices5;

attribute vec4 alpha0;
attribute vec4 beta0;

attribute vec4 alpha1;
attribute vec4 beta1;

attribute vec4 alpha2;
attribute vec4 beta2;

attribute vec4 alpha3;
attribute vec4 beta3;

void main()
{
    f_color = vec4(1.0,0.0,0.0,1.0);//vec4(v_color, 1.0);
    f_light1 =normalize(my_ModelViewMatrix * vec4(0.0, 0.707, -0.707, 0.0));
    f_light2 =normalize(my_ModelViewMatrix * vec4(-0.707, -0.0, 0.707, 0.0));
    f_light3 =normalize(my_ModelViewMatrix * vec4(0.707, -0.0, 0.707, 0.0));
    //f_texcoord = v_texcoord;

	vec4 just_cite_this_not_use0 = handle_pos[0];	
	//vec4 just_cite_this_not_use1 = my_Vertex;	
	vec4 just_cite_this_not_use2 =  my_LightSource_position
									+ my_LightSource_ambient
									+ my_LightSource_diffuse
									+ my_LightSource_halfVector
									+ my_FrontMaterial_diffuse
									+ my_FrontMaterial_ambient
									+ my_LightModel_ambient;
 	vec4 just_cite_this_not_use3 = alpha0 + beta0;
	
	vec4 trans_LightSource_position = my_LightSource_position;
	vec4 trans_eye_position = vec4(0,0,1,0);
	//vec4 trans_LightSource_position = my_ModelViewMatrix * my_LightSource_position;
	//vec4 trans_eye_position = my_ModelViewMatrix * vec4(0,0,1,0);
	/*****************rendering***********************/
    lightDir = normalize(vec3(trans_LightSource_position));
	viewerDir = normalize(trans_eye_position.xyz);
    halfVector = normalize(lightDir+viewerDir.xyz);//normalize(my_LightSource_halfVector.xyz);
   	diffuse = my_FrontMaterial_diffuse * my_LightSource_diffuse;
    ambient = my_FrontMaterial_ambient * my_LightSource_ambient;
    ambient += my_LightModel_ambient * my_FrontMaterial_ambient;
	
	/*****************Normal Updating*****************/
	//f_normal = transpose(inverse(my_ModelViewMatrix)) * my_Normal;
	
	//normal = vec3(my_Normal+alpha0+beta0);
	
 	vec3 tangent = vec3(0,0,0);
	vec3 binormal = vec3(0,0,0);
	for(int k=0; k<4; k++)
	{
		int i = int(weight_indices0[k]); 
		vec3 temp = handle_pos[i].xyz;		
		tangent += temp * 1.0 * alpha0[k];
		binormal += temp * 1.0 * beta0[k];
	}
	for(int k=0; k<4; k++)
	{
		int i = int(weight_indices1[k]); 
		vec3 temp = handle_pos[i].xyz;		
		tangent += temp * 1.0 * alpha1[k];
		binormal += temp * 1.0 * beta1[k];
	}
	for(int k=0; k<4; k++)
	{
		int i = int(weight_indices2[k]); 
		vec3 temp = handle_pos[i].xyz;		
		tangent += temp * 1.0 * alpha2[k];
		binormal += temp * 1.0 * beta2[k];
	}
	for(int k=0; k<4; k++)
	{
		int i = int(weight_indices3[k]); 
		vec3 temp = handle_pos[i].xyz;		
		tangent += temp * 1.0 * alpha3[k];
		binormal += temp * 1.0 * beta3[k];
	}
	normal = cross(tangent,binormal);//*0.999+0.001*my_Normal.xyz;
	normal = (my_ModelViewMatrix*vec4(normal,0)).xyz;//transform normals
	normal = normalize(normal);
	//normal = cross(tangent,binormal)*0.000+1.000*my_Normal.xyz;
	
	
	/**************Point Based Skinning***************/
	vec4 _pos = vec4(0.0,0.0,0.0,0.0);

 	for(int k=0; k<4; k++)
	{
		// influencing handle index
		int i = int(weight_indices0[k]); 
		vec4 temp = handle_pos[i];		
		temp.w = 1.0;
		_pos += temp * 1.0 * weights0[k];
	} 
#define efef
#ifdef efef		
	for(int k=0; k<4; k++)
	{
		// influencing handle index
		int i = int(weight_indices1[k]); 
		vec4 temp = handle_pos[i];		
		temp.w = 1.0;
		_pos += temp * 1.0 * weights1[k];
	} 
	for(int k=0; k<4; k++)
	{
		// influencing handle index
		int i = int(weight_indices2[k]); 
		vec4 temp = handle_pos[i];		
		temp.w = 1.0;
		_pos += temp * 1.0 * weights2[k];
	}
	for(int k=0; k<4; k++)
	{
		// influencing handle index
		int i = int(weight_indices3[k]); 
		vec4 temp = handle_pos[i];		
		temp.w = 1.0;
		_pos += temp * 1.0 * weights3[k];
	} 
/*	for(int k=0; k<4; k++)
	{
		// influencing handle index
		int i = int(weight_indices4[k]); 
		vec4 temp = handle_pos[i];		
		temp.w = 1.0;
		_pos += temp * 1.0 * weights4[k];
	} 
 	for(int k=0; k<4; k++)
	{
		// influencing handle index
		int i = int(weight_indices5[k]); 
		vec4 temp = handle_pos[i];		
		temp.w = 1.0;
		_pos += temp * 1.0 * weights5[k] * 0.1;
	}  */
#endif
    gl_Position = my_ProjectionMatrix * my_ModelViewMatrix * (_pos*1.0);//+0.00*my_Vertex);
}
