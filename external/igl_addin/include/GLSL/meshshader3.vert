#version 150

// rendering
varying vec4 diffuse,ambient;
varying vec3 normal,lightDir,halfVector;

attribute vec3 v_position;
attribute vec3 v_color;
attribute vec3 v_normal;
attribute vec2 v_texcoord;

void main()
{
	//rendering
    normal = vec3(1.0,1.0,1.0);//normalize(gl_NormalMatrix * v_normal);
    lightDir = normalize(vec3(gl_LightSource[0].position));
	
    halfVector = normalize(gl_LightSource[0].halfVector.xyz);
    
    diffuse = gl_FrontMaterial.diffuse * gl_LightSource[0].diffuse;
    ambient = gl_FrontMaterial.ambient * gl_LightSource[0].ambient;
    ambient += gl_LightModel.ambient * gl_FrontMaterial.ambient;
	
    gl_Position = gl_ModelViewProjectionMatrix * vec4(v_position, 1.0);
}
