#version 140

//uniform int u_choose_tex;
//uniform sampler2D u_sampler1;

varying vec4 diffuse,ambient;
varying vec3 normal,lightDir,halfVector;

varying vec4 f_color;
varying vec4 f_normal;
varying vec4 f_light1;
varying vec4 f_light2;
varying vec4 f_light3;
//in vec2 f_texcoord;

uniform vec4 my_LightSource_position;
uniform vec4 my_LightSource_ambient;
uniform vec4 my_LightSource_diffuse;
uniform vec4 my_LightSource_halfVector;

uniform vec4 my_FrontMaterial_diffuse;
uniform vec4 my_FrontMaterial_ambient;
uniform vec4 my_LightModel_ambient;


out vec4 out_Color;
void main()
{
    vec4 diffuseColor = f_color;//material color
    
    float diffuseTerm1 = clamp(dot(f_normal, f_light1), 0.0, 1.0);
    float diffuseTerm2 = clamp(dot(f_normal, f_light2), 0.0, 1.0);
    float diffuseTerm3 = clamp(dot(f_normal, f_light3), 0.0, 1.0);

    float totalTerm = (diffuseTerm1 + diffuseTerm2 + diffuseTerm3) / 2 * 0.7 + 0.3;

    /*if (u_choose_tex != 0)
        out_Color = texture( u_sampler1, vec2(f_texcoord)) * totalTerm;
    else*/
        out_Color = diffuseColor * totalTerm;
	
	
	vec3 n,halfV;
    float NdotL,NdotHV;
    
    vec4 color = ambient;
    
    n = normalize(normal);
    
    NdotL = max(dot(n,lightDir),0.0);
    
    if (NdotL > 0.0) {
        color += diffuse * NdotL;
        halfV = normalize(halfVector);
        NdotHV = max(dot(n,halfV),0.0);
        color += gl_FrontMaterial.specular * gl_LightSource[0].specular * pow(NdotHV, gl_FrontMaterial.shininess);
    } 
	
    out_Color = color;	
		
}
