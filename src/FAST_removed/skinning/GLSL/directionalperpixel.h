// isoline Shaderv nGLint pUVRotMatrix;
namespace GLSL_directional
{
    GLint pUVFrequency;
    GLint pUVRotMatrix;
    
    GLfloat uvFrequency = 16.0;
    GLfloat uvAngle = 0;
}

// DO NOT CHANGE ANYTHING BELOW THIS LINE - AUTOMATICALLY GENERATED
//==
char directionalperpixel_vert[] = "\
varying vec4 diffuse,ambient;\n\
varying vec3 normal,lightDir,halfVector;\n\
\n\
void main()\n\
{	\n\
    normal = normalize(gl_NormalMatrix * gl_Normal);\n\
    lightDir = normalize(vec3(gl_LightSource[0].position));\n\
	\n\
    halfVector = normalize(gl_LightSource[0].halfVector.xyz);\n\
    \n\
    diffuse = gl_FrontMaterial.diffuse * gl_LightSource[0].diffuse;\n\
    ambient = gl_FrontMaterial.ambient * gl_LightSource[0].ambient;\n\
    ambient += gl_LightModel.ambient * gl_FrontMaterial.ambient;\n\
	\n\
    gl_Position = ftransform();\n\
} \n\
";
char directionalperpixel_frag[] = "\
varying vec4 diffuse,ambient;\n\
varying vec3 normal,lightDir,halfVector;\n\
\n\
void main()\n\
{\n\
    vec3 n,halfV;\n\
    float NdotL,NdotHV;\n\
    \n\
    vec4 color = ambient;\n\
    \n\
    n = normalize(normal);\n\
    \n\
    NdotL = max(dot(n,lightDir),0.0);\n\
    \n\
    if (NdotL > 0.0) {\n\
        color += diffuse * NdotL;\n\
        halfV = normalize(halfVector);\n\
        NdotHV = max(dot(n,halfV),0.0);\n\
        color += gl_FrontMaterial.specular * gl_LightSource[0].specular * pow(NdotHV, gl_FrontMaterial.shininess);\n\
    }\n\
	\n\
    gl_FragColor = color;\n\
}\n\
";
