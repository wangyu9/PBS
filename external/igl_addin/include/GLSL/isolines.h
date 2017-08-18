// isoline Shaderv nGLint pUVRotMatrix;
namespace GLSL_isolines
{
    GLint pUVFrequency;
    GLint pUVRotMatrix;
    
    GLfloat uvFrequency = 16.0;
    GLfloat uvAngle = 0;
}

// DO NOT CHANGE ANYTHING BELOW THIS LINE - AUTOMATICALLY GENERATED
//==
char isolines_vert[] = "\
varying vec2 TexCoord;\n\
\n\
void main()\n\
{\n\
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;\n\
    TexCoord    = gl_MultiTexCoord0.st;\n\
}\n\
";
char isolines_frag[] = "\
vec3  Color1=vec3(1,0,0);\n\
vec3  Color2=vec3(0,0,1);\n\
vec3  AvgColor=vec3(0.5,0.5,0.5);\n\
\n\
float uvFrequency=8.;\n\
vec4  uvRotation=vec4(1.,0.,0.,1.);\n\
\n\
varying vec2  TexCoord;\n\
\n\
void main()\n\
{\n\
    vec3 color;\n\
    \n\
    vec2 uv = mat2(uvRotation.x, uvRotation.y, uvRotation.z, uvRotation.w)*TexCoord;\n\
        \n\
    // Determine the width of the projection of one pixel into s-t space\n\
    vec2 fw = fwidth(uv);\n\
\n\
    // Determine the amount of fuzziness\n\
    vec2 fuzz = fw * uvFrequency * 2.0;\n\
\n\
    float fuzzMax = max(fuzz.s, fuzz.t);\n\
\n\
    // Determine the position in the checkerboard pattern\n\
    vec2 checkPos = fract(uv * uvFrequency);\n\
    \n\
    // If the filter width is small enough, compute the pattern color\n\
    vec2 p = (smoothstep(vec2(1.0) - fuzz, vec2(1.0), checkPos)) +\n\
             (1.0 - smoothstep(vec2(0.0), fuzz, checkPos));\n\
\n\
    color = mix(vec3(1.0,1.0,1.0), vec3(0.0, 0.0, 1.0), p.x) * \n\
            mix(vec3(1.0,1.0,1.0), vec3(1.0, 0.0, 0.0), p.y) ;\n\
            \n\
    // Fade in the average color when we get close to the limit\n\
    //color = mix(color, AvgColor, smoothstep(0.125, 0.5, fuzzMax));\n\
\n\
	color.g = min(color.r, color.b);\n\
    gl_FragColor = vec4(color, 1.0);\n\
    \n\
//    gl_FragColor = vec4(0.0,1.0,1.0,1.0);\n\
}\n\
";
