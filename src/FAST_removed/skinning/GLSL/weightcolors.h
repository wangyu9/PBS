// DO NOT CHANGE ANYTHING BELOW THIS LINE - AUTOMATICALLY GENERATED
//==
char weightcolors_vert[] = "\
// Visualize weights on a mesh. Each weight function gets a pseudocolor and\n\
// these colors are belnded according to value of weights.\n\
\n\
// Necessary to use mat4x3 syntax for non-square matrices, otherwise compiler\n\
// treats code as version 110 which does not support non-square matrices\n\
#version 120\n\
\n\
\n\
#define MAT4\n\
#define MAX_NUM_HANDLES 100\n\
#define MAX_NUM_WEIGHTS_PER_VERTEX 8\n\
\n\
varying vec4 diffuse,ambient;\n\
varying vec3 normal,lightDir,halfVector;\n\
attribute vec4 weights0;\n\
attribute vec4 weights4;\n\
attribute vec4 weight_indices0;\n\
attribute vec4 weight_indices4;\n\
// Number of handles\n\
uniform int num_handles;\n\
\n\
int min(int a, int b) { return a < b ? a : b; }\n\
\n\
void main()\n\
{\n\
\n\
  //////////////////////////////////////////////////////////////////////////\n\
  // Vertex position\n\
  //////////////////////////////////////////////////////////////////////////\n\
  vec4 v = gl_Vertex;\n\
  // apply model view and projection matrix\n\
  gl_Position =  gl_ModelViewProjectionMatrix * v;\n\
  ////////////////////////////////////////////////////////////////////////////\n\
  //// Phong shading (per vertex colors)\n\
  ////////////////////////////////////////////////////////////////////////////\n\
  // Color table\n\
  vec4 colors[12];\n\
  colors[0] = vec4(1.0,0.0,0.0,1.0);\n\
  colors[1] = vec4(0.0,1.0,0.0,1.0);\n\
  colors[2] = vec4(0.0,0.0,1.0,1.0);\n\
  colors[3] = vec4(0.0,1.0,1.0,1.0);\n\
  colors[4] = vec4(1.0,0.0,1.0,1.0);\n\
  colors[5] = vec4(1.0,1.0,0.0,1.0);\n\
  colors[6] = vec4(1.0,1.0,1.0,1.0);\n\
  colors[7] = vec4(0.1,0.1,0.1,1.0);\n\
  colors[8] = vec4(0.7,0.7,0.7,1.0);\n\
  colors[9] = vec4(1.0,0.5,0.5,1.0);\n\
  colors[10] = vec4(0.5,1.0,0.5,1.0);\n\
  colors[11] = vec4(0.5,0.5,1.0,1.0);\n\
  vec4 color = vec4(0.0,0.0,0.0,0.0);\n\
  // Number of influencing handles\n\
  int m = int(min(num_handles,MAX_NUM_WEIGHTS_PER_VERTEX));\n\
\n\
  // First 4\n\
  for(int k = 0; k<min(m,4);k++)\n\
  {\n\
    // influencing handle index\n\
    int i = int(weight_indices0[k]);\n\
    color +=\n\
      colors[i]*weights0[k];\n\
  }\n\
  // Next 4\n\
  for(int k = 4; k<min(m,8);k++)\n\
  {\n\
    // influencing handle index\n\
    int i = int(weight_indices4[k-4]);\n\
    color +=\n\
      colors[i]*weights4[k-4];\n\
  }\n\
  normal = normalize(gl_NormalMatrix * gl_Normal);\n\
  lightDir = normalize(vec3(gl_LightSource[0].position));\n\
  halfVector = normalize(gl_LightSource[0].halfVector.xyz);\n\
  float NdotL = dot(normal,lightDir);\n\
  gl_FrontColor = \n\
    color*gl_LightSource[0].diffuse*NdotL + \n\
    gl_FrontMaterial.ambient * gl_LightSource[0].ambient + \n\
    gl_LightModel.ambient * gl_FrontMaterial.ambient;\n\
} \n\
";
char weightcolors_frag[] = "\
varying vec4 diffuse,ambient;\n\
varying vec3 normal,lightDir,halfVector;\n\
\n\
void main()\n\
{\n\
    //vec3 n,halfV;\n\
    //float NdotL,NdotHV;\n\
    //\n\
    //vec4 color = ambient;\n\
    //\n\
    //n = normalize(normal);\n\
    //\n\
    //NdotL = max(dot(n,lightDir),0.0);\n\
    //\n\
    //if (NdotL > 0.0) {\n\
    //    color += diffuse * NdotL;\n\
    //    halfV = normalize(halfVector);\n\
    //    NdotHV = max(dot(n,halfV),0.0);\n\
    //    color += gl_Color * gl_LightSource[0].specular * pow(NdotHV, gl_FrontMaterial.shininess);\n\
    //}\n\
    //    \n\
    //gl_FragColor = color;\n\
\n\
    float NdotL,NdotHV;\n\
    \n\
    //vec4 color = ambient;\n\
    vec4 color = gl_Color;\n\
    \n\
    NdotL = dot(normal,lightDir);\n\
    \n\
    if (NdotL > 0.0) {\n\
        //color += diffuse * NdotL;\n\
        NdotHV = max(dot(normal,halfVector),0.0);\n\
        NdotHV = pow(NdotHV,gl_FrontMaterial.shininess);\n\
        //NdotHV *= NdotHV;\n\
        //NdotHV *= NdotHV;\n\
        //NdotHV *= NdotHV;\n\
        //NdotHV *= NdotHV;\n\
        color += gl_Color * gl_LightSource[0].specular * NdotHV;\n\
    }\n\
        \n\
    gl_FragColor = color;\n\
\n\
}\n\
\n\
";
