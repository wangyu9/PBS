// DO NOT CHANGE ANYTHING BELOW THIS LINE - AUTOMATICALLY GENERATED
//==
char scalar_vert[] = "\
// Visualize weights on a mesh. Each weight function gets a pseudocolor and\n\
// these colors are belnded according to value of weights.\n\
\n\
// Necessary to use mat4x3 syntax for non-square matrices, otherwise compiler\n\
// treats code as version 110 which does not support non-square matrices\n\
#version 120\n\
\n\
\n\
#define MAT4\n\
#define MAX_NUMBER_OF_HANDLES 100\n\
#define MAX_NUMBER_OF_WEIGHTS_PER_VERTEX 16\n\
\n\
varying vec4 diffuse,ambient;\n\
varying vec3 normal,lightDir,halfVector;\n\
attribute vec4 weights0;\n\
attribute vec4 weight_indices0;\n\
#if MAX_NUMBER_OF_WEIGHTS_PER_VERTEX > 4\n\
attribute vec4 weights4;\n\
attribute vec4 weight_indices4;\n\
#endif\n\
#if MAX_NUMBER_OF_WEIGHTS_PER_VERTEX > 8\n\
attribute vec4 weights8;\n\
attribute vec4 weight_indices8;\n\
#endif\n\
#if MAX_NUMBER_OF_WEIGHTS_PER_VERTEX > 12\n\
attribute vec4 weights12;\n\
attribute vec4 weight_indices12;\n\
#endif\n\
// Number of handles\n\
uniform int number_of_handles;\n\
// Selected weight function\n\
uniform int selected_weight;\n\
\n\
// Functions\n\
int min(int a, int b) { return a < b ? a : b; }\n\
// Return a color based on scalar value\n\
vec4 heat_color(float s)\n\
{\n\
  vec4 zero = vec4(1.0,1.0,1.0,1.0);\n\
  vec4 one = vec4(1.0,0.0,0.0,1.0);\n\
  vec4 plus = vec4(1.0,1.0,0.0,1.0);\n\
  vec4 minus = vec4(0.0,0.0,1.0,1.0);\n\
  if(s >= 2)\n\
  {\n\
    return plus;\n\
  }else if(s >= 1)\n\
  {\n\
    return (1-(s-1))*one + (s-1)*plus;\n\
  }else if(s>=0)\n\
  {\n\
    return (1-s)*zero+ (s)*one;\n\
  }else if(s>=-1)\n\
  {\n\
    return (1-(s+1))*minus+ (s+1)*zero;\n\
  }\n\
  return minus;\n\
}\n\
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
\n\
  // Initialize assuming zero weight\n\
  vec4 color = heat_color(0.0);\n\
\n\
  // Number of influencing handles\n\
  int m = int(min(number_of_handles,MAX_NUMBER_OF_WEIGHTS_PER_VERTEX));\n\
\n\
  // First 4\n\
  int k_max = int(min(m,0+4))-(0);\n\
  for(int k = 0; k<k_max;k++)\n\
  {\n\
    // influencing handle index\n\
    int i = int(weight_indices0[k]);\n\
    if(i == selected_weight)\n\
    {\n\
      color = heat_color(weights0[k]);\n\
    }\n\
  }\n\
\n\
#if MAX_NUMBER_OF_WEIGHTS_PER_VERTEX > 4\n\
  // Next 4\n\
  k_max = int(min(m,4+4))-(4);\n\
  for(int k = 0; k<k_max;k++)\n\
  {\n\
    // influencing handle index\n\
    int i = int(weight_indices4[k]);\n\
    if(i == selected_weight)\n\
    {\n\
      color = heat_color(weights4[k]);\n\
    }\n\
  }\n\
#endif\n\
\n\
#if MAX_NUMBER_OF_WEIGHTS_PER_VERTEX > 8\n\
  // Next 4\n\
  k_max = int(min(m,8+4))-(8);\n\
  for(int k = 0; k<k_max;k++)\n\
  {\n\
    // influencing handle index\n\
    int i = int(weight_indices8[k]);\n\
    if(i == selected_weight)\n\
    {\n\
      color = heat_color(weights8[k]);\n\
    }\n\
  }\n\
#endif\n\
\n\
#if MAX_NUMBER_OF_WEIGHTS_PER_VERTEX > 12\n\
  // Next 4\n\
  k_max = int(min(m,12+4))-(12);\n\
  for(int k = 0; k<k_max;k++)\n\
  {\n\
    // influencing handle index\n\
    int i = int(weight_indices12[k]);\n\
    if(i == selected_weight)\n\
    {\n\
      color = heat_color(weights12[k]);\n\
    }\n\
  }\n\
#endif\n\
\n\
  normal = normalize(gl_NormalMatrix * gl_Normal);\n\
  lightDir = normalize(vec3(gl_LightSource[0].position));\n\
  halfVector = normalize(gl_LightSource[0].halfVector.xyz);\n\
  float NdotL = dot(normal,lightDir);\n\
  gl_FrontColor = \n\
    color*gl_LightSource[0].diffuse*NdotL + \n\
    gl_FrontMaterial.ambient * gl_LightSource[0].ambient + \n\
    gl_LightModel.ambient * gl_FrontMaterial.ambient;\n\
} \n\
\n\
";
char scalar_frag[] = "\
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
