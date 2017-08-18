// DO NOT CHANGE ANYTHING BELOW THIS LINE - AUTOMATICALLY GENERATED
//==
char lbs_vert[] = "\
// Necessary to use mat4x3 syntax for non-square matrices, otherwise compiler\n\
// treats code as version 110 which does not support non-square matrices\n\
#version 120\n\
\n\
// 61 goes into software renderer mode\n\
#define MAX_NUM_HANDLES 60\n\
#define MAX_NUM_WEIGHTS_PER_VERTEX 16\n\
\n\
// Transform normals by rotation portion of each transformation weighted by\n\
// corresponding skinning weight: a poor man's inverse transpose of the jacobian\n\
// (assumes transformations are locally rigid)\n\
#define TRANSFORM_NORMALS\n\
\n\
//#define NO_INDEXING_BY_VARIABLES\n\
#ifdef NO_INDEXING_BY_VARIABLES\n\
#  define IM_MAD_AS_HELL_AND_IM_NOT_GOING_TO_TAKE_IT_ANYMORE\n\
#endif\n\
\n\
#define MAT4\n\
// Allow exactly one of MAT4, MAT4x3 or MAT3x4 to be defined, defaulting to\n\
// MAT4\n\
#if defined( MAT4x3 )\n\
#  ifdef MAT3x4\n\
#    error\n\
#  endif\n\
#  ifdef MAT4\n\
#    error\n\
#  endif\n\
#elif defined( MAT3x4 )\n\
#  ifdef MAT4x3\n\
#    error\n\
#  endif\n\
#  ifdef MAT4\n\
#    error\n\
#  endif\n\
#elif defined( MAT4 )\n\
#  ifdef MAT4x3\n\
#    error\n\
#  endif\n\
#  ifdef MAT3x4\n\
#    error\n\
#  endif\n\
#else\n\
#  define MAT4\n\
#endif\n\
\n\
varying vec4 diffuse,ambient;\n\
varying vec3 normal,lightDir,halfVector;\n\
attribute vec4 weights0;\n\
attribute vec4 weight_indices0;\n\
#if MAX_NUM_WEIGHTS_PER_VERTEX > 4\n\
attribute vec4 weights4;\n\
attribute vec4 weight_indices4;\n\
#endif\n\
#if MAX_NUM_WEIGHTS_PER_VERTEX > 8\n\
attribute vec4 weights8;\n\
attribute vec4 weight_indices8;\n\
#endif\n\
#if MAX_NUM_WEIGHTS_PER_VERTEX > 12\n\
attribute vec4 weights12;\n\
attribute vec4 weight_indices12;\n\
#endif\n\
// List of transformations, one for each handle\n\
#if defined( MAT4 )\n\
uniform mat4 handle_transformation[MAX_NUM_HANDLES];\n\
#elif defined( MAT4x3 )\n\
uniform mat4x3 handle_transformation[MAX_NUM_HANDLES];\n\
#elif defined( MAT3x4 )\n\
uniform mat3x4 handle_transformation[MAX_NUM_HANDLES];\n\
#endif\n\
// Number of handles\n\
uniform int num_handles;\n\
\n\
// Functions\n\
int min(int a, int b) { return a < b ? a : b; }\n\
\n\
#ifdef NO_INDEXING_BY_VARIABLES\n\
// Hack to return handle_transformation[i]\n\
// Input:\n\
//   i  index into handle_transformation\n\
// Returns handle_transformation[i]\n\
mat4 handle_transformation_at(int i);\n\
#endif\n\
\n\
// per vertex color\n\
vec4 color = vec4(0.0,0.0,0.0,0.0);\n\
\n\
// Useful sources:\n\
// http://www.opengl.org/sdk/docs/man/xhtml/glUniform.xml \n\
//   mat4x3 --> 3 rows, 4 columns\n\
// http://www.opengl.org/sdk/docs/tutorials/ClockworkCoders/attributes.php\n\
// http://www.opengl.org/registry/doc/GLSLangSpec.Full.1.20.8.pdf \n\
// http://gaim.umbc.edu/2010/06/24/nonlinear-transformations/\n\
void main()\n\
{\n\
\n\
  color = gl_Color;\n\
  //color = gl_FrontMaterial.diffuse;\n\
\n\
  // Number of influencing handles\n\
  int m = int(min(num_handles,MAX_NUM_WEIGHTS_PER_VERTEX));\n\
\n\
  //////////////////////////////////////////////////////////////////////////\n\
  // LBS\n\
  //////////////////////////////////////////////////////////////////////////\n\
\n\
#ifdef TRANSFORM_NORMALS\n\
  vec4 n = vec4(0.0,0.0,0.0,0.0);\n\
#endif\n\
\n\
  // Alter vertex position\n\
#if defined( MAT4 )\n\
  vec4 v = vec4(0.0,0.0,0.0,0.0);\n\
#elif defined( MAT4x3 )\n\
  vec3 v = vec3(0.0,0.0,0.0);\n\
#elif defined( MAT3x4 )\n\
  vec3 v = vec3(0.0,0.0,0.0);\n\
#endif\n\
\n\
  int k_max = int(min(m,0+4))-(0);\n\
  for(int k = 0;k<k_max;k++)\n\
  {\n\
    // influencing handle index\n\
    int i = int(weight_indices0[k]);\n\
#if defined (NO_INDEXING_BY_VARIABLES)\n\
    mat4 Ti = handle_transformation_at(i);\n\
#else\n\
    mat4 Ti = handle_transformation[i];\n\
#endif\n\
\n\
#if defined( MAT4 )\n\
    v += weights0[k]*(Ti * gl_Vertex);\n\
#  ifdef TRANSFORM_NORMALS\n\
    n += weights0[k]*(Ti * vec4(gl_Normal.xyz,0));\n\
#  endif\n\
#elif defined( MAT4x3 )\n\
    v += weights0[k]*(Ti * gl_Vertex);\n\
#elif defined( MAT3x4 )\n\
    v += weights0[k]*(gl_Vertex * Ti);\n\
#endif\n\
  }\n\
\n\
\n\
#if MAX_NUM_WEIGHTS_PER_VERTEX > 4\n\
  // Next 4\n\
  k_max = int(min(m,4+4))-(4);\n\
  for(int k = 0;k<k_max;k++)\n\
  {\n\
    // influencing handle index\n\
    int i = int(weight_indices4[k]);\n\
#if defined (NO_INDEXING_BY_VARIABLES)\n\
    mat4 Ti = handle_transformation_at(i);\n\
#else\n\
    mat4 Ti = handle_transformation[i];\n\
#endif\n\
#if defined( MAT4 )\n\
    v += weights4[k]*(Ti * gl_Vertex);\n\
#  ifdef TRANSFORM_NORMALS\n\
    n += weights4[k]*(Ti * vec4(gl_Normal.xyz,0));\n\
#  endif\n\
#elif defined( MAT4x3 )\n\
    v += weights4[k]*(Ti * gl_Vertex);\n\
#elif defined( MAT3x4 )\n\
    v += weights4[k]*(gl_Vertex * handle_transformation[i]);\n\
#endif\n\
  }\n\
#endif\n\
\n\
\n\
#if MAX_NUM_WEIGHTS_PER_VERTEX > 8\n\
  // Next 4\n\
  k_max = int(min(m,8+4))-(8);\n\
  for(int k = 0;k<k_max;k++)\n\
  {\n\
    // influencing handle index\n\
    int i = int(weight_indices8[k]);\n\
#if defined (NO_INDEXING_BY_VARIABLES)\n\
    mat4 Ti = handle_transformation_at(i);\n\
#else\n\
    mat4 Ti = handle_transformation[i];\n\
#endif\n\
#if defined( MAT4 )\n\
    v += weights8[k]*(Ti * gl_Vertex);\n\
#  ifdef TRANSFORM_NORMALS\n\
    n += weights8[k]*(Ti * vec4(gl_Normal.xyz,0));\n\
#  endif\n\
#elif defined( MAT4x3 )\n\
    v += weights8[k]*(Ti * gl_Vertex);\n\
#elif defined( MAT3x4 )\n\
    v += weights8[k]*(gl_Vertex * handle_transformation[i]);\n\
#endif\n\
  }\n\
#endif\n\
\n\
#if MAX_NUM_WEIGHTS_PER_VERTEX > 12\n\
  // Next 4\n\
  k_max = int(min(m,12+4))-(12);\n\
  for(int k = 0;k<k_max;k++)\n\
  {\n\
    // influencing handle index\n\
    int i = int(weight_indices12[k]);\n\
#if defined (NO_INDEXING_BY_VARIABLES)\n\
    mat4 Ti = handle_transformation_at(i);\n\
#else\n\
    mat4 Ti = handle_transformation[i];\n\
#endif\n\
#if defined( MAT4 )\n\
    v += weights12[k]*(Ti * gl_Vertex);\n\
#  ifdef TRANSFORM_NORMALS\n\
    n += weights12[k]*(Ti * vec4(gl_Normal.xyz,0));\n\
#  endif\n\
#elif defined( MAT4x3 )\n\
    v += weights12[k]*(Ti * gl_Vertex);\n\
#elif defined( MAT3x4 )\n\
    v += weights12[k]*(gl_Vertex * handle_transformation[i]);\n\
#endif\n\
  }\n\
#endif\n\
\n\
  // Force homogenous w-coordinate to be one (same as ignoring LBS of\n\
  // w-coordinate, which could be garbage if weights don't sum to one for\n\
  // example)\n\
  v.w = 1;\n\
\n\
  //v.xyz = gl_Vertex.xyz;\n\
\n\
  // Finally, apply model view and projection matrix\n\
#if defined( MAT4 )\n\
  gl_Position =  gl_ModelViewProjectionMatrix * v;\n\
#elif defined( MAT4x3 )\n\
  gl_Position =  gl_ModelViewProjectionMatrix * vec4(v.xyz,1.0);\n\
#elif defined( MAT3x4 )\n\
  gl_Position =  gl_ModelViewProjectionMatrix * vec4(v.xyz,1.0);\n\
#endif\n\
\n\
  //color.xyz = handle_transformation[1][2].xyz;\n\
 // color.x = handle_transformation[0][0][3];\n\
 // color.y = handle_transformation[0][1][3];\n\
 // color.z = handle_transformation[0][2][3];\n\
\n\
  ////////////////////////////////////////////////////////////////////////////\n\
  //// Phong shading (per vertex colors)\n\
  ////////////////////////////////////////////////////////////////////////////\n\
  //vec4 color;\n\
  //color = gl_Color;\n\
  //normal = normalize(gl_NormalMatrix * gl_Normal);\n\
  //lightDir = normalize(vec3(gl_LightSource[0].position));\n\
  //halfVector = normalize(gl_LightSource[0].halfVector.xyz);\n\
  ////diffuse = gl_FrontMaterial.diffuse * gl_LightSource[0].diffuse;\n\
  //diffuse = color * gl_LightSource[0].diffuse;\n\
  //ambient = gl_FrontMaterial.ambient * gl_LightSource[0].ambient;\n\
  ////ambient = gl_Color * gl_LightSource[0].ambient;\n\
  //ambient += gl_LightModel.ambient * gl_FrontMaterial.ambient;\n\
  //// Tell fragment shader what the color is\n\
  //gl_FrontColor = color;\n\
\n\
  //// Color table\n\
  //vec4 colors[12];\n\
  //colors[0] = vec4(1.0,0.0,0.0,1.0);\n\
  //colors[1] = vec4(0.0,1.0,0.0,1.0);\n\
  //colors[2] = vec4(0.0,0.0,1.0,1.0);\n\
  //colors[3] = vec4(0.0,1.0,1.0,1.0);\n\
  //colors[4] = vec4(1.0,0.0,1.0,1.0);\n\
  //colors[5] = vec4(1.0,1.0,0.0,1.0);\n\
  //colors[6] = vec4(1.0,1.0,1.0,1.0);\n\
  //colors[7] = vec4(0.1,0.1,0.1,1.0);\n\
  //colors[8] = vec4(0.7,0.7,0.7,1.0);\n\
  //colors[9] = vec4(1.0,0.5,0.5,1.0);\n\
  //colors[10] = vec4(0.5,1.0,0.5,1.0);\n\
  //colors[11] = vec4(0.5,0.5,1.0,1.0);\n\
\n\
  ////color.xyz = handle_transformation[0][0].xyz;\n\
  ////color = vec4(1.0,0.0,0.0,0.0);\n\
  //  //colors[int(weight_indices[i][0])]*weights[i][0];\n\
  ////color.x = weights[asdf][0];\n\
  //// First 4\n\
  //for(int k = 0; k<min(m,4);k++)\n\
  //{\n\
  //  // influencing handle index\n\
  //  int i = int(weight_indices0[k]);\n\
  //  color +=\n\
  //    colors[i]*weights0[k];\n\
  //  //color = handle_transformation[k][1];\n\
  //}\n\
  //// Next 4\n\
  //for(int k = 4; k<min(m,8);k++)\n\
  //{\n\
  //  // influencing handle index\n\
  //  int i = int(weight_indices4[k-4]);\n\
  //  color +=\n\
  //    colors[i]*weights4[k-4];\n\
  //  //color = handle_transformation[k][1];\n\
  //}\n\
#ifdef TRANSFORM_NORMALS\n\
  normal = normalize(gl_NormalMatrix * n.xyz);\n\
#else\n\
  normal = normalize(gl_NormalMatrix * gl_Normal);\n\
#endif\n\
\n\
    lightDir = normalize(vec3(gl_LightSource[0].position));\n\
	\n\
    halfVector = normalize(gl_LightSource[0].halfVector.xyz);\n\
    \n\
    //diffuse = gl_FrontMaterial.diffuse * gl_LightSource[0].diffuse;\n\
    diffuse = gl_Color * gl_LightSource[0].diffuse;\n\
    ambient = gl_FrontMaterial.ambient * gl_LightSource[0].ambient;\n\
    ambient += gl_LightModel.ambient * gl_FrontMaterial.ambient;\n\
	\n\
} \n\
\n\
\n\
#ifdef NO_INDEXING_BY_VARIABLES\n\
mat4 handle_transformation_at(int i)\n\
{\n\
  //(1..10).each{|e| puts \"else if(i==#{e}) return handle_transformation[#{e}];\"}\n\
  if(i==0) return handle_transformation[0];\n\
  //if(i<=2)\n\
  //{\n\
  //  if(i==1) return handle_transformation[1];\n\
  //  return handle_transformation[2];\n\
  //}\n\
\n\
  //if(i<=6)\n\
  //{\n\
  //  if(i<=4)\n\
  //  {\n\
  //    if(i==3) return handle_transformation[3];\n\
  //    return handle_transformation[4];\n\
  //  }\n\
  //  if(i==5) return handle_transformation[5];\n\
  //  return handle_transformation[6];\n\
  //}\n\
\n\
  //if(i<=14)\n\
  //{\n\
  //  if(i<=10)\n\
  //  {\n\
  //    if(i<=8)\n\
  //    {\n\
  //      if(i==7) return handle_transformation[7];\n\
  //      return handle_transformation[8];\n\
  //    }\n\
  //    if(i==9) return handle_transformation[9];\n\
  //    return handle_transformation[10];\n\
  //  }\n\
  //  if(i<=12)\n\
  //  {\n\
  //    if(i==11) return handle_transformation[11];\n\
  //    return handle_transformation[12];\n\
  //  }\n\
  //  if(i==13) return handle_transformation[13];\n\
  //  return handle_transformation[14];\n\
  //}\n\
\n\
  //\n\
  //if(i<=30)\n\
  //{\n\
  //  if(i<=22)\n\
  //  {\n\
  //    if(i<=18)\n\
  //    {\n\
  //      if(i<=16)\n\
  //      {\n\
  //        if(i==15) return handle_transformation[15];\n\
  //        return handle_transformation[16];\n\
  //      }\n\
  //      if(i==17) return handle_transformation[17];\n\
  //      return handle_transformation[18];\n\
  //    }\n\
  //    if(i<=20)\n\
  //    {\n\
  //      if(i==19) return handle_transformation[19];\n\
  //      return handle_transformation[20];\n\
  //    }\n\
  //    if(i==21) return handle_transformation[21];\n\
  //    return handle_transformation[22];\n\
  //  }\n\
  //  if(i<=26)\n\
  //  {\n\
  //    if(i<=24)\n\
  //    {\n\
  //      if(i==23) return handle_transformation[23];\n\
  //      return handle_transformation[24];\n\
  //    }\n\
  //    if(i==25) return handle_transformation[25];\n\
  //    return handle_transformation[26];\n\
  //  }\n\
  //  if(i<=28)\n\
  //  {\n\
  //    if(i==27) return handle_transformation[27];\n\
  //    return handle_transformation[28];\n\
  //  }\n\
  //  if(i==29) return handle_transformation[29];\n\
  //  return handle_transformation[30];\n\
  //}\n\
  //if(i==31) return handle_transformation[31];\n\
  //if(i==32) return handle_transformation[32];\n\
\n\
  if(i==0) return handle_transformation[0];\n\
  else if(i==1) return handle_transformation[1];\n\
  else if(i==2) return handle_transformation[2];\n\
  else if(i==3) return handle_transformation[3];\n\
  else if(i==4) return handle_transformation[4];\n\
  else if(i==5) return handle_transformation[5];\n\
  else if(i==6) return handle_transformation[6];\n\
  else if(i==7) return handle_transformation[7];\n\
  else if(i==8) return handle_transformation[8];\n\
  else if(i==9) return handle_transformation[9];\n\
  else if(i==10) return handle_transformation[10];\n\
  else if(i==11) return handle_transformation[11];\n\
  else if(i==12) return handle_transformation[12];\n\
  else if(i==13) return handle_transformation[13];\n\
  else if(i==14) return handle_transformation[14];\n\
  else if(i==15) return handle_transformation[15];\n\
  else if(i==16) return handle_transformation[16];\n\
  else if(i==17) return handle_transformation[17];\n\
  else if(i==18) return handle_transformation[18];\n\
  else if(i==19) return handle_transformation[19];\n\
  else if(i==20) return handle_transformation[20];\n\
  else if(i==21) return handle_transformation[21];\n\
  else if(i==22) return handle_transformation[22];\n\
  else if(i==23) return handle_transformation[23];\n\
  else if(i==24) return handle_transformation[24];\n\
// You can always add more but things start to slow down and maybe even switch\n\
// to software renderer if there are too many lines here\n\
#if MAX_NUM_HANDLES > 25\n\
  else if(i==25) return handle_transformation[25];\n\
  else if(i==26) return handle_transformation[26];\n\
  else if(i==27) return handle_transformation[27];\n\
  else if(i==28) return handle_transformation[28];\n\
  else if(i==29) return handle_transformation[29];\n\
  else if(i==30) return handle_transformation[30];\n\
  else if(i==31) return handle_transformation[31];\n\
  else if(i==32) return handle_transformation[32];\n\
  //ERROR: 0:505: Index 100 beyond bounds (size 100)\n\
  //else if(i==100) return handle_transformation[100];\n\
#endif\n\
  color = vec4(1.0,0.0,1.0,1.0);\n\
  // dummy base case, be nice if I could make it crash if it ever got here\n\
  return handle_transformation[0];\n\
}\n\
#endif\n\
";
char lbs_frag[] = "\
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
