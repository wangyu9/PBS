//#define MAX_NUM_WEIGHTS_PER_VERTEX 4

// rendering
varying vec4 diffuse,ambient;
varying vec3 normal,lightDir,halfVector;

// skinning
uniform vec4 handle_pos[256];

attribute vec4 weights0;
attribute vec4 weight_indices0;


void main()
{	
	// skinning
	//vec4 pos4 = vec4(handle_pos,1);
/* mat4 blend = mat4(0,0,0,0,
					 0,0,0,0,
					 0,0,0,0,
					 0,0,0,0); */
					 
	mat3 I3 =  mat3(1,0,0,
					0,1,0,
					0,0,1);
	mat4 I4 = 	mat4(1,0,0,0,
					 0,1,0,0,
					 0,0,1,0,
					 0,0,0,1);// Notice: All matrix are ROW-major in GLSL!
    //vec3 q0 = I3 * weights4 * handle_pos[0];


	//rendering
    normal = normalize(gl_NormalMatrix * gl_Normal);
    lightDir = normalize(vec3(gl_LightSource[0].position));
	
    halfVector = normalize(gl_LightSource[0].halfVector.xyz);
    
    diffuse = gl_FrontMaterial.diffuse * gl_LightSource[0].diffuse;
    ambient = gl_FrontMaterial.ambient * gl_LightSource[0].ambient;
    ambient += gl_LightModel.ambient * gl_FrontMaterial.ambient;
	
	//diffuse = diffuse - 0.5*weights0;
	//ambient = weights0;
	//ambient = 1.0/155.0*weight_indices0;
weights0;
weight_indices0;
handle_pos[0];	
	//vec4 _pos = vec4(0,0,0,0);
/* 	for(int k=0; k<1; k++)
	{
		// influencing handle index
		//int i = int(weight_indices0[k]);    
		//_pos += handle_pos[i] * weights0[k];
		//_pos = handle_pos[i];
		
		//diffuse = diffuse + handle_pos[i]*0.6;
		//ambient = ambient - handle_pos[i]*0.6;
		
		//ambient = handle_pos[i]*0.8;
	} */
	



  //vec4 v = blend * gl_Vertex;
	
	//vec4 test = handle_pos[0];
	
	//_pos = vec3(weights0);
	
	//gl_Position =  gl_ModelViewProjectionMatrix * vec4(vec3(weights0),1);
	
	gl_Position = ftransform();
} 
