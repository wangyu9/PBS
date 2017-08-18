//#define MAX_NUM_WEIGHTS_PER_VERTEX 4

// rendering
varying vec4 diffuse,ambient;
varying vec3 normal,lightDir,halfVector;

uniform mat4 my_ModelViewMatrix;
uniform mat4 my_ProjectionMatrix;

uniform vec4 my_LightSource_position;
uniform vec4 my_LightSource_ambient;
uniform vec4 my_LightSource_diffuse;
uniform vec4 my_LightSource_halfVector;

uniform vec4 my_FrontMaterial_diffuse;
uniform vec4 my_FrontMaterial_ambient;
uniform vec4 my_LightModel_ambient;
// skinning
uniform vec4 handle_pos[256];

attribute vec4 v_vertex;
attribute vec4 v_normal;

attribute vec4 weights0;
attribute vec4 weight_indices0;

/* attribute vec4 weights1;
attribute vec4 weight_indices1;

attribute vec4 weights2;
attribute vec4 weight_indices2;

attribute vec4 weights3;
attribute vec4 weight_indices3;

attribute vec4 weights4;
attribute vec4 weight_indices4;

attribute vec4 weights5;
attribute vec4 weight_indices5; */

void main()
{	
	// skinning
	//vec4 pos4 = vec4(handle_pos,1);
/* mat4 blend = mat4(0,0,0,0,
					 0,0,0,0,
					 0,0,0,0,
					 0,0,0,0); */
					 
	//weights0 = vec4(0.5,0.5,0,0);				 
					 
					 
	mat3 I3 =  mat3(1,0,0,
					0,1,0,
					0,0,1);
	mat4 I4 = 	mat4(1,0,0,0,
					 0,1,0,0,
					 0,0,1,0,
					 0,0,0,1);// Notice: All matrix are ROW-major in GLSL!
    //vec3 q0 = I3 * weights4 * handle_pos[0];

	//normal = normalize(gl_NormalMatrix * gl_Normal);
	normal = vec3(v_normal);//normalize( gl_NormalMatrix * vec3(v_normal) );
	
	/*lightDir = normalize(vec3(gl_LightSource[0].position));
	halfVector = normalize(gl_LightSource[0].halfVector.xyz);
	diffuse = gl_FrontMaterial.diffuse * gl_LightSource[0].diffuse;
    ambient = gl_FrontMaterial.ambient * gl_LightSource[0].ambient;
    ambient += gl_LightModel.ambient * gl_FrontMaterial.ambient;*/

	//rendering
     	
    lightDir = normalize(vec3(my_LightSource_position));
    halfVector = normalize(my_LightSource_halfVector.xyz);
   	diffuse = my_FrontMaterial_diffuse * my_LightSource_diffuse;
    ambient = my_FrontMaterial_ambient * my_LightSource_ambient;
    ambient += my_LightModel_ambient * my_FrontMaterial_ambient;

	
	//diffuse = diffuse - 0.5*weights0;
	//ambient = weights0;
	//ambient = 1.0/155.0*weight_indices0;
//weights0;
//weight_indices0;
	vec4 just_cite_this_not_use = handle_pos[0];	
	vec4 _pos = vec4(0.0,0.0,0.0,0.0);
		
 	for(int k=0; k<4; k++)
	{
		// influencing handle index
		int i = int(weight_indices0[k]); 
		vec4 temp = handle_pos[i];		
		temp.w = 1.0;
		_pos += temp * 1.0 * weights0[k];
		//_pos = handle_pos[i];_pos.w = 1.0;
		
		//diffuse = diffuse + handle_pos[i]*0.6;
		//ambient = ambient - handle_pos[i]*0.6;
		
		if(k==0)
		{	
			//ambient = weights0;//
			diffuse = weights0;
		}
		//ambient = 0.0 * vec4(1.0,1.0,1.0,1.0)*handle_pos[i].x;//handle_pos[i]*0.8;
		
		//if(i<0)
		//	ambient = vec4(1.0,1.0,1.0,1.0);
	} 
//#define HANDLE_4
#ifdef HANDLE_4			
 	for(int k=0; k<4; k++)
	{
		// influencing handle index
		int i = int(weight_indices1[k]); 
		vec4 temp = handle_pos[i];		
		temp.w = 1.0;
		_pos += temp * 1.0 * weights1[k];
		//if(k==0)
		//	ambient = handle_pos[i]*0.8;
		//ambient = weights1;
		//_pos = handle_pos[i];_pos.w = 1.0;
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

	for(int k=0; k<4; k++)
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
	} 
#endif

	//gl_Vertex = _pos;

  //vec4 v = blend * gl_Vertex;
	_pos.x += 155.0; _pos.x -= 155.0;
	
	vec4 test = handle_pos[0];
	
	//_pos = vec3(weights0);
	//_pos.w=1;
     if(_pos.w>0.0)
		_pos = _pos *(1.0 /_pos.w);
	else
		_pos.w=1.0;   
	//_pos = vec4(0,0,0,0);
	//vec4 who_knows_why_I_must_use_gl_Vertex = gl_Vertex;//vec4(1.0,0.0,0.0,1.0);
	
	//_pos = _pos*1.0 + 0.5*gl_Vertex;
	//_pos = _pos*1.0 - 0.5*gl_Vertex;
	//gl_Position =  gl_ModelViewProjectionMatrix * _pos;// * 0.9 + 0.1 * ftransform();// (_pos * 0.8 + gl_Vertex * 0.2);// + 0.1 * ftransform();
	//gl_Position.w = 1;
	//gl_Position = ftransform();
	//gl_Position =  gl_ModelViewProjectionMatrix * (v_vertex*0.001+0.999*_pos);
	gl_Position =  my_ModelViewMatrix * my_ProjectionMatrix * (v_vertex*0.999+0.001*_pos);
} 
