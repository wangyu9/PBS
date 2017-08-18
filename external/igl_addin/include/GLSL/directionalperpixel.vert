varying vec4 diffuse,ambient;
varying vec3 normal,lightDir,halfVector;

void main()
{	
    normal = normalize(gl_NormalMatrix * gl_Normal);
    lightDir = normalize(vec3(gl_LightSource[0].position));
	
	vec4 trans_eye_position = vec4(0,0,1,0);
	viewerDir = normalize(trans_eye_position.xyz);
	halfVector = normalize(lightDir+viewerDir.xyz);
	// Original Version
    //halfVector = normalize(gl_LightSource[0].halfVector.xyz);
    
    diffuse = gl_FrontMaterial.diffuse * gl_LightSource[0].diffuse;
    ambient = gl_FrontMaterial.ambient * gl_LightSource[0].ambient;
    ambient += gl_LightModel.ambient * gl_FrontMaterial.ambient;
	
    gl_Position = ftransform();
} 
