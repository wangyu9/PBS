vec3  Color1=vec3(1,0,0);
vec3  Color2=vec3(0,0,1);
vec3  AvgColor=vec3(0.5,0.5,0.5);

float uvFrequency=8.;
vec4  uvRotation=vec4(1.,0.,0.,1.);

varying vec2  TexCoord;

void main()
{
    vec3 color;
    
    vec2 uv = mat2(uvRotation.x, uvRotation.y, uvRotation.z, uvRotation.w)*TexCoord;
        
    // Determine the width of the projection of one pixel into s-t space
    vec2 fw = fwidth(uv);

    // Determine the amount of fuzziness
    vec2 fuzz = fw * uvFrequency * 2.0;

    float fuzzMax = max(fuzz.s, fuzz.t);

    // Determine the position in the checkerboard pattern
    vec2 checkPos = fract(uv * uvFrequency);
    
    // If the filter width is small enough, compute the pattern color
    vec2 p = (smoothstep(vec2(1.0) - fuzz, vec2(1.0), checkPos)) +
             (1.0 - smoothstep(vec2(0.0), fuzz, checkPos));

    color = mix(vec3(1.0,1.0,1.0), vec3(0.0, 0.0, 1.0), p.x) * 
            mix(vec3(1.0,1.0,1.0), vec3(1.0, 0.0, 0.0), p.y) ;
            
    // Fade in the average color when we get close to the limit
    //color = mix(color, AvgColor, smoothstep(0.125, 0.5, fuzzMax));

	color.g = min(color.r, color.b);
    gl_FragColor = vec4(color, 1.0);
    
//    gl_FragColor = vec4(0.0,1.0,1.0,1.0);
}