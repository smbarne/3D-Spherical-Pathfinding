// Frag Shader
varying vec4 colorAmbientEmissive;
uniform sampler2D baseTexture;
uniform sampler2DShadow shadowTexture;

float DynamicShadow( ) 
{   
	return shadow2DProj( shadowTexture, gl_TexCoord[3] ).r;
}

void main()
{    
	vec4 color = texture2D( baseTexture, gl_TexCoord[2].xy );
	//color *= mix( colorAmbientEmissive, gl_Color, DynamicShadow() );
	color *= mix( gl_Color*0.65, gl_Color, DynamicShadow() );      

	gl_FragColor = color; 
}