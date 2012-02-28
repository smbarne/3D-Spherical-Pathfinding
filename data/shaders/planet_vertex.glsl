const int NumEnabledLights = 2;
varying vec4 colorAmbientEmissive; 

void DynamicShadow( in vec4 ecPosition )
{   
    // generate coords for shadow mapping      
    gl_TexCoord[3].s = dot( ecPosition, gl_EyePlaneS[3] );
    gl_TexCoord[3].t = dot( ecPosition, gl_EyePlaneT[3] );
    gl_TexCoord[3].p = dot( ecPosition, gl_EyePlaneR[3] );
    gl_TexCoord[3].q = dot( ecPosition, gl_EyePlaneQ[3] );
}

void PointLight(in int i,
                in vec3 eye, 
                in vec3 ecPosition3,
                in vec3 normal,    
                inout vec4 ambient,
                inout vec4 diffuse,
                inout vec4 specular)
{
        float nDotVP;      // normal . light direction
        float nDotHV;      // normal . light half vector 
        float pf;   // power factor
        float attenuation; // computed attenuation factor 
        float d;    // distance from surface to light source 
        vec3  VP;   // direction from surface to light position 
        vec3  halfVector;  // direction of maximum highlights    
         
        // Compute vector from surface to light position 
        VP = vec3(gl_LightSource[i].position) - ecPosition3;    
         
        // Compute distance between surface and light position    
        d = length(VP);    
         
        // Normalize the vector from surface to light position  
        VP = normalize(VP);
         
        // Compute attenuation
        attenuation = 1.0 / (gl_LightSource[i].constantAttenuation + 
          gl_LightSource[i].linearAttenuation * d + 
          gl_LightSource[i].quadraticAttenuation * d*d);
         
        halfVector = normalize(VP + eye);
         
        nDotVP = max(0.0, dot(normal, VP)); 
        nDotHV = max(0.0, dot(normal, halfVector));
         
        if (nDotVP == 0.0)    
            pf = 0.0; 
        else    
            pf = pow(nDotHV, gl_FrontMaterial.shininess);      
         
        ambient += gl_LightSource[i].ambient * attenuation;    
        diffuse += gl_LightSource[i].diffuse * nDotVP * attenuation; 
        specular += gl_LightSource[i].specular * pf * attenuation;
}
  
void main()
{
    // Transform vertex to clip space
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
    vec3 normal = normalize( gl_NormalMatrix * gl_Normal );
     
    vec4  ecPos  = gl_ModelViewMatrix * gl_Vertex;
    float ecLen  = length( ecPos );
    vec3  ecPosition3 = ecPos.xyz / ecPos.w;        
    vec3  eye = -normalize(ecPosition3);     

    DynamicShadow( ecPos ); 
     
    gl_TexCoord[2] = gl_TextureMatrix[2] * gl_MultiTexCoord0;   
     
    // Clear the light intensity accumulators    
    vec4 amb  = vec4(0.0);
    vec4 diff = vec4(0.0);
    vec4 spec = vec4(0.0);
     
    // Loop through enabled lights, compute contribution from each 
    for (int i = 0; i < NumEnabledLights; i++)  
    {
        PointLight(i, eye, ecPosition3, normal, amb, diff, spec);
    }      
     
    colorAmbientEmissive = gl_FrontLightModelProduct.sceneColor + 
        amb * gl_FrontMaterial.ambient;

    gl_FrontColor = colorAmbientEmissive +     
        diff * gl_FrontMaterial.diffuse;
    gl_FrontSecondaryColor = vec4(spec*gl_FrontMaterial.specular);     
     
    gl_BackColor = gl_FrontColor;    
    gl_BackSecondaryColor = gl_FrontSecondaryColor; 
     
    gl_FogFragCoord = ecLen; 
 }