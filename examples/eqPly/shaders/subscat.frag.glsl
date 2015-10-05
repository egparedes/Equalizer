
/* Copyright (c) 2015, Enrique g. Paredes <egparedes@ifi.uzh.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of Eyescale Software GmbH nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */ 
    
// Fragment shader 
///* --------------------------
//SubScatter Fragment Shader:
// 
//Fake sub-surface scatter lighting shader by InvalidPointer 2008.
//Found at
//http://www.gamedev.net/community/forums/topic.asp?topic_id=481494
// 
//HLSL > GLSL translation
//toneburst 2008
//-------------------------- */
 
// Variables for lighting properties
//uniform float MaterialThickness;
//uniform vec3 ExtinctionCoefficient; // Will show as X Y and Z ports in QC, but actually represent RGB values.
//uniform vec4 LightColor;
//uniform vec4 BaseColor;
//uniform vec4 SpecColor;
//uniform float SpecPower;
//uniform float RimScalar;
//uniform sampler2D Texture;
 
// Varying variables to be sent to Fragment Shader
varying vec3 worldNormal, eyeVec, lightVec, vertPos, lightPos;

float MaterialThickness = 0.5;
vec3 ExtinctionCoefficient = vec3(0.3, 0.3, 0.3);
vec4 LightColor = vec4(1.0, 1.0, 1.0, 1.0);
vec4 BaseColor = gl_Color;
vec4 SpecColor = BaseColor * 0.5;
float SpecPower = 1.0;
float RimScalar = 1.0;
 
float halfLambert(in vec3 vect1, in vec3 vect2)
{
    float product = dot(vect1,vect2);
    return product * 0.5 + 0.5;
}
 
float blinnPhongSpecular(in vec3 normalVec, in vec3 lightVec, in float specPower)
{
    vec3 halfAngle = normalize(normalVec + lightVec);
    return pow(clamp(0.0,1.0,dot(normalVec,halfAngle)),specPower);
}
 
// Main fake sub-surface scatter lighting function
vec4 subScatterFS()
{
    float attenuation = 10.0 * (1.0 / distance(lightPos,vertPos));
    vec3 eVec = normalize(eyeVec);
    vec3 lVec = normalize(lightVec);
    vec3 wNorm = normalize(worldNormal);
     
    vec4 dotLN = vec4(halfLambert(lVec,wNorm) * attenuation);
    //dotLN *= texture2D(Texture, gl_TexCoord[0].xy);
    dotLN *= BaseColor;
     
    vec3 indirectLightComponent = vec3(MaterialThickness * max(0.0,dot(-wNorm,lVec)));
    indirectLightComponent += MaterialThickness * halfLambert(-eVec,lVec);
    indirectLightComponent *= attenuation;
    indirectLightComponent.r *= ExtinctionCoefficient.r;
    indirectLightComponent.g *= ExtinctionCoefficient.g;
    indirectLightComponent.b *= ExtinctionCoefficient.b;
     
    vec3 rim = vec3(1.0 - max(0.0,dot(wNorm,eVec)));
    rim *= rim;
    rim *= max(0.0,dot(wNorm,lVec)) * SpecColor.rgb;
     
    vec4 finalCol = dotLN + vec4(indirectLightComponent,1.0);
    finalCol.rgb += (rim * RimScalar * attenuation * finalCol.a);
    finalCol.rgb += vec3(blinnPhongSpecular(wNorm,lVec,SpecPower) * attenuation * SpecColor * finalCol.a * 0.05);
    finalCol.rgb *= LightColor.rgb;
     
    return finalCol;   
}

 
////////////////
//  MAIN LOOP //
////////////////
 
void main()
{
    gl_FragColor = subScatterFS();
}