
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
    
// Fragment shader for sub-surface scatter and procedural noise functions
// 
///* --------------------------
//SubScatter Vertex Shader:
// 
//Fake sub-surface scatter lighting shader by InvalidPointer 2008.
//Found at
//http://www.gamedev.net/community/forums/topic.asp?topic_id=481494
// 
//HLSL > GLSL translation
//toneburst 2008
//-------------------------- */

// Procedural noise functions from (c) Ashima Arts
// URL: https://github.com/ashima/webgl-noise

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
varying vec3 v_texCoord3D;
varying vec3 worldNormal, eyeVec, lightVec, vertPos, lightPos;

float time = 1.0;

float MaterialThickness = 0.25;
vec3 ExtinctionCoefficient = vec3(0.25, 0.25, 0.25);
vec4 LightColor = vec4(0.5, 0.5, 0.5, 1.0);
//vec4 BaseColor = gl_Color;
vec4 SpecColor = vec4(LightColor.x * 0.5, LightColor.y * 0.25, LightColor.x * 0.15, LightColor.w);
float SpecPower = 2.0;
float RimScalar = 0.25;
 
vec4 mod289(vec4 x) {
  return x - floor(x * (1.0 / 289.0)) * 289.0; }

float mod289(float x) {
  return x - floor(x * (1.0 / 289.0)) * 289.0; }

vec4 permute(vec4 x) {
     return mod289(((x*34.0)+1.0)*x);
}

float permute(float x) {
     return mod289(((x*34.0)+1.0)*x);
}

vec4 taylorInvSqrt(vec4 r)
{
  return 1.79284291400159 - 0.85373472095314 * r;
}

float taylorInvSqrt(float r)
{
  return 1.79284291400159 - 0.85373472095314 * r;
}

vec4 grad4(float j, vec4 ip)
  {
  const vec4 ones = vec4(1.0, 1.0, 1.0, -1.0);
  vec4 p,s;

  p.xyz = floor( fract (vec3(j) * ip.xyz) * 7.0) * ip.z - 1.0;
  p.w = 1.5 - dot(abs(p.xyz), ones.xyz);
  s = vec4(lessThan(p, vec4(0.0)));
  p.xyz = p.xyz + (s.xyz*2.0 - 1.0) * s.www; 

  return p;
  }
                        
// (sqrt(5) - 1)/4 = F4, used once below
#define F4 0.309016994374947451

float snoise(vec3 v3)
  {
  vec4 v = vec4(v3, 0.5);
  const vec4  C = vec4( 0.138196601125011,  // (5 - sqrt(5))/20  G4
                        0.276393202250021,  // 2 * G4
                        0.414589803375032,  // 3 * G4
                       -0.447213595499958); // -1 + 4 * G4

// First corner
  vec4 i  = floor(v + dot(v, vec4(F4)) );
  vec4 x0 = v -   i + dot(i, C.xxxx);

// Other corners

// Rank sorting originally contributed by Bill Licea-Kane, AMD (formerly ATI)
  vec4 i0;
  vec3 isX = step( x0.yzw, x0.xxx );
  vec3 isYZ = step( x0.zww, x0.yyz );
//  i0.x = dot( isX, vec3( 1.0 ) );
  i0.x = isX.x + isX.y + isX.z;
  i0.yzw = 1.0 - isX;
//  i0.y += dot( isYZ.xy, vec2( 1.0 ) );
  i0.y += isYZ.x + isYZ.y;
  i0.zw += 1.0 - isYZ.xy;
  i0.z += isYZ.z;
  i0.w += 1.0 - isYZ.z;

  // i0 now contains the unique values 0,1,2,3 in each channel
  vec4 i3 = clamp( i0, 0.0, 1.0 );
  vec4 i2 = clamp( i0-1.0, 0.0, 1.0 );
  vec4 i1 = clamp( i0-2.0, 0.0, 1.0 );

  //  x0 = x0 - 0.0 + 0.0 * C.xxxx
  //  x1 = x0 - i1  + 1.0 * C.xxxx
  //  x2 = x0 - i2  + 2.0 * C.xxxx
  //  x3 = x0 - i3  + 3.0 * C.xxxx
  //  x4 = x0 - 1.0 + 4.0 * C.xxxx
  vec4 x1 = x0 - i1 + C.xxxx;
  vec4 x2 = x0 - i2 + C.yyyy;
  vec4 x3 = x0 - i3 + C.zzzz;
  vec4 x4 = x0 + C.wwww;

// Permutations
  i = mod289(i); 
  float j0 = permute( permute( permute( permute(i.w) + i.z) + i.y) + i.x);
  vec4 j1 = permute( permute( permute( permute (
             i.w + vec4(i1.w, i2.w, i3.w, 1.0 ))
           + i.z + vec4(i1.z, i2.z, i3.z, 1.0 ))
           + i.y + vec4(i1.y, i2.y, i3.y, 1.0 ))
           + i.x + vec4(i1.x, i2.x, i3.x, 1.0 ));

// Gradients: 7x7x6 points over a cube, mapped onto a 4-cross polytope
// 7*7*6 = 294, which is close to the ring size 17*17 = 289.
  vec4 ip = vec4(1.0/294.0, 1.0/49.0, 1.0/7.0, 0.0) ;

  vec4 p0 = grad4(j0,   ip);
  vec4 p1 = grad4(j1.x, ip);
  vec4 p2 = grad4(j1.y, ip);
  vec4 p3 = grad4(j1.z, ip);
  vec4 p4 = grad4(j1.w, ip);

// Normalise gradients
  vec4 norm = taylorInvSqrt(vec4(dot(p0,p0), dot(p1,p1), dot(p2, p2), dot(p3,p3)));
  p0 *= norm.x;
  p1 *= norm.y;
  p2 *= norm.z;
  p3 *= norm.w;
  p4 *= taylorInvSqrt(dot(p4,p4));

// Mix contributions from the five corners
  vec3 m0 = max(0.6 - vec3(dot(x0,x0), dot(x1,x1), dot(x2,x2)), 0.0);
  vec2 m1 = max(0.6 - vec2(dot(x3,x3), dot(x4,x4)            ), 0.0);
  m0 = m0 * m0;
  m1 = m1 * m1;
  return 49.0 * ( dot(m0*m0, vec3( dot( p0, x0 ), dot( p1, x1 ), dot( p2, x2 )))
               + dot(m1*m1, vec2( dot( p3, x3 ), dot( p4, x4 ) ) ) ) ;

  }

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
 // Perturb the texcoords with three components of noise
  vec3 uvw = v_texCoord3D + 0.1*vec3(snoise(v_texCoord3D + vec3(0.0, 0.0, time)),
    snoise(v_texCoord3D + vec3(43.0, 17.0, time)),
    snoise(v_texCoord3D + vec3(-17.0, -43.0, time)));
  // Six components of noise in a fractal sum
  float n = snoise(uvw - vec3(0.0, 0.0, time));
  n += 0.5 * snoise(uvw * 2.0 - vec3(0.0, 0.0, time*1.4)); 
  n += 0.25 * snoise(uvw * 4.0 - vec3(0.0, 0.0, time*2.0)); 
  n += 0.125 * snoise(uvw * 8.0 - vec3(0.0, 0.0, time*2.8)); 
  n += 0.0625 * snoise(uvw * 16.0 - vec3(0.0, 0.0, time*4.0)); 
  n += 0.03125 * snoise(uvw * 32.0 - vec3(0.0, 0.0, time*5.6)); 
  n = n * 0.7;
  // A "hot" colormap - cheesy but effective 
  vec4 BaseColor = 0.25 * gl_Color * vec4(vec3(1.0, 0.5, 0.0) + vec3(n, n, n), 1.0);


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