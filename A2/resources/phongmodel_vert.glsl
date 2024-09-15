#version 410

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNor;
layout (location = 2) in vec2 aTex;

uniform mat4 P;
uniform mat4 MV;

out vec3 vPos;
out vec3 vNor;
out vec2 vTex;

void main()
{
	vec4 posCam = MV * vec4(aPos, 1.0);
	vec3 norCam = (MV * vec4(aNor, 0.0)).xyz;
	gl_Position = P * posCam;
	vPos = posCam.xyz;
	vNor = norCam;
	vTex = aTex;
}
