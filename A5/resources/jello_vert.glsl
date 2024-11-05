#version 410

layout (location = 0) in vec3 aPos;
uniform mat4 P;
uniform mat4 MV;

void main()
{
	gl_Position = P * MV * vec4(aPos, 1);
}