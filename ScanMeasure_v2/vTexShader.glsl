#version 330 core

uniform mat4 ModelViewMatrix;
uniform mat4 ProjectionMatrix;
uniform mat4 depthMatrix;

layout(location = 0) in vec4 position;
layout(location = 2) in vec3 normal;
layout(location = 3) in vec2 vTexCoord;

out vec4 fragVert;
out vec3 fragNormal;
out vec2 fTexCoord;
out vec4 shadowCoord;

void main (void)
{
	fTexCoord = vTexCoord;
	fragNormal = normal;
	fragVert = position;
	shadowCoord = depthMatrix * position;
    gl_Position = ProjectionMatrix * ModelViewMatrix * position;
}