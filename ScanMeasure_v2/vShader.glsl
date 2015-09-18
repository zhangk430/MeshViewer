#version 330 core

uniform mat4 ModelViewMatrix;
uniform mat4 ProjectionMatrix;
uniform mat4 depthMatrix;

layout(location = 0) in vec4 position;
layout(location = 1) in vec4 color;
layout(location = 2) in vec3 normal;

out vec4 fragVert;
out vec3 fragNormal;
out vec4 colorV;
out vec4 shadowCoord;

void main (void)
{
    colorV = color;
	fragNormal = normal;
	fragVert = position;
	shadowCoord = depthMatrix * position;
    gl_Position = ProjectionMatrix * ModelViewMatrix * position;
}