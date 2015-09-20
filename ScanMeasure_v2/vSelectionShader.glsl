#version 330 core

uniform mat4 ModelViewMatrix;
uniform mat4 ProjectionMatrix;

layout(location = 0) in vec4 position;
layout(location = 1) in vec4 color;

out vec4 colorV;

void main (void)
{
	colorV = color;
    gl_Position = ProjectionMatrix * ModelViewMatrix * position;
}