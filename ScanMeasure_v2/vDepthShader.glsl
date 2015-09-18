#version 330 core

uniform mat4 depthMatrix;

layout(location = 0) in vec4 position;

void main (void)
{
    gl_Position = depthMatrix * position;
}