#version 330 core

uniform float scaleX; 
uniform float scaleY; 
uniform float xpos;  
uniform float ypos; 
uniform vec3 textColor; 

layout(location = 0) in vec2 inVert; 
layout(location = 1) in vec2 inUV; 
out vec2 vertUV; 

void main() 
{ 
	vertUV=inUV; 
	gl_Position=vec4( ((xpos+inVert.x)*scaleX)-1.0,((ypos+inVert.y)*scaleY)+1.0,0.0,1.0);
}
