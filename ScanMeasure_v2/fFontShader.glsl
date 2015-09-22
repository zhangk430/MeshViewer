#version 330 core

uniform sampler2D tex; 
uniform vec3 textColor; 

in vec2 vertUV; 

layout(location = 0) out vec4 fragColor; 
 
void main() 
{ 
  vec4 text = texture(tex, vertUV.st); 
  fragColor.rgb = textColor.rgb;
  fragColor.a = text.a;
}