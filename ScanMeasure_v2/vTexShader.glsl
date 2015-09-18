#version 330 core

uniform mat4 ModelViewMatrix;
uniform mat4 ProjectionMatrix;
uniform mat4 depthMatrix;
uniform vec4 lightPosition;

layout(location = 0) in vec4 position;
layout(location = 2) in vec3 normal;
layout(location = 3) in vec2 vTexCoord;
layout(location = 4) in vec3 tangent;

out vec3 cameraPosition_modelspace;
out vec3 lightPosition_modelspace;
out vec2 fTexCoord;
out vec4 shadowCoord;

void main (void)
{
	mat3 normalMatrix = transpose(inverse(mat3(ModelViewMatrix)));
	vec3 normal_cameraspace = normalMatrix * normal;
	vec3 tangent_cameraspace = normalMatrix * tangent;
	mat3 TBN = transpose(mat3(
		tangent_cameraspace,
		cross(normal_cameraspace, tangent_cameraspace),
		normal_cameraspace	
	));
	vec4 vertexPosition_modelspace = ModelViewMatrix * position;
	cameraPosition_modelspace = TBN *vec3(-vertexPosition_modelspace);
	lightPosition_modelspace = TBN * vec3(lightPosition - vertexPosition_modelspace);
	fTexCoord = vTexCoord;
	shadowCoord = depthMatrix * position;
    gl_Position = ProjectionMatrix * ModelViewMatrix * position;
}