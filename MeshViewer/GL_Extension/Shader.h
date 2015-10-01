#ifndef SHADER_H
#define SHADER_H

#include "Angel.h"

struct Shader
{
	GLuint projectionMatrixUniform;
	GLuint modelViewMatrixUniform;
	GLuint depthMatrixUniform;
	GLuint lightPositionUniform;
	GLuint diffuseLight, ambientLight, specularLight;
	GLuint texture, shadowMap;
	GLuint shaderProgram;

	void init(char *vShaderSource, char *fShaderSource, char *outputAttribName) {
		shaderProgram = InitShader(vShaderSource, fShaderSource, outputAttribName);
		modelViewMatrixUniform = glGetUniformLocation(shaderProgram, "ModelViewMatrix");
		projectionMatrixUniform = glGetUniformLocation(shaderProgram, "ProjectionMatrix");
		depthMatrixUniform = glGetUniformLocation(shaderProgram, "depthMatrix");
		lightPositionUniform = glGetUniformLocation(shaderProgram, "lightPosition");
		ambientLight = glGetUniformLocation(shaderProgram, "ambientLight");
		diffuseLight = glGetUniformLocation(shaderProgram, "diffuseLight");
		specularLight = glGetUniformLocation(shaderProgram, "specularLight");
		texture = glGetUniformLocation(shaderProgram, "ambientTexture");
		shadowMap = glGetUniformLocation(shaderProgram, "shadowMap");
	}
};

#endif