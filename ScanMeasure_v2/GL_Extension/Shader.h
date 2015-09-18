#ifndef SHADER_H
#define SHADER_H

#include "Angel.h"
#include <iostream>

struct Shader
{
	GLuint projectionMatrixUniform;
	GLuint modelViewMatrixUniform;
	GLuint depthMatrixUniform;
	GLuint lightPositionUniform, cameraPositionUniform;
	GLuint diffuseLight, ambientLight, specularLight;
	GLuint texture, shadowMap;
	GLuint shaderProgram;

	void init(char *vShaderSource, char *fShaderSource, char *outputAttribName) {
		shaderProgram = InitShader(vShaderSource, fShaderSource, outputAttribName);
		modelViewMatrixUniform = glGetUniformLocation(shaderProgram, "ModelViewMatrix");
		projectionMatrixUniform = glGetUniformLocation(shaderProgram, "ProjectionMatrix");
		depthMatrixUniform = glGetUniformLocation(shaderProgram, "depthMatrix");
		lightPositionUniform = glGetUniformLocation(shaderProgram, "lightPosition");
		cameraPositionUniform = glGetUniformLocation(shaderProgram, "cameraPosition");
		ambientLight = glGetUniformLocation(shaderProgram, "ambientLight");
		diffuseLight = glGetUniformLocation(shaderProgram, "diffuseLight");
		specularLight = glGetUniformLocation(shaderProgram, "specularLight");
		texture = glGetUniformLocation(shaderProgram, "ambientTexture");
		shadowMap = glGetUniformLocation(shaderProgram, "shadowMap");
	}
};

#endif