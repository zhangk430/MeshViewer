#ifndef OPENGL_BUFFER_DATA_H
#define OPENGL_BUFFER_DATA_H

#include "ModelView.h"
#include "../GL_Extension/Angel.h"

class OpenglBufferData
{
public:
	ModelView *theModelView;
	GLuint vao, vao_wireFrame;
	GLuint vbo;
	GLuint vbo_edge_indices;
	GLuint vbo_face_indices;
	GLuint texture, normalTexture;
	OpenglBufferData(ModelView *modelView) : theModelView(modelView), vao(0), vao_wireFrame(0), vbo(0), vbo_edge_indices(0), vbo_face_indices(0), 
		texture(0) {

	}
	~OpenglBufferData() {
		if (vbo) glDeleteBuffers(1, &vbo);
		if (vbo_edge_indices) glDeleteBuffers(1, &vbo_edge_indices);
		if (vbo_face_indices) glDeleteBuffers(1, &vbo_face_indices);
		if (vao) glDeleteVertexArrays(1, &vao);
		if (vao_wireFrame) glDeleteVertexArrays(1, &vao_wireFrame);
		if (texture) glDeleteBuffers(1, &texture);
	}

	void loadBufferData();

private:

};

#endif