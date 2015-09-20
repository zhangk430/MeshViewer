#ifndef OPENGL_SELECTION_BUFFER_DATA
#define OPENGL_SELECTION_BUFFER_DATA

#include "ModelView.h"
#include "../GL_Extension/Angel.h"
#include "SelectedObject.h"

class OpenGLSelectionBufferData
{
public:
	OpenGLSelectionBufferData(std::vector<ModelView *> modelView, SelectedObject *obj) : vao(0), vao_sobj(0),
		vbo(0), vbo_indices(0), vbo_sobj(0), vbo_sobj_indices(0),
		bufferSize(0), sobj_bufferSize(0),
		theModelView(modelView), m_obj(obj){}
	~OpenGLSelectionBufferData() {
		clearObjBufferData();
		clearSelectedBufferData();
	}

	void loadObjBufferData();
	void loadSelectedObjBufferData();

	void clearObjBufferData() {
		if (vbo) glDeleteBuffers(1, &vbo);
		if (vbo_indices) glDeleteBuffers(1, &vbo_indices);
		if (vao) glDeleteVertexArrays(1, &vao);
		bufferSize = 0;
	}

	void clearSelectedBufferData() {
		if (vbo_sobj) glDeleteBuffers(1, &vbo_sobj);
		if (vbo_sobj_indices) glDeleteBuffers(1, &vbo_sobj_indices);
		if (vao_sobj) glDeleteVertexArrays(1, &vao_sobj);
		sobj_bufferSize = 0;
	}

	GLuint vao, vao_sobj;
	GLuint vbo, vbo_indices;
	GLuint vbo_sobj, vbo_sobj_indices;

	GLenum mode, sobj_mode;
	int bufferSize, sobj_bufferSize;

protected:
	
	std::vector<ModelView *> theModelView;
	SelectedObject *m_obj;


};


#endif