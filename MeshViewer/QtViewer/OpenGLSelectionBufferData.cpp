#include "OpenGLSelectionBufferData.h"

typedef struct{
	vec4 position;
	vec4 color;
}selected_vertex;

void OpenGLSelectionBufferData::loadObjBufferData() {
	if (!theModelView.size())
		return;
	if (m_obj->mode != SelectedObject::NONE) {
		clearObjBufferData();
		glGenVertexArrays(1, &vao);
		glBindVertexArray(vao);
	}
	if (m_obj->mode == SelectedObject::Vertex_Mode) {
		int start_id = 1, numVertices = 0;
		for (size_t i = 0; i < theModelView.size(); i++) 
			numVertices += theModelView[i]->theMesh->numVertices();
		selected_vertex *vertex_data = new selected_vertex[numVertices];
		unsigned int *vertex_indices = new unsigned int[numVertices];
		for (size_t i = 0; i < theModelView.size(); i++) {
			if (!showMesh[i])
				continue;
			SimMesh *theMesh = theModelView[i]->theMesh;
			for (size_t j = 0; j < theMesh->numVertices(); j++) {
				int id = start_id + j;
				vertex_data[id - 1].position = vec4(theMesh->indVertex(j)->p[0], theMesh->indVertex(j)->p[1], theMesh->indVertex(j)->p[2], 1.0f);
				int r = (id & 0x000000FF) >> 0;
				int g = (id & 0x0000FF00) >> 8;
				int b = (id & 0x00FF0000) >> 16;
				vertex_data[id - 1].color = vec4(r / 255.0f, g / 255.0f, b / 255.0f, 1.0f);
				vertex_indices[id - 1] = id - 1;
			}
			start_id += theMesh->numVertices();
		}
		glGenBuffers(1, &vbo);
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBufferData(GL_ARRAY_BUFFER, numVertices * sizeof(selected_vertex), vertex_data, GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(selected_vertex), BUFFER_OFFSET(0));
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(selected_vertex), BUFFER_OFFSET(sizeof(vec4)));
		glGenBuffers(1, &vbo_indices);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo_indices);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, numVertices * sizeof(unsigned int), vertex_indices, GL_STATIC_DRAW);
		delete [] vertex_indices;
		delete [] vertex_data;
		mode = GL_POINTS;
		bufferSize = numVertices;
	}
	else if (m_obj->mode == SelectedObject::Edge_Mode) {
		int start_id = 1, numEdges = 0;
		for (size_t i = 0; i < theModelView.size(); i++) 
			numEdges += theModelView[i]->theMesh->numEdges();
		selected_vertex *vertex_data = new selected_vertex[numEdges * 2];
		unsigned int *vertex_indices = new unsigned int[numEdges * 2];
		for (size_t i = 0; i < theModelView.size(); i++) {
			if (!showMesh[i])
				continue;
			SimMesh *theMesh = theModelView[i]->theMesh;
			for (size_t j = 0; j < theMesh->numEdges(); j++) {
				SimEdge *e = theMesh->indEdge(j);
				SimVertex *v0 = e->v0;
				SimVertex *v1 = e->v1;
				int id = start_id + j;
				vertex_data[2 * id - 2].position = vec4(v0->p[0], v0->p[1], v0->p[2], 1.0f);
				vertex_data[2 * id - 1].position = vec4(v1->p[0], v1->p[1], v1->p[2], 1.0f);
				int r = (id & 0x000000FF) >> 0;
				int g = (id & 0x0000FF00) >> 8;
				int b = (id & 0x00FF0000) >> 16;
				vertex_data[2 * id - 2].color = vec4(r / 255.0f, g / 255.0f, b / 255.0f, 1.0f);
				vertex_data[2 * id - 1].color = vec4(r / 255.0f, g / 255.0f, b / 255.0f, 1.0f);
				vertex_indices[2 * id - 2] = 2 * id - 2;
				vertex_indices[2 * id - 1] = 2 * id - 1;
			}
			start_id += theMesh->numEdges();
		}
		glGenBuffers(1, &vbo);
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBufferData(GL_ARRAY_BUFFER, numEdges * 2 * sizeof(selected_vertex), vertex_data, GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(selected_vertex), BUFFER_OFFSET(0));
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(selected_vertex), BUFFER_OFFSET(sizeof(vec4)));
		glGenBuffers(1, &vbo_indices);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo_indices);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, numEdges * 2 * sizeof(unsigned int), vertex_indices, GL_STATIC_DRAW);
		delete [] vertex_data;
		delete [] vertex_indices;
		mode = GL_LINES;
		bufferSize = numEdges * 2;
	}
	else if (m_obj->mode == SelectedObject::Face_Mode) {
		int start_id = 1, numFaces = 0;
		for (size_t i = 0; i < theModelView.size(); i++) 
			numFaces += theModelView[i]->theMesh->numFaces();
		selected_vertex *vertex_data = new selected_vertex[numFaces * 3];
		unsigned int *vertex_indices = new unsigned[numFaces * 3];
		for (size_t i = 0; i < theModelView.size(); i++) {
			if (!showMesh[i])
				continue;
			SimMesh *theMesh = theModelView[i]->theMesh;
			for (size_t j = 0; j < theMesh->numFaces(); j++) {
				int id = start_id + j;
				int r = (id & 0x000000FF) >> 0;
				int g = (id & 0x0000FF00) >> 8;
				int b = (id & 0x00FF0000) >> 16;
				SimFace *f = theMesh->indFace(j);
				for (int k = 0; k < 3; k++) {
					vertex_data[3 * id + k - 3].position = vec4(f->ver[k]->p[0], f->ver[k]->p[1], f->ver[k]->p[2], 1.0);
					vertex_data[3 * id + k - 3].color = vec4(r / 255.0f, g / 255.0f, b / 255.0f, 1.0f);
					vertex_indices[3 * id + k - 3] = 3 * id + k - 3;
				}
			}
			start_id += theMesh->numFaces();
		}
		glGenBuffers(1, &vbo);
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBufferData(GL_ARRAY_BUFFER, numFaces * 3 * sizeof(selected_vertex), vertex_data, GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(selected_vertex), BUFFER_OFFSET(0));
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(selected_vertex), BUFFER_OFFSET(sizeof(vec4)));
		glGenBuffers(1, &vbo_indices);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo_indices);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, numFaces * 3 * sizeof(unsigned int), vertex_indices, GL_STATIC_DRAW);
		delete [] vertex_data;
		delete [] vertex_indices;
		mode = GL_TRIANGLES;
		bufferSize = numFaces * 3;
	}
}

void OpenGLSelectionBufferData::loadSelectedObjBufferData() {
	if (!theModelView.size())
		return;
	clearSelectedBufferData();
	if (m_obj->selectedVertex.size() || m_obj->selectedEdge.size() || m_obj->selectedFace.size()) {
		glGenVertexArrays(1, &vao_sobj);
		glBindVertexArray(vao_sobj);
	}
	if (m_obj->selectedVertex.size()) {
		std::vector<SimVertex *> orderedVertex;
		m_obj->orderedSeletedVertex(orderedVertex);
		selected_vertex *vertex_data = new selected_vertex[orderedVertex.size()];
		unsigned int *vertex_indices = new unsigned int[orderedVertex.size()];
		for (size_t i = 0; i < orderedVertex.size(); i++) {
			vertex_data[i].position = vec4(orderedVertex[i]->p[0], orderedVertex[i]->p[1], orderedVertex[i]->p[2], 1.0);
			vertex_data[i].color = vec4(0.0f, 1.0f, 1.0f, 1.0f);
			vertex_indices[i] = i;
		}
		glGenBuffers(1, &vbo_sobj);
		glBindBuffer(GL_ARRAY_BUFFER, vbo_sobj);
		glBufferData(GL_ARRAY_BUFFER, orderedVertex.size() * sizeof(selected_vertex), vertex_data, GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(selected_vertex), BUFFER_OFFSET(0));
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(selected_vertex), BUFFER_OFFSET(sizeof(vec4)));
		glGenBuffers(1, &vbo_sobj_indices);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo_sobj_indices);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, orderedVertex.size() * sizeof(unsigned int), vertex_indices, GL_STATIC_DRAW);
		delete [] vertex_data;
		delete [] vertex_indices;
		sobj_bufferSize = orderedVertex.size();
		sobj_mode = GL_POINTS;
	}
	else if (m_obj->selectedEdge.size()) {
		std::vector<SimEdge *> orderedEdges;
		m_obj->orderedSeletedEdge(orderedEdges);
		selected_vertex *vertex_data = new selected_vertex[orderedEdges.size() * 2];
		unsigned int *vertex_indices = new unsigned int[orderedEdges.size() * 2];
		for (size_t i = 0; i < orderedEdges.size(); i++) {
			SimEdge *e = orderedEdges[i];
			vertex_data[2 * i].position = vec4(e->v0->p[0], e->v0->p[1], e->v0->p[2], 1.0);
			vertex_data[2 * i + 1].position = vec4(e->v1->p[0], e->v1->p[1], e->v1->p[2], 1.0);
			vertex_data[2 * i].color = vec4(0.0f, 1.0f, 1.0f, 1.0f);
			vertex_data[2 * i + 1].color = vec4(0.0f, 1.0f, 1.0f, 1.0f);
			vertex_indices[2 * i] = 2 * i;
			vertex_indices[2 * i + 1] = 2 * i + 1;
		}
		glGenBuffers(1, &vbo_sobj);
		glBindBuffer(GL_ARRAY_BUFFER, vbo_sobj);
		glBufferData(GL_ARRAY_BUFFER, orderedEdges.size() * 2 * sizeof(selected_vertex), vertex_data, GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(selected_vertex), BUFFER_OFFSET(0));
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(selected_vertex), BUFFER_OFFSET(sizeof(vec4)));
		glGenBuffers(1, &vbo_sobj_indices);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo_sobj_indices);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, orderedEdges.size() * 2 * sizeof(unsigned int), vertex_indices, GL_STATIC_DRAW);
		delete [] vertex_data;
		delete [] vertex_indices;
		sobj_bufferSize = orderedEdges.size() * 2;
		sobj_mode = GL_LINES;
	}
	else if (m_obj->selectedFace.size()) {
		std::vector<SimFace *> orderedFaces;
		m_obj->orderedSeletedFace(orderedFaces);
		selected_vertex *vertex_data = new selected_vertex[orderedFaces.size() * 3];
		unsigned int *vertex_indices = new unsigned int[orderedFaces.size() * 3];
		for (size_t i = 0; i < orderedFaces.size(); i++) {
			SimFace *f = orderedFaces[i];
			for (int j = 0; j < 3; j++) {
				vertex_data[3 * i + j].position = vec4(f->ver[j]->p[0], f->ver[j]->p[1], f->ver[j]->p[2], 1.0);
				vertex_data[3 * i + j].color = vec4(0.0f, 1.0f, 1.0f, 1.0f);
				vertex_indices[3 * i + j] = 3 * i + j;
			}
		}
		glGenBuffers(1, &vbo_sobj);
		glBindBuffer(GL_ARRAY_BUFFER, vbo_sobj);
		glBufferData(GL_ARRAY_BUFFER, orderedFaces.size() * 3 * sizeof(selected_vertex), vertex_data, GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(selected_vertex), BUFFER_OFFSET(0));
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(selected_vertex), BUFFER_OFFSET(sizeof(vec4)));
		glGenBuffers(1, &vbo_sobj_indices);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo_sobj_indices);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, orderedFaces.size() * 3 * sizeof(unsigned int), vertex_indices, GL_STATIC_DRAW);
		delete [] vertex_data;
		delete [] vertex_indices;
		sobj_bufferSize = orderedFaces.size() * 3;
		sobj_mode = GL_TRIANGLES;
	}
}
