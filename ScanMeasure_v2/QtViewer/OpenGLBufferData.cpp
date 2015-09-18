#include "OpenGLBufferData.h"
#include <QtGui/QImage>
#include <QtOpenGL/qgl.h>

typedef struct{
	vec4 position;
	vec4 color;
	vec3 normal;
	vec2 texCoord;
	vec3 tangent;
}vertex;

void openglBufferData::loadBufferData() {
	if (!theModelView)
		return;
	SimMesh *theMesh = theModelView->theMesh;
	MeshColor *theColor = theModelView->theColor;
	MeshTexture *theTexture = theModelView->theTexture;
	MeshNormal *theNormals = theModelView->theNormals;
	vertex *data = NULL;
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);
	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	if (!theTexture) {
		data = new vertex[theMesh->numVertices()];
		for (int i = 0; i < theMesh->numVertices(); i++) {
			data[i].position = vec4(theMesh->indVertex(i)->p[0], theMesh->indVertex(i)->p[1], theMesh->indVertex(i)->p[2], 1.0);
			if (theModelView->theColor) {
				GLfloat a = 1.0f;
				if (i < theColor->a.size()) a = (GLfloat)theColor->a[i] / 255;
				data[i].color = vec4((GLfloat)theColor->r[i] / 255, (GLfloat)theColor->g[i] / 255, (GLfloat)theColor->b[i] / 255, a);
			}
			else
				data[i].color = vec4((GLfloat)theModelView->color[0], (GLfloat)theModelView->color[1], (GLfloat)theModelView->color[2], 1.0f);
		}
		if (theNormals) {
			for (int i = 0; i < theNormals->vNormals.size(); i++) {
				data[i].normal = vec3(theNormals->vNormals[i][0], theNormals->vNormals[i][1], theNormals->vNormals[i][2]);
			}
		}
		glBufferData(GL_ARRAY_BUFFER, theMesh->numVertices() * sizeof(vertex), data, GL_STATIC_DRAW);
	}
	else {
		// if texture exists, we need to duplicate the vertex on each triangle
		data = new vertex[theMesh->numFaces() * 3];
		for (int i = 0; i < theMesh->numFaces(); i++) {
			SimFace *f = theMesh->indFace(i);
			for (int j = 0; j < 3; j++) {
				SimVertex *v = f->ver[j];
				data[i * 3 + j].position = vec4(theMesh->indVertex(v->idx)->p[0], theMesh->indVertex(v->idx)->p[1], theMesh->indVertex(v->idx)->p[2], 1.0);
				if (theNormals) {
					data[i * 3 + j].normal = vec3(theNormals->vNormals[v->idx][0], theNormals->vNormals[v->idx][1], theNormals->vNormals[v->idx][2]);
					data[i * 3 + j].tangent = vec3(theModelView->tangents[i * 3 + j][0], theModelView->tangents[i * 3 + j][1], theModelView->tangents[i * 3 + j][2]);
				}
				data[i * 3 + j].texCoord = vec2(theTexture->getU(v, f), theTexture->getV(v, f));
			}
		}
		glBufferData(GL_ARRAY_BUFFER, 3 * theMesh->numFaces() * sizeof(vertex), data, GL_STATIC_DRAW);
	}

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(vertex), BUFFER_OFFSET(0));
	if (theTexture) {
		glEnableVertexAttribArray(3);
		glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, sizeof(vertex), BUFFER_OFFSET(2 * sizeof(vec4) + sizeof(vec3)));
		if (theNormals) {
			glEnableVertexAttribArray(4);
			glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(vertex), BUFFER_OFFSET(2 * sizeof(vec4) + sizeof(vec3) + sizeof(vec2)));
		}
	}
	else {
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(vertex), BUFFER_OFFSET(sizeof(vec4)));
	}
	if (theNormals) {
		glEnableVertexAttribArray(2);
		glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(vertex), BUFFER_OFFSET(2 * sizeof(vec4)));
	}
	unsigned int *face_indices = new unsigned int[3 * theMesh->numFaces()];
	for (int i = 0; i < theMesh->numFaces(); i++) {
		SimFace *f = theMesh->indFace(i);
		for (int j = 0; j < 3; j++) {
			if (!theTexture) 
				face_indices[3 * i + j] = f->ver[j]->idx;
			else 
				face_indices[3 * i + j] = 3 * i + j;
		}
	}
	glGenBuffers(1, &vbo_face_indices);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo_face_indices);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, 3 * theMesh->numFaces() * sizeof(unsigned int), face_indices, GL_STATIC_DRAW);
	glGenVertexArrays(1, &vao_wireFrame);
	glBindVertexArray(vao_wireFrame);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(vertex), BUFFER_OFFSET(0));
	unsigned int *edge_indices = new unsigned int[2 * theMesh->numEdges()];
	if (theTexture) {
		for (int i = 0; i < theMesh->numFaces(); i++) {
			SimFace *f = theMesh->indFace(i);
			for (int j = 0; j < 3; j++) {
				SimEdge * e = theMesh->idEdge(f->ver[j]->idx, f->ver[(j + 1) % 3]->idx);
				edge_indices[2 * e->idx + 0] = i * 3 + j;
				edge_indices[2 * e->idx + 1] = i * 3 + (j + 1) % 3;
			}
		}
	}
	else {
		for (int i = 0; i < theMesh->numEdges(); i++) {
			SimEdge * e = theMesh->indEdge(i);
			edge_indices[2 * i + 0] = e->v0->idx;
			edge_indices[2 * i + 1] = e->v1->idx;
		}
	}
	
	glGenBuffers(1, &vbo_edge_indices);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo_edge_indices);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, 2 * theMesh->numEdges() * sizeof(unsigned int), edge_indices, GL_STATIC_DRAW);
	if (theTexture) {
		glGenTextures(1, &texture);
		glBindTexture(GL_TEXTURE_2D, texture);
			//get the OpenGL-friendly image
		QImage GL_formatted_image = QGLWidget::convertToGLFormat(QImage(theModelView->theTexture->texture_filename.c_str()));
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
					//generate the texture
		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, GL_formatted_image.width(),
		GL_formatted_image.height(),
		0, GL_RGBA, GL_UNSIGNED_BYTE, GL_formatted_image.bits() );
		glBindTexture(GL_TEXTURE_2D, 0);
		QImage GL_formatted_image2 = QGLWidget::convertToGLFormat(QImage("normal.bmp"));
		glGenTextures(1, &normalTexture);
		glBindTexture(GL_TEXTURE_2D, normalTexture);
		//get the OpenGL-friendly image
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		//generate the texture
		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, GL_formatted_image2.width(),
			GL_formatted_image2.height(),
			0, GL_RGBA, GL_UNSIGNED_BYTE, GL_formatted_image2.bits() );
		glBindTexture(GL_TEXTURE_2D, 0);
		
	}

	delete []data;
	delete []edge_indices;
	delete []face_indices;
}