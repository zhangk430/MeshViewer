#ifndef PICKING_VIEWER_H
#define PICKING_VIEWER_H

#include <QtViewer.h>
#include "OpenGLSelectionBufferData.h"
#include "RenderText.h"


class PickingViewer : public QtViewer
{
	Q_OBJECT
public:


	PickingViewer(QWidget * obj = 0) : QtViewer(obj), selectedBufferData(NULL){
		selectionShaderProgram.init("vSelectionShader.glsl", "fSelectionShader.glsl", "fragColor");
		fontShaderProgram = InitShader("vFontShader.glsl", "fFontShader.glsl", "fragColor");
	}

	virtual void setModelView(ModelView * modelView);

	virtual void paintGL();
	virtual void mousePressEvent(QMouseEvent *);

	void select();
	void drawSelectObject();
	void drawSelectedObject();
	void drawText(float x, float y, const std::string &s, vec3 color);

	void clearSelection();
	virtual void clear();
	

protected:

	SelectedObject m_selectObject;
	OpenGLSelectionBufferData *selectedBufferData;
	RenderText font;

	Shader selectionShaderProgram;
	GLuint fontShaderProgram;

	private slots:
		void turnOnSelectionFace();
		void turnOnSelectionEdge();
		void turnOnSelectionVertex();
		void turnOffSelection();


};


#endif