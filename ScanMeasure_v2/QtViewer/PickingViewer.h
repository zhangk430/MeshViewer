#ifndef PICKING_VIEWER_H
#define PICKING_VIEWER_H

#include <QtViewer.h>
#include "OpenGLSelectionBufferData.h"


class PickingViewer : public QtViewer
{

#define SELECT_BUFSIZE 1024

	Q_OBJECT
public:


	PickingViewer(QWidget * obj = 0) : QtViewer(obj), selectedBufferData(NULL){
		selectionShaderProgram.init("vSelectionShader.glsl", "fSelectionShader.glsl", "fragColor");
	}

	virtual void setModelView(ModelView * modelView);

	virtual void paintGL();
	virtual void mousePressEvent(QMouseEvent *);

	void select();
	void drawSelectObject();
	void drawSelectedObject();

	void clearSelection();
	virtual void clear();
	

protected:

	SelectedObject m_selectObject;
	OpenGLSelectionBufferData *selectedBufferData;

	Shader selectionShaderProgram;


	private slots:
		void turnOnSelectionFace();
		void turnOnSelectionEdge();
		void turnOnSelectionVertex();
		void turnOffSelection();


};


#endif