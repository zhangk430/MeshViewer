#ifndef PICKING_VIEWER_H
#define PICKING_VIEWER_H

#include "QtViewer.h"
#include "OpenGLSelectionBufferData.h"
#include "RenderText.h"


class PickingViewer : public QtViewer
{
	Q_OBJECT
public:


	PickingViewer(QWidget * obj = 0) : QtViewer(obj), selectedBufferData(NULL){
	}

	void setModelView(ModelView * modelView);
	void addModelView(ModelView * modelView);

	void initializeGL();
	void paintGL();
	void mousePressEvent(QMouseEvent *);
	void mouseReleaseEvent(QMouseEvent *);

	void select();
	void drawSelectObject();
	void drawSelectedObject();
	void drawText(float x, float y, const std::string &s, vec3 color);

	void clearSelection();
	void clear();

	const SelectedObject & getSelectedObjecct(){  return m_selectObject;   }
	

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

signals:
		void picked();


};


#endif