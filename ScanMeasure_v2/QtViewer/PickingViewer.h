#ifndef PICKING_VIEWER_H
#define PICKING_VIEWER_H

#include <QtViewer.h>
#include <SelectedObject.h>


class PickingViewer : public QtViewer
{

#define SELECT_BUFSIZE 1024

	Q_OBJECT
public:


	PickingViewer(QWidget * obj = 0) : QtViewer(obj){}


	virtual void paintGL();
	virtual void mousePressEvent(QMouseEvent *);


	virtual void Render_Mesh();
	virtual void Render_Mesh_Edge();

	void select();
	void drawSelectObject();
	void ProcessHits(GLint hits, GLuint buffer[]);


	void loadSelectedVertices(const char filename[]);
	void loadSelectedVerticesByPosition(const char filename[]);
	void saveSelectedVertices(const char filename[]);
	void clearSelection();
	virtual void clear();
	

protected:

	SelectedObject m_selectObject;

	int findClosestPoints(Point p);



	private slots:
		void turnOnSelectionFace();
		void turnOnSelectionEdge();
		void turnOnSelectionVertex();
		void turnOffSelection();


};


#endif