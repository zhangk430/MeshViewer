#ifndef QTVIEWER_H
#define QTVIEWER_H

#include "Camera.h"
#include "OpenGLBufferData.h"
#include "Lighting.h"
#include "trackball.h"
#include "../GL_Extension/Shader.h"
#include <QOpenGLWidget>
#include <QtOpenGL/qgl.h>  // for QGLWidget
#include <QtGui/QMouseEvent>


class QtViewer: public QGLWidget{

public:

	QtViewer(QWidget * obj =NULL):QGLWidget(obj), theModelView(NULL), showWireFrame(false), floor(NULL),
	frameBufferName(0), depthTexture(0)
	{
		windowWidth = width();
		windowHeight = height();
		makeCurrent();
	}

	virtual ~QtViewer()
	{
		clear();
	}

	virtual void paintGL();
	virtual void initializeGL();
	virtual void resizeGL(int w, int h);

	virtual void mousePressEvent(QMouseEvent *);
	virtual void mouseMoveEvent(QMouseEvent *);
	virtual void mouseReleaseEvent(QMouseEvent *);

	virtual void setModelView(ModelView * modelView);
	virtual void addModelView(ModelView * modelView);

	int locateVertex(SimVertex *v);

	void ShowWireFrame(bool ifshow);
	virtual void setShowMesh(int idx, bool ifshow);
	virtual void clear();

	ModelView * getModelView(unsigned int idx){     return idx >= theModelView.size() ? NULL : theModelView[idx];    }
	int getModelViewSize() {   return theModelView.size();   }

	void refresh();

protected:

	void drawFloor();
	void createPlane(Point center, Point n, double len);
	bool generateDepthBuffer();

	void SetCameraFromModelView();
	mat4 getModelViewMatrix();
	mat4 getProjectionMatrix();
	mat4 getDepthMatrix();

	virtual void Render_Mesh(int idx);
	virtual void Render_Mesh_Edge(int idx);

	QPointF pixelPosToViewPos(const QPointF& p);
	QPointF viewPosToPixelPos(const QPointF& p);
	QPointF viewPosToPixelPos(Point& p);

	
protected:

	Camera theCamera;
	Light light0;
	std::vector<ModelView *> theModelView;
	std::vector<OpenglBufferData *> m_bufferData;
	OpenglBufferData *floor;

	Shader colorShaderProgram, textureShadedrProgram, shadowShaderProgram;
	GLuint frameBufferName, depthTexture, texID;
	
	double mouseX, mouseY;
	TrackBall m_Trackball;
	int windowWidth, windowHeight;

	bool showWireFrame;
	std::vector<bool> showMesh;






};







#endif