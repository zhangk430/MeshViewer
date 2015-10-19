#ifndef TRACING_VIEWER_H
#define TRACING_VIEWER_H


#include "PickingViewer.h"

class TracingViewer : public PickingViewer
{
public:
	TracingViewer(QWidget *obj = 0) : PickingViewer(obj){};
	~TracingViewer(){};

public:

	void traceShortestPath(bool isLoop = false);

};


#endif // !TRACING_VIEWER_H
