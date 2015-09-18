#ifndef MEASURE_VIEWER_H
#define MEASURE_VIEWER_H

#include "PickingViewer.h"

struct dist
{
	int fromIdx;
	int toIdx;
	float d;
};

class MeasureViewer : public PickingViewer
{
public:
	MeasureViewer(QWidget *obj = 0) : PickingViewer(obj){}

	void setModelView(ModelView * modelView);

	virtual void keyPressEvent(QKeyEvent *);


	void measureBetweenSelectedVertices();
	void saveMeasurement(const char *filename);
	virtual void clear();

	std::vector<dist> ms;


};


#endif