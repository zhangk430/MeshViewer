#ifndef CAMERA_H
#define CAMERA_H

#include <Point.h>

class Camera{
public:
	Camera(){ 
		//boxMin = Point(0,0,0);
		boxMax = Point(1,1,1);
		boxAxisLen = (boxMax-boxMin).norm();
		position=Point(0,0,5);
	}

	~Camera(){;}

public:
	Point position;
	Point center;
	Point translation;
	Point rotation;
	Point boxMin, boxMax;	
	double boxAxisLen;
};

#endif