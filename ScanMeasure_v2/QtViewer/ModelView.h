#ifndef MODELVIEW_H
#define MODELVIEW_H

#include <SimMesh.h>
#include <MeshNormal.h>
#include <MeshColor.h>
#include <MeshTexture.h>

class ModelView{
public:
	~ModelView(){ 
		delete theMesh;
		delete theNormals;
		delete theColor;
		delete theTexture;
	}
	ModelView() : color(1.0f, (float)170/255, (float)127/255) {
		theMesh = NULL;
		theNormals = NULL;
		theColor = NULL;
		theTexture = NULL;
	}

	void LoadMesh(SimMesh * inMesh);

	Point & VertexNormal(SimVertex * v){
		return theNormals->vNormals[v->idx];	
	}

	
	void ComputCentroidAndBox();


public:
	SimMesh * theMesh;
	MeshNormal * theNormals;
	MeshColor * theColor;
	MeshTexture * theTexture;
	Point center, color;
	Point boxMin, boxMax;
	float boxAxisLen;
};

#endif