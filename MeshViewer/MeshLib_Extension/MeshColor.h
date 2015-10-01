#ifndef MESHCOLOR_H
#define MESHCOLOR_H


#include "../SimMeshLib/SimMesh.h"


class MeshColor
{
public:
	MeshColor(SimMesh * inmesh) : theMesh(inmesh){}
	SimMesh * theMesh;
	std::vector<unsigned char> r, g, b, a;
	void clear()
	{
		r.clear();
		g.clear();
		b.clear();
		a.clear();
	}
};

#endif