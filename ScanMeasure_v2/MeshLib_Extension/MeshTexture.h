#ifndef MESHTEXTURE_H
#define MESHTEXTURE_H

#include "../SimMeshLib/SimMesh.h"
#include <string>

class MeshTexture
{
public:	
	enum TextureMode
	{
		VERTEX_MODE,
		CORNER_MODE,
	};
	MeshTexture(SimMesh * inMesh):theMesh(inMesh), mode(VERTEX_MODE){}
	SimMesh * theMesh;
	std::vector<float> u, v;
	std::string texture_filename;
	TextureMode mode;

	float getU(SimVertex * ver, SimFace * f = NULL){
		if (mode == CORNER_MODE && f)
		{
			for (int i = 0; i < 3; i++)
			{
				if (f->ver[i]->idx == ver->idx)
					return u[3 * f->idx + i];
			}
			return -1;
		}
		else
			return u[ver->idx];
	}
	float getV(SimVertex * ver, SimFace * f = NULL)
	{
		if (mode == CORNER_MODE && f)
		{
			for (int i = 0; i < 3; i++)
			{
				if (f->ver[i]->idx == ver->idx)
					return v[3 * f->idx + i];
			}
			return -1;
		}
		else
			return v[ver->idx];
	}

};


#endif