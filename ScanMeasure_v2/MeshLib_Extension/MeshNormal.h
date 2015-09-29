#ifndef MESHNORMAL_H
#define MESHNORMAL_H

#include "../SimMeshLib/SimMesh.h"
#include <vector>
#include <assert.h>
#include <iostream>


class MeshNormal{

public:
	MeshNormal(SimMesh * inMesh){
		assert(inMesh);
		theMesh=inMesh;
	}

	~MeshNormal(){};

	void ComputeNormals(){                  //modified by Kang 

		if (faceTrgAngles.size() != theMesh->numFaces())
		{
			faceTrgAngles.resize(theMesh->numFaces());
			//To compute corner angles
			for (size_t i = 0; i < theMesh->numFaces(); i++){
				SimFace * f = theMesh->indFace(i);
				float len[3], len2[3];
				for (int i=0;i<3;++i)
				{
					len2[i] = (f->ver[(i+1)%3]->p - f->ver[i]->p).norm2();
					len[i] = sqrt(len2[i]);
				}
				for (int i=0;i<3;++i)
				{
					int ip1=(i+1)%3;
					int ip2=(i+2)%3;
					if (len[i]<1e-8 || len[ip1]<1e-8)
						faceTrgAngles[f->idx].push_back((float)3.14159265359/2); //to handle the degenerate case
					else
						faceTrgAngles[f->idx].push_back((float)acos((len2[i]+len2[ip1]-len2[ip2])/(2*len[i]*len[ip1])));
				}
			}
		}
	

		if (fNormals.size() != theMesh->numFaces())
		{
			fNormals.resize(theMesh->numFaces());
			//To compute face normal
			int f_ind = 0;
			for (size_t i = 0; i < theMesh->numFaces(); i++){
				SimFace * f = theMesh->indFace(i);
				SimVertex * v1 = f->ver[0];
				SimVertex * v2 = f->ver[1];
				SimVertex * v3 = f->ver[2];
				Point nface = (v2->p-v1->p)^(v3->p-v2->p);			
				nface /= nface.norm();
				fNormals[f->idx]=nface;
			}
		}
		if (vNormals.size() != theMesh->numVertices())
		{
			vNormals.resize(theMesh->numVertices());
			for (size_t i = 0; i < theMesh->numVertices(); i++)
			{
				Point sumVNorm(0,0,0);
				float sumWeight = 0;
				SimVertex * v = theMesh->indVertex(i);
				int FaceCount = 0;		
				std::vector<SimFace *> orf = theMesh->getOneRingFace(v);
				for (size_t i = 0; i < orf.size(); i++)
				{			
					SimFace * f = orf[i];
					for (int j = 0; j < 3; j++)
					{
						if (v->idx == f->ver[j]->idx)
						{
							sumVNorm += fNormals[f->idx]*faceTrgAngles[f->idx][j];
							sumWeight += faceTrgAngles[f->idx][j];
							break;
						}
					}
					FaceCount++;
					
				}	
				vNormals[v->idx] = sumVNorm/sumWeight;
			}
		}
		

	}

	SimMesh * theMesh;
	std::vector<Point> vNormals;
	std::vector<std::vector<float>> faceTrgAngles;
	std::vector<Point> fNormals;
};

#endif