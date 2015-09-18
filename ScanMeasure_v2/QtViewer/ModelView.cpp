#include "ModelView.h"

void ModelView::LoadMesh(SimMesh * inMesh) {
	theMesh=inMesh;
	ComputCentroidAndBox();
	
	if (!theMesh->numFaces()) {
		return ;
	}
	if (!theNormals)
		theNormals=new MeshNormal(theMesh);
	theNormals->ComputeNormals();
}

void ModelView::ComputCentroidAndBox() {

	center[0] = center[1] = center[2] = 0;
	boxMin[0] = boxMin[1] = boxMin[2] = 1e5;
	boxMax[0] = boxMax[1] = boxMax[2] = -1e5;
	for (int i = 0; i < theMesh->numVertices(); i++)
	{
		SimVertex * v = theMesh->indVertex(i);
		Point & pt = v->p;
		for (int j = 0; j < 3; ++j) {
			float value = (float) pt[j];
			center[j] += value;
			if (boxMax[j] < value)
				boxMax[j] = value;
			if (boxMin[j] > value)
				boxMin[j] = value;
		}
	}

	boxAxisLen = (float)sqrt((boxMax[2] - boxMin[2]) * (boxMax[2] - boxMin[2]) + 
		(boxMax[1] - boxMin[1]) * (boxMax[1] - boxMin[1]) + 
		(boxMax[0] - boxMin[0]) * (boxMax[0]-boxMin[0]));

	center[0] /= theMesh->numVertices();
	center[1] /= theMesh->numVertices();
	center[2] /= theMesh->numVertices();

}

void ModelView::computeTangentSpace() {
	if (!theNormals || !theTexture)
		return;
	if (theTexture->mode == MeshTexture::VERTEX_MODE) {
		tangents.resize(theMesh->numVertices());
	}
	for (int i = 0; i < theMesh->numFaces(); i++) {
		SimFace *f = theMesh->indFace(i);
		for (int j = 0; j < 3; j++) {
			SimVertex *v0 = f->ver[j];
			SimVertex *v1 = f->ver[(j + 1) % 3];
			SimVertex *v2 = f->ver[(j + 2) % 3];

			std::pair<float, float> uv0 = std::make_pair(theTexture->getU(v0, f), theTexture->getV(v0, f));
			std::pair<float, float> uv1 = std::make_pair(theTexture->getU(v1, f), theTexture->getV(v1, f));
			std::pair<float, float> uv2 = std::make_pair(theTexture->getU(v2, f), theTexture->getV(v2, f));
			Point deltaPos1 = v1->p - v0->p;
			Point deltaPos2 = v2->p - v0->p;

			std::pair<float, float> deltaUV1 = std::make_pair(uv1.first - uv0.first, uv1.second - uv0.second);
			std::pair<float, float> deltaUV2 = std::make_pair(uv2.first - uv0.first, uv2.second - uv0.second);

			float r = 1.0f / (deltaUV1.first * deltaUV2.second - deltaUV1.second * deltaUV2.first);
			Point tangent = (deltaPos1 * deltaUV2.second - deltaPos2 * deltaUV1.second) * r;
			Point n = theNormals->vNormals[v0->idx];
			tangent = tangent - n * (n * tangent);
			tangent /= tangent.norm();
			Point bitangent = tangent ^ n;
			if (theTexture->mode == MeshTexture::VERTEX_MODE) {
				tangents[v0->idx] = tangent;
			}
			else {
				tangents.push_back(tangent);
			}
		}
	}
}