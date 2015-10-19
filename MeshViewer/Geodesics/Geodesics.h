#ifndef GEODESICS_H
#define GEODESICS_H

#include "../SimMeshLib/SimMesh.h"
#include "geodesic_algorithm_base.h"

class geodesics
{
public:
	geodesics(SimMesh *mesh);
	void setSource(std::vector<SimVertex *> srcList);
	void setSource(std::vector<SimFace *> srcList);
	void computeGeodesics_Dijkstra();
	void computeGeodesics_Exact();
	double Path(SimVertex *trg, std::vector<geodesic::SurfacePoint> &path);
	double Distance(SimVertex *v);

	SimVertex * convert2Vertex(geodesic::SurfacePoint sp);
	SimEdge * convert2Edge(geodesic::SurfacePoint sp);

	double agd;
protected:
	void write();
private:
	stdext::hash_map<SimVertex *, double> distance;
	std::vector<SimVertex *> m_verts;
	std::vector<SimFace *> m_faces;
	SimMesh *cmesh;
	geodesic::Mesh *tMesh;
	std::vector<geodesic::SurfacePoint> sources;
	geodesic::GeodesicAlgorithmBase * algorithms;

};



#endif