#include "Geodesics.h"
#include "geodesic_algorithm_exact.h"
#include "geodesic_algorithm_dijkstra.h"
#include <sstream>

//#include "geodesic_algorithm_subdivision.h"


geodesics::geodesics(SimMesh *mesh)
{
	cmesh = mesh;
	algorithms = NULL;
	for (unsigned i = 0; i < mesh->numVertices(); ++i)
	{
		SimVertex *v = mesh->indVertex(i);
		m_verts.push_back(v);
	}
	for (unsigned i = 0; i < mesh->numFaces(); ++i)
	{
		SimFace *f = mesh->indFace(i);
		m_faces.push_back(f);
	}
	write();
	std::vector<double> points;	
	std::vector<unsigned> faces;
	geodesic::read_mesh_from_file("temp.txt",points,faces); 


	tMesh = new geodesic::Mesh;
	tMesh->initialize_mesh_data(points, faces);		//create internal mesh data structure including edges
	system("del temp.txt");
}

void geodesics::write()
{
	FILE * fp = fopen("temp.txt","w");
	fprintf(fp, "%d %d\n", cmesh->numVertices(), cmesh->numFaces());

	for (std::vector<SimVertex *>::iterator vit = m_verts.begin(); vit != m_verts.end(); ++vit)
	{
		SimVertex * ver = *vit;
		fprintf(fp, "%f %f %f\n", ver->p[0], ver->p[1], ver->p[2]);
	}
	for (std::vector<SimFace *>::iterator fit = m_faces.begin(); fit != m_faces.end(); ++fit)
	{
		SimFace * f = *fit;
		fprintf(fp, "%d %d %d\n", f->ver[0]->idx, f->ver[1]->idx, f->ver[2]->idx);
	}
	fclose(fp);
}

void geodesics::setSource(std::vector<SimVertex *> srcList)
{
	sources.clear();
	for (unsigned i = 0; i < srcList.size(); i++)
	{
		int ind = srcList[i]->idx;
		sources.push_back(geodesic::SurfacePoint(&tMesh->vertices()[ind]));
	}
}
void geodesics::setSource(std::vector<SimFace *> srcList)
{
	sources.clear();
	for (int i = 0; i < srcList.size(); i++)
	{
		int ind = srcList[i]->idx;
		sources.push_back(geodesic::SurfacePoint(&tMesh->faces()[ind]));
	}
}

double geodesics::Path(SimVertex *trg, std::vector<geodesic::SurfacePoint> &path)
{
	if (!algorithms)
		return -1;
	geodesic::SurfacePoint target = geodesic::SurfacePoint(&tMesh->vertices()[trg->idx]);
	algorithms->trace_back(target, path);
	return geodesic::length(path);
}

void geodesics::computeGeodesics_Dijkstra()
{
	algorithms = new geodesic::GeodesicAlgorithmDijkstra(tMesh);
	algorithms->propagate(sources);
	agd = 0;
	for (int i = 0; i < m_verts.size(); i++)
	{
		geodesic::SurfacePoint target = geodesic::SurfacePoint(&tMesh->vertices()[m_verts[i]->idx]);
		algorithms->best_source(target, distance[m_verts[i]]);
		agd += distance[m_verts[i]];
	}
	agd /= m_verts.size()-1;
}
void geodesics::computeGeodesics_Exact()
{
	algorithms = new geodesic::GeodesicAlgorithmExact(tMesh);
	algorithms->propagate(sources);
	agd = 0;
	for (int i = 0; i < m_verts.size(); i++)
	{
		geodesic::SurfacePoint target = geodesic::SurfacePoint(&tMesh->vertices()[m_verts[i]->idx]);
		algorithms->best_source(target, distance[m_verts[i]]);
		agd += distance[m_verts[i]];
	}
	agd /= m_verts.size()-1;
}


double geodesics::Distance(SimVertex *v)
{
	return distance[v];
}

SimVertex * geodesics::convert2Vertex(geodesic::SurfacePoint sp)
{
	if (sp.type() != geodesic::VERTEX)
		return NULL;
	geodesic::vertex_pointer vp = (geodesic::vertex_pointer)sp.base_element();
	return m_verts[vp->id()];
}

SimEdge * geodesics::convert2Edge(geodesic::SurfacePoint sp)
{
	if (sp.type() != geodesic::EDGE)
		return NULL;
	geodesic::edge_pointer ep = (geodesic::edge_pointer)sp.base_element();
	geodesic::vertex_pointer vp0 = ep->adjacent_vertices()[0];
	geodesic::vertex_pointer vp1 = ep->adjacent_vertices()[1];
	int id0 = m_verts[vp0->id()]->idx;
	int id1 = m_verts[vp1->id()]->idx;
	return cmesh->idEdge(id0, id1);
}