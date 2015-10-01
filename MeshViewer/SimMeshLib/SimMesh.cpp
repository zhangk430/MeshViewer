#include "SimMesh.h"
#include <iostream>

#pragma warning (disable : 4996)
#pragma warning (disable : 4018)

SimMesh::SimMesh() {;}

SimMesh::~SimMesh(){	clear();}

void SimMesh::clear(){
	for (std::vector<SimFace *>::iterator fiter = m_faces.begin(); fiter!=m_faces.end(); ++fiter)
	{
		SimFace * f = *fiter;
		delete f;
	}
	for (std::vector<SimEdge *>::iterator eiter = m_edges.begin(); eiter!=m_edges.end(); ++eiter)
	{
		SimEdge * e = *eiter;
		delete e;
	}
	for (std::vector<SimVertex *>::iterator viter = m_verts.begin(); viter!=m_verts.end(); ++viter)
	{
		SimVertex * v = *viter;
		delete v;
	}
	m_verts.clear();
	m_edges.clear();
	m_faces.clear();
	oneRingEdge.clear();
	//maxVid=-1;
	//maxFid = -1;
}

SimEdge * SimMesh::idEdge( int id0, int id1 )
{
	SimVertex * v0 = indVertex(id0);
	SimVertex * v1 = indVertex(id1);
	if (!v0 || !v1)
		return NULL;
	else
		return vertexEdge(v0, v1);
}

SimEdge * SimMesh::vertexEdge( SimVertex * v0, SimVertex * v1 )
{
	std::vector<std::pair<int,int>> neigh = oneRingEdge[v0->idx];
	for (int i = 0; i < neigh.size(); i++)
	{
		std::pair<int,int> ep = neigh[i];
		if (ep.first == v1->idx)
			return m_edges[ep.second];
	}
	return NULL;
}

std::vector<SimEdge *> SimMesh::getOneRingEdge(SimVertex * v)
{
	std::vector<SimEdge *> ore;
	for (int i = 0; i < oneRingEdge[v->idx].size(); i++)
	{
		std::pair<int,int> ep = oneRingEdge[v->idx][i];
		ore.push_back(m_edges[ep.second]);
	}
	return ore;
}

std::vector<SimFace *> SimMesh::getOneRingFace(SimVertex * v)
{
	std::vector<SimFace *> orf;
	for (int i = 0; i < oneRingFace[v->idx].size(); i++)
	{
		orf.push_back(m_faces[oneRingFace[v->idx][i]]);
	}
	return orf;
}

//create new geometric simplexes

SimVertex * SimMesh::createVertex()
{
	SimVertex * v = new SimVertex;	
	v->idx= m_verts.size();
	m_verts.push_back( v );
	return v;
}

SimEdge * SimMesh::createEdge(SimVertex * v0, SimVertex * v1)
{
	SimEdge * e = new SimEdge;
	e->v0 = v0;
	e->v1 = v1;
	e->idx = m_edges.size();
	m_edges.push_back(e);
	return e;
}

SimFace * SimMesh::createFace()
{
	SimFace * f = new SimFace();
	f->idx = m_faces.size();
	m_faces.push_back(f);
	return f;
}

SimFace * SimMesh::createFace( int v[3])
{
	SimVertex * ver[3];
	for(int i = 0; i < 3; i ++ )
	{
		ver[i] =  indVertex( v[i] );
		if (!ver[i])
		{
			std::cerr << "Error: invalid Vertex id: " << v[i] << " provided when creating Face "
				<< " !" << std::endl;
			return NULL;
		}
	}

	SimFace * f = createFace(ver);
	return f;
}

SimFace * SimMesh::createFace( SimVertex * ver[3])
{		
	SimFace * f = createFace();	
	for (int i = 0; i < 3; i++)
		f->ver[i] = ver[i];
	for (int i = 0; i < 3; i++)
	{
		SimEdge * e = idEdge(ver[i]->idx, ver[(i+1)%3]->idx);
		if (!e)
			e = idEdge(ver[(i+1)%3]->idx, ver[i]->idx);
		if (!e)
		{
			e = createEdge(ver[i], ver[(i+1)%3]);
			std::pair<int, int> ep0, ep1;
			ep0.first = e->v1->idx;
			ep0.second = e->idx;
			ep1.first = e->v0->idx;
			ep1.second = e->idx;
			oneRingEdge[e->v0->idx].push_back(ep0);
			oneRingEdge[e->v1->idx].push_back(ep1);
			boundary.push_back(true);
		}
		else
			boundary[e->idx] = false;
			
		oneRingFace[ver[i]->idx].push_back(f->idx);
	}
	return f;
}
