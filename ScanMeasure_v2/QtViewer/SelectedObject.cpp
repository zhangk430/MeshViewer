#include "SelectedObject.h"


void SelectedObject::updateSelectionOrder()
{
	switch( mode ) 
	{
	case Vertex_Mode:
		{
			std::vector<SimVertex *> orderedVertex;
			orderedSeletedVertex(orderedVertex);
			for (int i = 0; i < orderedVertex.size(); i++)
				selectedVertex[orderedVertex[i]] = i+1;
			break;
		}
		
	case Edge_Mode:
		{
			std::vector<SimEdge *> orderedEdge;
			orderedSeletedEdge(orderedEdge);
			for (int i = 0; i < orderedEdge.size(); i++)
				selectedEdge[orderedEdge[i]] = i+1;
			break;
		}
		
	case Face_Mode:
		{
			std::vector<SimFace *> orderedFace;
			orderedSeletedFace(orderedFace);
			for (int i = 0; i < orderedFace.size(); i++)
				selectedFace[orderedFace[i]] = i+1;
			break;
		}
		
	}
}

void SelectedObject::orderedSeletedVertex(std::vector<SimVertex *>& orderedVertex)
{
	orderedVertex.clear();
	std::vector<SimVertex *> t_orderedVertex;
	int max = 0;
	for (vertexIterator vit = selectedVertex.begin(); vit != selectedVertex.end(); ++vit)
	{
		std::pair<SimVertex *, int> vp = *vit;
		max = max > vp.second ? max : vp.second;
	}
	t_orderedVertex.resize(max, NULL);
	for (vertexIterator vit = selectedVertex.begin(); vit != selectedVertex.end(); ++vit)
	{
		std::pair<SimVertex *, int> vp = *vit;
		t_orderedVertex[vp.second - 1] = vp.first;
	}
	for (int i = 0; i < t_orderedVertex.size(); i++)
	{
		if (t_orderedVertex[i])
			orderedVertex.push_back(t_orderedVertex[i]);
	}
}

void SelectedObject::orderedSeletedEdge(std::vector<SimEdge *>& orderedEdge)
{
	orderedEdge.clear();
	std::vector<SimEdge*> t_orderedEdge;
	int max = 0;
	for (edgeIterator vit = selectedEdge.begin(); vit != selectedEdge.end(); ++vit)
	{
		std::pair<SimEdge *, int> vp = *vit;
		max = max > vp.second ? max : vp.second;
	}
	t_orderedEdge.resize(max, NULL);
	for (edgeIterator vit = selectedEdge.begin(); vit != selectedEdge.end(); ++vit)
	{
		std::pair<SimEdge *, int> vp = *vit;
		t_orderedEdge[vp.second - 1] = vp.first;
	}
	for (int i = 0; i < t_orderedEdge.size(); i++)
	{
		if (t_orderedEdge[i])
			orderedEdge.push_back(t_orderedEdge[i]);
	}
}

void SelectedObject::orderedSeletedFace(std::vector<SimFace *>& orderedFace)
{
	orderedFace.clear();
	std::vector<SimFace *> t_orderedFace;
	int max = 0;
	for (faceIterator vit = selectedFace.begin(); vit != selectedFace.end(); ++vit)
	{
		std::pair<SimFace *, int> vp = *vit;
		max = max > vp.second ? max : vp.second;
	}
	t_orderedFace.resize(max, NULL);
	for (faceIterator vit = selectedFace.begin(); vit != selectedFace.end(); ++vit)
	{
		std::pair<SimFace *, int> vp = *vit;
		t_orderedFace[vp.second - 1] = vp.first;
	}
	for (int i = 0; i < t_orderedFace.size(); i++)
	{
		if (t_orderedFace[i])
			orderedFace.push_back(t_orderedFace[i]);
	}
}

void SelectedObject::clear()
{
	selectedVertex.clear();
	selectedEdge.clear();
	selectedFace.clear();
	mode = NONE;
}