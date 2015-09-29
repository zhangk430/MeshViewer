#ifndef SELECTED_OBJECT_H
#define SELECTED_OBJECT_H

#include "../SimMeshLib/SimMesh.h"

class SelectedObject
{
public:
	enum SelectMode
	{
		NONE,
		Vertex_Mode,
		Edge_Mode,
		Face_Mode,
	};
	SelectMode mode;

	stdext::hash_map<SimVertex *, int> selectedVertex;
	stdext::hash_map<SimEdge *, int> selectedEdge;
	stdext::hash_map<SimFace *, int> selectedFace;
	typedef stdext::hash_map<SimVertex *, int>::iterator vertexIterator;
	typedef stdext::hash_map<SimEdge *, int>::iterator edgeIterator;
	typedef stdext::hash_map<SimFace *, int>::iterator faceIterator;


	void updateSelectionOrder();
	void orderedSeletedVertex(std::vector<SimVertex *>& orderedVertex);
	void orderedSeletedEdge(std::vector<SimEdge *>& orderedEdge);
	void orderedSeletedFace(std::vector<SimFace *>& orderedFace);

	void clear();


};

#endif