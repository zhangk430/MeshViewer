#ifndef SIMMESH_H
#define SIMMESH_H

#include "SimVertex.h"
#include "SimFace.h"
#include "SimEdge.h"
#include <hash_map>

class SimMesh
{
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////								Methods										//////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
public:
	//(1) Constructor and Destructor
	SimMesh();
	//SimMesh::SimMesh(std::vector<Face*>& faces);
	~SimMesh();

	//(2) I/O
	size_t numVertices()	{return m_verts.size();}							//number of vertices
	size_t numEdges()      {return m_edges.size();}
	size_t numFaces()		{return m_faces.size();}							//number of faces
	void clear();


	//indexing elements
	SimEdge *				indEdge(unsigned int eInd){ return (eInd>=m_edges.size()?NULL:m_edges[eInd]);}
	SimFace *				indFace(unsigned int fInd) { return (fInd>=m_faces.size()?NULL:m_faces[fInd]);}
	SimVertex *			indVertex(unsigned int vInd) { return (vInd>=m_verts.size()?NULL:m_verts[vInd]);}

	SimEdge * idEdge( int id0, int id1 );
	SimEdge * vertexEdge( SimVertex * v0, SimVertex * v1 );
	bool isBoundary(SimEdge * e) {      return boundary[e->idx];      }
 
	std::vector<SimEdge *> getOneRingEdge(SimVertex * v);
	std::vector<SimFace *> getOneRingFace(SimVertex * v);

	SimVertex *	createVertex();		
	SimEdge *   createEdge(SimVertex * v0, SimVertex * v1);
	SimFace *		createFace();
	SimFace *		createFace(int v[]);
	SimFace *		createFace(SimVertex * v[]);	
	//(4) SimMesh Modification Operators

	void init(){   
		oneRingEdge.resize(m_verts.size());   
		oneRingFace.resize(m_verts.size());   
		boundary.resize(m_verts.size());   
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////								Variables										//////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////

protected:
	//stdext::hash_map<Vertex *, int> vertIndex;
	std::vector<SimEdge *>					m_edges;		// edge container
	std::vector<SimVertex *>				m_verts;		// vertex container
	std::vector<SimFace *>					m_faces;		// face container
	std::vector<bool>                       boundary;

	std::vector<std::vector<std::pair<int, int>>>  oneRingEdge;
	std::vector<std::vector<int>> oneRingFace;


protected:
	friend class MeshIO;
};


#endif