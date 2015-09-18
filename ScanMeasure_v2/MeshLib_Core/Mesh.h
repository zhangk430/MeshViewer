#ifndef _MESH_H_
#define _MESH_H_

#include "Edge.h"
#include "Face.h"
#include "Halfedge.h"
#include "Vertex.h"
#include "Point.h"
#include <hash_map>
#include <list>

class Mesh
{
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////								Methods										//////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
public:
	//(1) Constructor and Destructor
	Mesh();
	//Mesh::Mesh(std::vector<Face*>& faces);
	~Mesh();

	//(2) I/O
	int numVertices()	{return m_verts.size();}							//number of vertices
	int numEdges()		{return m_edges.size();}							//number of edges
	int numFaces()		{return m_faces.size();}							//number of faces
	void copyTo( Mesh & targetMesh );								//copy current mesh to the target mesh 
	bool read( const char inFile[]);								//read a mesh from inFile
	bool write( const char outFile[]);								//write a mesh to outFile
	void clear();

	//(3) BASIC OPERATIONS
	//Check whether an element is on the boundary:
	bool isBoundary( Vertex *  v ) const {return v->boundary();}
	bool isBoundary( Edge *    e ) const {return e->boundary();}
	bool isBoundary( Halfedge *  he ) const {return !(he->twin());}

	//indexing elements
	Edge *				idEdge( int id0, int id1 );
	Face *				idFace(const int id );		
	Halfedge *			idHalfedge( int srcVid, int trgVid );
	Vertex *			idVertex( int id );
	Edge *				vertexEdge( Vertex * v0, Vertex * v1 );
	Halfedge *			vertexHalfedge(Vertex * srcV, Vertex * trgV);
	Edge *				indEdge(unsigned int eInd){ return (eInd>=m_edges.size()?NULL:m_edges[eInd]);}
	Face *				indFace(unsigned int fInd) { return (fInd>=m_faces.size()?NULL:m_faces[fInd]);}
	Vertex *			indVertex(unsigned int vInd) { return (vInd>=m_verts.size()?NULL:m_verts[vInd]);}
protected:
	//(4) Mesh Modification Operators
	Vertex *	createVertex(int id);										
	Edge *		createEdge(Halfedge * he0, Halfedge * he1);
	Face *		createFace(int id);
	Face *		createFace(int v[], int id);
	Face *		createFace(Vertex * v[], int id);	
	void		LabelBoundaries();

	void IndexingHalfedges();
	////(6) Some Basic Mesh Processing Operations
	////	Flip the input edge, and return it.
	////	if the flip works, return the resultant edge, otherwise return NULL
	//Edge *				EdgeFlip(Edge * e);

	//// Split the given edge(input) into two. 
	////	  return :  newly created vertex or NULL(when the split fails);
	//Vertex *			EdgeSplit(Edge * e);
	//Vertex *			FaceSplit(Face * f, double bary[3]);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////								Variables										//////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////

protected:
	//stdext::hash_map<Vertex *, int> vertIndex;
	std::vector<Edge *>					m_edges;		// edge container
	std::vector<Vertex *>				m_verts;		// vertex container
	std::vector<Face *>					m_faces;		// face container

	stdext::hash_map<int, int>			vid2ind;
	stdext::hash_map<int, int>			fid2ind;

	std::vector<std::vector<Halfedge *>> v_adjInHEList;	

	bool boundaryLabeled;

protected:
	friend class MeshVertexIterator;
	friend class MeshEdgeIterator;
	friend class MeshFaceIterator;
	friend class MeshHalfedgeIterator;
	friend class MeshUtility;
	friend class MeshIO;
};



#endif //_XMESHLIB_CORE_MESH_H_ defined
