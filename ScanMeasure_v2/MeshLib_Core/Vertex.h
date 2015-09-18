#ifndef _VERTEX_H_
#define _VERTEX_H_

#include "Halfedge.h"
#include "Point.h"

class Vertex
{
public:		
	Vertex(){ m_halfedge = NULL; m_boundary = false;m_propertyIndex=-1; }
	~Vertex(){;}

	Point & point() { return  m_point; }
	Halfedge * & he(){ return m_halfedge; }

	int & id() { return m_id; }
	int & index() {return m_propertyIndex; }	
	bool & boundary() { return m_boundary; }	//whether this is a boundary vertex
	std::string & PropertyStr() { return m_propertyStr;}

    Halfedge *  most_ccw_in_halfedge();
	Halfedge *  most_ccw_out_halfedge();
	Halfedge *  most_clw_in_halfedge();
	Halfedge *  most_clw_out_halfedge();

protected:
	//for half-edge data structure
	Point			m_point;
	Halfedge	*	m_halfedge;
	//optional
	int				m_id;		
	bool			m_boundary; // whether this is a boundary vertex
	std::string		m_propertyStr;
	int				m_propertyIndex; // index to Property array
};

#endif

