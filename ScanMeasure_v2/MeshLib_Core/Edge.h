#ifndef _EDGE_H_
#define _EDGE_H_

#include <string>

class Halfedge;

class Edge
{
public:
	Edge(){m_halfedge[0]=NULL;m_halfedge[1]=NULL;m_propertyIndex=-1;}
	Edge(Halfedge * he0, Halfedge * he1){m_halfedge[0]=he0;m_halfedge[1]=he1;m_propertyIndex=-1;}
	~Edge(){;}
	int & index() {return m_propertyIndex; }
	Halfedge * & he (int i) { return m_halfedge[i];}	
	bool boundary() { return (!m_halfedge[0] || !m_halfedge[1]); }		
	std::string & PropertyStr() { return m_propertyStr;}	
	Halfedge * & twin( Halfedge * he ) {return (he==m_halfedge[0])?(m_halfedge[1]):(m_halfedge[0]);}

protected:		
	Halfedge	*	m_halfedge[2];		// for boundary edge, m_halfedge[1]=NULL
	std::string		m_propertyStr;		// a string to store edge properties
	int				m_propertyIndex;	// index to Property array
};

#endif
