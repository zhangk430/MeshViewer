#ifndef _FACE_H_
#define _FACE_H_

class Halfedge;

class Face
{
public:
	Face(){ m_halfedge = NULL; m_propertyIndex=-1;}
	~Face(){;}
	Halfedge    *	& he() { return m_halfedge; }
	int				& id() { return m_id; }
	int				& index() {return m_propertyIndex; }
	std::string		& PropertyStr() { return m_propertyStr;}

protected:
	Halfedge	*	m_halfedge;
	int				m_id;
	std::string		m_propertyStr;
	int				m_propertyIndex; // index to Property array
};


#endif